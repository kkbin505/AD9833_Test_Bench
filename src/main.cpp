#include "arduinoFFT.h"
#include <Arduino.h>
#include <SPI.h>

// Hardware Pin Definitions
const int FSYNC_PIN = 7;
const int SCK_PIN = 10;
const int MOSI_PIN = 11;
const int MISO_PIN = -1;
const int ADC_PIN = 2;

// Sampling Parameters (ADC DMA continuous mode)
const uint16_t SAMPLES = 2048;
const uint32_t SAMPLING_FREQUENCY = 50000; // 50 kSPS
const size_t DMA_BUFFER_SIZE = 1024;       // frames per DMA buffer
const size_t DMA_BUFFER_COUNT = 8;

// ---------------------------------------------------------------------------
// Binary streaming frame format (little-endian, 28-byte header + payload)
//   [ 0] u32 magic          0xAA55AD98
//   [ 4] u16 version        1
//   [ 6] u32 seq            frame sequence number (drop detection)
//   [10] u32 sampling_rate  -> pyfar.Signal sampling_rate
//   [14] u32 n_samples      -> pyfar.Signal time_data length
//   [18] u16 n_channels     1 (reserved)
//   [20] u32 time_stamp_us  esp_timer timestamp
//   [24] f32 full_scale     ADC full-scale reference (reserved for scaling)
//   [28] f32 x n_samples    time_data (raw ADC codes as float32)
// ---------------------------------------------------------------------------
const uint32_t FRAME_MAGIC = 0xAA55AD98;
const uint16_t FRAME_VERSION = 1;
const uint16_t FRAME_HEADER_SIZE = 28;
static uint32_t frame_seq = 0;

double vReal[SAMPLES];
double vImag[SAMPLES];

// Instantiate ArduinoFFT object
ArduinoFFT<double> FFT =
    ArduinoFFT<double>(vReal, vImag, SAMPLES, (double)SAMPLING_FREQUENCY);

void writeAD9833(uint16_t data) {
  digitalWrite(FSYNC_PIN, LOW);
  SPI.transfer16(data);
  digitalWrite(FSYNC_PIN, HIGH);
}

static void sendFrame(const double *samples, uint16_t n) {
  uint8_t header[FRAME_HEADER_SIZE];
  uint32_t ts = (uint32_t)esp_timer_get_time();

  header[0] = FRAME_MAGIC & 0xFF;
  header[1] = (FRAME_MAGIC >> 8) & 0xFF;
  header[2] = (FRAME_MAGIC >> 16) & 0xFF;
  header[3] = (FRAME_MAGIC >> 24) & 0xFF;
  header[4] = FRAME_VERSION & 0xFF;
  header[5] = (FRAME_VERSION >> 8) & 0xFF;
  uint32_t seq = frame_seq++;
  memcpy(&header[6], &seq, 4);
  uint32_t fs = SAMPLING_FREQUENCY;
  memcpy(&header[10], &fs, 4);
  uint32_t ns = n;
  memcpy(&header[14], &ns, 4);
  uint16_t nch = 1;
  memcpy(&header[18], &nch, 2);
  memcpy(&header[20], &ts, 4);
  float full_scale = 4096.0f; // LSB full-scale reference
  memcpy(&header[24], &full_scale, 4);

  static float payload[SAMPLES];
  for (int i = 0; i < n; i++)
    payload[i] = (float)samples[i];

  Serial.write(header, FRAME_HEADER_SIZE);
  Serial.write((uint8_t *)payload, n * sizeof(float));
  Serial.flush(); // CDC: flush after complete frame
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- AD9833 ADC DMA 50kSPS Streaming Bench ---");
  Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());

  pinMode(FSYNC_PIN, OUTPUT);
  digitalWrite(FSYNC_PIN, HIGH);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, FSYNC_PIN);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
  writeAD9833(0x2100);
  writeAD9833(0x69F1);
  writeAD9833(0x4000);
  writeAD9833(0xC000);
  writeAD9833(0x2000);
  SPI.endTransaction();

  // NOTE: do NOT call analogRead()/analogReadMilliVolts() here — it puts
  // ADC1 into oneshot mode and makes analogContinuous() abort.
  const uint8_t adc_pins[] = {ADC_PIN};
  analogContinuousSetAtten(ADC_11db); // full range
  analogContinuousSetWidth(12);       // 12-bit (0-4095)
  if (!analogContinuous(adc_pins, 1, 1, SAMPLING_FREQUENCY, NULL)) {
    Serial.println("# Error: analogContinuous init failed!");
  }
  analogContinuousStart();

  adc_continuous_result_t *dbg = NULL;
  for (int i = 0; i < 5; i++) {
    int dma_raw = -1, dma_mv = -1;
    if (analogContinuousRead(&dbg, 100)) {
      dma_raw = dbg->avg_read_raw;
      dma_mv = dbg->avg_read_mvolts;
    }
    Serial.printf("#   pin=%u ch=%u dma_raw=%d dma_mV=%d\n",
                  dbg ? dbg->pin : 255, dbg ? dbg->channel : 255, dma_raw,
                  dma_mv);
    delay(200);
  }
}

static int get_folded_bin(int bin, int N) {
  bin = bin % N;
  if (bin > N / 2) {
    bin = N - bin;
  }
  return bin;
}

void analyzeAndStream() {
  double *raw_samples = (double *)malloc(SAMPLES * sizeof(double));
  if (!raw_samples) {
    Serial.println("# Error: malloc failed");
    return;
  }

  // Collect one block from the DMA queue (already hardware-timed)
  adc_continuous_result_t *adc_result = NULL;
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < SAMPLES) {
    if (analogContinuousRead(&adc_result, 100)) {
      raw_samples[got++] = (double)adc_result->avg_read_raw;
    } else if (millis() - t0 > 2000) {
      Serial.println("# Error: ADC DMA timeout");
      free(raw_samples);
      return;
    }
  }

  // --- Stream raw block to host (binary frame) ---
  sendFrame(raw_samples, SAMPLES);

  // --- On-board IEEE 1241 analysis (text output, '#' prefixed) ---
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = raw_samples[i];
    vImag[i] = 0.0;
  }
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);

  double max_mag_sq = 0.0;
  int fund_bin = 0;
  for (int i = 4; i < SAMPLES / 2; i++) {
    double mag_sq = vReal[i] * vReal[i] + vImag[i] * vImag[i];
    if (mag_sq > max_mag_sq) {
      max_mag_sq = mag_sq;
      fund_bin = i;
    }
  }

  double omega = 2.0 * PI * fund_bin / SAMPLES;
  double A = 0, B = 0, C = 0; // Model: y = A*cos(w*n) + B*sin(w*n) + C

  auto solve3 = [&](double w, double &a_out, double &b_out, double &c_out) {
    double m[3][3] = {0};
    double yv[3] = {0};
    for (int n = 0; n < SAMPLES; n++) {
      double c = cos(w * n), s = sin(w * n), y = raw_samples[n];
      m[0][0] += c * c;
      m[0][1] += c * s;
      m[0][2] += c;
      m[1][0] += c * s;
      m[1][1] += s * s;
      m[1][2] += s;
      m[2][0] += c;
      m[2][1] += s;
      m[2][2] += 1.0;
      yv[0] += y * c;
      yv[1] += y * s;
      yv[2] += y;
    }
    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                 m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    a_out = (yv[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
             m[0][1] * (yv[1] * m[2][2] - m[1][2] * yv[2]) +
             m[0][2] * (yv[1] * m[2][1] - m[1][1] * yv[2])) /
            det;
    b_out = (m[0][0] * (yv[1] * m[2][2] - m[1][2] * yv[2]) -
             yv[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
             m[0][2] * (m[1][0] * yv[2] - yv[1] * m[2][0])) /
            det;
    c_out = (m[0][0] * (m[1][1] * yv[2] - yv[1] * m[2][1]) -
             m[0][1] * (m[1][0] * yv[2] - yv[1] * m[2][0]) +
             yv[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) /
            det;
  };

  solve3(omega, A, B, C);

  // 4-Parameter Iteration (Newton-Raphson)
  for (int iter = 0; iter < 10; iter++) {
    double m[4][4] = {0};
    double yv[4] = {0};
    for (int n = 0; n < SAMPLES; n++) {
      double arg = omega * n;
      double c_val = cos(arg), s_val = sin(arg);
      double d_omega = -A * n * s_val + B * n * c_val;
      double y_est = A * c_val + B * s_val + C;
      double res = raw_samples[n] - y_est;

      double row[4] = {c_val, s_val, 1.0, d_omega};
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
          m[i][j] += row[i] * row[j];
        yv[i] += res * row[i];
      }
    }

    for (int i = 0; i < 4; i++) {
      int pivot = i;
      for (int j = i + 1; j < 4; j++)
        if (fabs(m[j][i]) > fabs(m[pivot][i]))
          pivot = j;
      for (int j = 0; j < 4; j++)
        std::swap(m[i][j], m[pivot][j]);
      std::swap(yv[i], yv[pivot]);
      double piv = m[i][i];
      if (fabs(piv) < 1e-15)
        continue;
      for (int j = i; j < 4; j++)
        m[i][j] /= piv;
      yv[i] /= piv;
      for (int j = 0; j < 4; j++) {
        if (i != j) {
          double factor = m[j][i];
          for (int k = i; k < 4; k++)
            m[j][k] -= factor * m[i][k];
          yv[j] -= factor * yv[i];
        }
      }
    }
    A += yv[0];
    B += yv[1];
    C += yv[2];
    omega += yv[3];
    if (fabs(yv[3]) < 1e-10)
      break;
  }

  double amplitude = sqrt(A * A + B * B);
  double phase = atan2(-B, A);

  Serial.println("# --- IEEE 1241 4-Param Sine Fit ---");
  Serial.printf("# Fitted Frequency  : %.4f Hz\n",
                (omega * SAMPLING_FREQUENCY) / (2.0 * PI));
  Serial.printf("# Refined DC Offset : %.3f LSB\n", C);
  Serial.printf("# Fitted Amplitude  : %.3f LSB\n", amplitude);
  Serial.printf("# Fitted Phase(rad) : %.4f\n", phase);

  // --- Golden Reference & LUT Training ---
  double *ideal_samples = (double *)malloc(SAMPLES * sizeof(double));
  const int ADC_MAX_CODE = 4096;
  double *LUT = (double *)calloc(ADC_MAX_CODE, sizeof(double));
  int *count = (int *)calloc(ADC_MAX_CODE, sizeof(int));
  if (!ideal_samples || !LUT || !count) {
    free(raw_samples);
    return;
  }

  for (int n = 0; n < SAMPLES; n++)
    ideal_samples[n] = amplitude * cos(omega * n + phase);

  for (int n = 0; n < SAMPLES; n++) {
    int code = (int)raw_samples[n];
    if (code < 0)
      code = 0;
    if (code >= ADC_MAX_CODE)
      code = ADC_MAX_CODE - 1;
    LUT[code] += (ideal_samples[n] - (raw_samples[n] - C));
    count[code]++;
  }
  for (int i = 0; i < ADC_MAX_CODE; i++)
    if (count[i] > 0)
      LUT[i] /= count[i];

  // --- Calibrated metrics ---
  double *calibrated_samples = (double *)malloc(SAMPLES * sizeof(double));
  for (int n = 0; n < SAMPLES; n++) {
    int code = (int)raw_samples[n];
    if (code < 0)
      code = 0;
    if (code >= ADC_MAX_CODE)
      code = ADC_MAX_CODE - 1;
    calibrated_samples[n] = (raw_samples[n] - C) + LUT[code];
    vReal[n] = calibrated_samples[n];
    vImag[n] = 0.0;
  }

  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  const int num_bins = SAMPLES / 2;
  double *pow_spec = (double *)malloc(num_bins * sizeof(double));
  double *noise_m = (double *)malloc(num_bins * sizeof(double));
  for (int i = 0; i < num_bins; i++) {
    pow_spec[i] = vReal[i] * vReal[i];
    noise_m[i] = 1.0;
  }

  int ws = 3;
  double p_sig = 0, p_harm = 0, p_noi = 0, p_spur = 0;
  int sig_b = floor((omega * SAMPLES) / (2.0 * PI) + 0.5);
  for (int i = sig_b - ws; i <= sig_b + ws; i++)
    if (i >= 0 && i < num_bins) {
      p_sig += pow_spec[i];
      noise_m[i] = 0;
    }
  for (int i = 0; i <= ws; i++)
    noise_m[i] = 0;

  for (int h = 2; h <= 6; h++) {
    int hb = get_folded_bin(sig_b * h, SAMPLES);
    for (int i = hb - ws; i <= hb + ws; i++)
      if (i >= 0 && i < num_bins) {
        if (noise_m[i] == 1.0)
          p_harm += pow_spec[i];
        noise_m[i] = 0;
      }
  }

  double pmax = 0;
  int spb = 0;
  for (int i = 0; i < num_bins; i++)
    if (noise_m[i] == 1.0) {
      p_noi += pow_spec[i];
      if (pow_spec[i] > pmax) {
        pmax = pow_spec[i];
        spb = i;
      }
    }
  for (int i = spb - ws; i <= spb + ws; i++)
    if (i >= 0 && i < num_bins && noise_m[i] == 1.0)
      p_spur += pow_spec[i];

  double snr = 10 * log10(p_sig / p_noi), thd = 10 * log10(p_harm / p_sig),
         sndr = 10 * log10(p_sig / (p_noi + p_harm)),
         enob = (sndr - 1.76) / 6.02;

  Serial.println("# --- Calibrated Metrics ---");
  Serial.printf("# SNR: %.2f dB, THD: %.2f dB, SNDR: %.2f dB, ENOB: %.2f "
                "bits\n",
                snr, thd, sndr, enob);
  Serial.println("# =========================");

  free(raw_samples);
  free(ideal_samples);
  free(LUT);
  free(count);
  free(calibrated_samples);
  free(pow_spec);
  free(noise_m);
}

void loop() { analyzeAndStream(); }
