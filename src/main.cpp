#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>

#include "display.h"
#include "spectrum.h"

// ES3C28P / ES3N28P 2.8" ESP32-S3 display module (CYD branch).
// Microphone path: ES8311 codec (ADC) -> I2S -> ESP32-S3, streamed over native USB CDC.
//
//   ES8311 control (I2C, 0x18)  SDA -> GPIO16, SCL -> GPIO15
//   MCLK  (ESP32 -> ES8311)     GPIO4
//   BCLK  (ESP32 -> ES8311)     GPIO5
//   LRCK  (ESP32 -> ES8311)     GPIO7
//   SDOUT (ES8311 -> ESP32)     GPIO6
//   Speaker amp enable          GPIO1 (active low, held off)
//
// Pin source: doc/ES3C28P_pinout.md in SimpleTrack-Logger.
// The ES8311 is an I2S slave: the ESP32 supplies MCLK, BCLK and LRCK, and the
// codec must be configured over I2C before any audio appears on SDOUT.
//
// The firmware reads 24-bit PCM from the codec, packs it as signed
// little-endian 24-bit samples, and streams binary frames to the PC over USB.

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int I2S_MCLK_PIN = 4;
constexpr int I2S_BCLK_PIN = 5;
constexpr int I2S_WS_PIN = 7;
constexpr int I2S_DATA_PIN = 6;
constexpr int AMP_EN_PIN = 1;          // active low
constexpr int CODEC_SDA_PIN = 16;
constexpr int CODEC_SCL_PIN = 15;
constexpr uint8_t ES8311_ADDR = 0x18;
constexpr bool MIC_USE_LEFT_CHANNEL = true; // ES8311 is mono; data on left slot

constexpr uint32_t SAMPLE_RATE_HZ = 44100;
constexpr uint32_t MCLK_MULTIPLE = 256;     // MCLK = 256 * fs, standard for ES8311
constexpr uint16_t SAMPLES_PER_FRAME = 2048;
constexpr uint16_t CHANNELS = 1;

// ES8311 analog mic gain settings. The values come from the ESPHome ES8311
// driver; they have not been checked against the ES8311 datasheet or measured.
constexpr uint8_t ES8311_REG_MIC_PGA = 0x14; // analog MIC, PGA gain
constexpr uint8_t ES8311_MIC_PGA_VAL = 0x1A;
constexpr uint8_t ES8311_REG_MIC_GAIN = 0x16; // ADC mic gain (0 dB assumed)
constexpr uint8_t ES8311_MIC_GAIN_VAL = 0x00;
constexpr uint8_t ES8311_REG_ADC_VOL = 0x17;
constexpr uint8_t ES8311_ADC_VOL_VAL = 0xC8;

// Binary streaming frame format:
//   [ 0] u32 magic         0xAA55AD98
//   [ 4] u16 version       3
//   [ 6] u32 seq           frame sequence number
//   [10] u32 sampling_rate 44100
//   [14] u32 n_samples     per channel
//   [18] u16 n_channels    1
//   [20] u32 time_stamp_us esp_timer timestamp
//   [24] f32 full_scale    2^23 for signed 24-bit PCM
//   [28] u32 payload_bytes packed payload size in bytes
//   [32] 24-bit payload    packed little-endian PCM samples
constexpr uint32_t FRAME_MAGIC = 0xAA55AD98;
constexpr uint16_t FRAME_VERSION = 3;
constexpr uint16_t FRAME_HEADER_SIZE = 32;
constexpr float MIC_FULL_SCALE = 8388608.0f; // 2^23

// Streaming every frame over native USB CDC paces the capture loop: one 6 KB
// frame takes ~64 ms to drain, which is longer than the 46 ms it takes to
// record it. Keep it off while the on-screen meter runs; turn it on for bench
// captures with python/ad9833_stream.py.
constexpr bool USB_STREAM_ENABLE = false;

static uint32_t frame_seq = 0;
static int32_t sample_buffer[SAMPLES_PER_FRAME];
static uint8_t payload[SAMPLES_PER_FRAME * 3];

// Capture runs on the Arduino core-1 task so I2S is never starved; the FFT and
// display run on core 0. A one-deep queue means the analyser always works on
// the newest block and simply skips blocks it cannot keep up with.
static QueueHandle_t audio_queue = nullptr;
static int32_t analysis_block[SAMPLES_PER_FRAME];  // owned by analysisTask

// Timing instrumentation, printed once every 5 s.
static volatile uint32_t stat_analysis_us = 0;

static void analysisTask(void *) {
  for (;;) {
    display::pollTouch();
    if (xQueueReceive(audio_queue, analysis_block, pdMS_TO_TICKS(20)) != pdTRUE) {
      continue;
    }
    const int64_t t0 = esp_timer_get_time();
    spectrum::pushSamples(analysis_block, SAMPLES_PER_FRAME);
    if (spectrum::hasNewSpectrum()) {
      display::update(spectrum::dbaBins(), spectrum::kBinCount,
                      spectrum::binHz(), spectrum::dba(), spectrum::spl(),
                      spectrum::peakSpl());
    }
    stat_analysis_us = static_cast<uint32_t>(esp_timer_get_time() - t0);
    // Keep the core-0 idle task alive so the task watchdog does not fire.
    vTaskDelay(1);
  }
}

static void packInt24LE(uint8_t *dst, int32_t sample) {
  dst[0] = static_cast<uint8_t>(sample & 0xFF);
  dst[1] = static_cast<uint8_t>((sample >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((sample >> 16) & 0xFF);
}

static void sendFrame24(const uint8_t *packed_samples, uint16_t n_samples) {
  uint8_t header[FRAME_HEADER_SIZE];
  uint32_t ts = static_cast<uint32_t>(esp_timer_get_time());

  header[0] = FRAME_MAGIC & 0xFF;
  header[1] = (FRAME_MAGIC >> 8) & 0xFF;
  header[2] = (FRAME_MAGIC >> 16) & 0xFF;
  header[3] = (FRAME_MAGIC >> 24) & 0xFF;
  header[4] = FRAME_VERSION & 0xFF;
  header[5] = (FRAME_VERSION >> 8) & 0xFF;

  uint32_t seq = frame_seq++;
  memcpy(&header[6], &seq, sizeof(seq));

  uint32_t fs = SAMPLE_RATE_HZ;
  memcpy(&header[10], &fs, sizeof(fs));

  uint32_t ns = n_samples;
  memcpy(&header[14], &ns, sizeof(ns));

  uint16_t nch = CHANNELS;
  memcpy(&header[18], &nch, sizeof(nch));

  memcpy(&header[20], &ts, sizeof(ts));
  memcpy(&header[24], &MIC_FULL_SCALE, sizeof(MIC_FULL_SCALE));
  uint32_t payload_bytes = n_samples * CHANNELS * 3;
  memcpy(&header[28], &payload_bytes, sizeof(payload_bytes));

  Serial.write(header, FRAME_HEADER_SIZE);
  Serial.write(packed_samples, payload_bytes);
  Serial.flush();
}

static bool es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Register sequence follows the ESPHome ES8311 driver for MCLK = 256 * fs.
// Clock divider values for 44.1 kHz with MCLK = 11.2896 MHz are my reading of
// that driver; they are not confirmed against the datasheet. Verify on hardware.
static bool setupES8311() {
  bool ok = true;

  ok &= es8311Write(0x00, 0x1F); // reset
  delay(20);
  ok &= es8311Write(0x00, 0x00); // release reset

  // Clock: MCLK from the MCLK pin, pre_div = 1, pre_mult = 1, single-speed
  ok &= es8311Write(0x01, 0x3F);
  ok &= es8311Write(0x02, 0x00);
  ok &= es8311Write(0x03, 0x10); // fs_mode = 0, adc_osr
  ok &= es8311Write(0x04, 0x10); // dac_osr
  ok &= es8311Write(0x05, 0x00); // adc_div = 1, dac_div = 1
  ok &= es8311Write(0x06, 0x03); // bclk_div = 4 (BCLK = MCLK / 4 = 64 * fs)
  ok &= es8311Write(0x07, 0x00); // LRCK divider high
  ok &= es8311Write(0x08, 0xFF); // LRCK divider low (256 * fs MCLK)

  // Format: 24-bit in and out, I2S
  ok &= es8311Write(0x09, 0x00);
  ok &= es8311Write(0x0A, 0x00);

  // Analog microphone input
  ok &= es8311Write(ES8311_REG_MIC_PGA, ES8311_MIC_PGA_VAL);
  ok &= es8311Write(ES8311_REG_MIC_GAIN, ES8311_MIC_GAIN_VAL);
  ok &= es8311Write(ES8311_REG_ADC_VOL, ES8311_ADC_VOL_VAL);

  // Power up
  ok &= es8311Write(0x0D, 0x01);
  ok &= es8311Write(0x0E, 0x02);
  ok &= es8311Write(0x12, 0x00);
  ok &= es8311Write(0x13, 0x10);
  ok &= es8311Write(0x1C, 0x6A);
  ok &= es8311Write(0x37, 0x08);
  ok &= es8311Write(0x00, 0x80); // power-on

  return ok;
}

static bool setupI2SMic() {
  i2s_config_t config;
  memset(&config, 0, sizeof(config));
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = SAMPLE_RATE_HZ;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = MIC_USE_LEFT_CHANNEL ? I2S_CHANNEL_FMT_ONLY_LEFT
                                               : I2S_CHANNEL_FMT_ONLY_RIGHT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 16;
  config.dma_buf_len = 256;
  config.use_apll = true;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = SAMPLE_RATE_HZ * MCLK_MULTIPLE; // MCLK output for the codec

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.mck_io_num = I2S_MCLK_PIN;
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num = I2S_WS_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_DATA_PIN;

  esp_err_t err = i2s_driver_install(I2S_PORT, &config, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("# error: i2s_driver_install failed: %d\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) {
    Serial.printf("# error: i2s_set_pin failed: %d\n", err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

static bool readMicFrame(int32_t *dst, size_t n_samples) {
  size_t bytes_needed = n_samples * sizeof(int32_t);
  size_t bytes_read = 0;
  uint8_t *write_ptr = reinterpret_cast<uint8_t *>(dst);

  while (bytes_read < bytes_needed) {
    size_t chunk = 0;
    esp_err_t err = i2s_read(I2S_PORT, write_ptr + bytes_read,
                             bytes_needed - bytes_read, &chunk, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("# error: i2s_read failed: %d\n", err);
      return false;
    }
    if (chunk == 0) {
      continue;
    }
    bytes_read += chunk;
  }
  return true;
}

static bool captureFrame() {
  if (!readMicFrame(sample_buffer, SAMPLES_PER_FRAME)) {
    delay(10);
    return false;
  }

  if (USB_STREAM_ENABLE) {
    for (uint16_t i = 0; i < SAMPLES_PER_FRAME; i++) {
      // The codec sends 24-bit samples left-justified in a 32-bit I2S slot.
      int32_t sample24 = sample_buffer[i] >> 8;
      packInt24LE(&payload[i * 3], sample24);
    }
    sendFrame24(payload, SAMPLES_PER_FRAME);
  }
  return true;
}

void setup() {
  pinMode(AMP_EN_PIN, OUTPUT);
  digitalWrite(AMP_EN_PIN, HIGH); // keep the speaker amplifier off

  Serial.begin(2000000);
  delay(1000);
  Serial.println("\n# ESP32-S3 CYD (ES3C28P) ES8311 24-bit / 44.1kHz stream");
  Serial.printf("# MCLK=%d BCLK=%d WS=%d SD=%d channel=%s\n", I2S_MCLK_PIN,
                I2S_BCLK_PIN, I2S_WS_PIN, I2S_DATA_PIN,
                MIC_USE_LEFT_CHANNEL ? "LEFT" : "RIGHT");
  Serial.printf("# sample_rate=%u frame_samples=%u full_scale=%.0f\n",
                SAMPLE_RATE_HZ, SAMPLES_PER_FRAME, MIC_FULL_SCALE);

  Wire.begin(CODEC_SDA_PIN, CODEC_SCL_PIN, 400000);
  if (!setupI2SMic()) {
    Serial.println("# fatal: I2S init failed");
    while (true) {
      delay(1000);
    }
  }

  if (!setupES8311()) {
    Serial.println("# fatal: ES8311 I2C configuration failed (check 0x18 wiring)");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("# ES8311 configured");

  spectrum::begin(SAMPLE_RATE_HZ);
  display::begin();

  audio_queue = xQueueCreate(1, sizeof(analysis_block));
  if (audio_queue == nullptr) {
    Serial.println("# fatal: could not create audio queue");
    while (true) {
      delay(1000);
    }
  }
  xTaskCreatePinnedToCore(analysisTask, "analysis", 8192, nullptr, 1, nullptr, 0);

  Serial.println("# display ready");
}

void loop() {
  const int64_t t0 = esp_timer_get_time();
  if (!captureFrame()) {
    return;
  }
  const int64_t t1 = esp_timer_get_time();

  // Overwrite: the analyser always gets the most recent block, and capture
  // never waits for it.
  xQueueOverwrite(audio_queue, sample_buffer);

  // Throughput check: 46.4 ms per 2048-sample block means 21.5 frames/s.
  static uint32_t frames = 0;
  static uint32_t last_report = 0;
  static uint64_t cap_us = 0;
  ++frames;
  cap_us += static_cast<uint64_t>(t1 - t0);
  const uint32_t now = millis();
  if (now - last_report >= 5000) {
    const float fps = frames * 1000.0f / (now - last_report);
    Serial.printf("# fps=%.1f capture=%.1fms analysis=%.1fms dba=%.1f spl=%.1f\n",
                  fps, cap_us / 1000.0f / frames,
                  stat_analysis_us / 1000.0f, spectrum::dba(),
                  spectrum::spl());
    frames = 0;
    cap_us = 0;
    last_report = now;
  }
}
