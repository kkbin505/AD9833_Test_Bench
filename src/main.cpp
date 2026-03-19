#include "arduinoFFT.h"
#include <Arduino.h>
#include <SPI.h>

// Hardware Pin Definitions
const int FSYNC_PIN = 7; // Frame Synchronization (Chip Select)
const int SCK_PIN = 10;  // Serial Clock
const int MOSI_PIN = 11; // Master Out Slave In (Data out to AD9833)
const int MISO_PIN = -1; // AD9833 has no MISO output, set to -1
const int ADC_PIN = 2;   // GPIO 2 for ADC sampling

// FFT Parameters
const uint16_t SAMPLES = 1024;
const double SAMPLING_FREQUENCY = 10000.0; // 10kHz sampling frequency
const unsigned long SAMPLING_PERIOD_US = round(1000000.0 / SAMPLING_FREQUENCY);

double vReal[SAMPLES];
double vImag[SAMPLES];

// Instantiate ArduinoFFT object using templates (arduinoFFT v2.x)
ArduinoFFT<double> FFT =
    ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

/**
 * Sends a 16-bit data word to the AD9833 via SPI.
 * @param data The 16-bit word to transmit.
 */
void writeAD9833(uint16_t data) {
  digitalWrite(FSYNC_PIN, LOW);  // Pull FSYNC low to start transmission
  SPI.transfer16(data);          // Send 16-bit data
  digitalWrite(FSYNC_PIN, HIGH); // Pull FSYNC high to end transmission
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- AD9833 & ADC FFT Test Bench ---");
  Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
  Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());

  pinMode(FSYNC_PIN, OUTPUT);
  digitalWrite(FSYNC_PIN, HIGH);

  // Initialize the SPI interface for ESP32-S3
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, FSYNC_PIN);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
  writeAD9833(0x2100);
  writeAD9833(0x69F1);
  writeAD9833(0x4000);
  writeAD9833(0xC000);
  writeAD9833(0x2000);
  SPI.endTransaction();

  // Configure ADC pin
  pinMode(ADC_PIN, INPUT);
  analogReadResolution(12); // ESP32-S3 typically uses 12-bit ADC
}

void analyzeADC() {
  Serial.println("Starting ADC sampling...");
  double sumBefore = 0;

  // Sample the signal
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long startTime = micros();
    double val = analogRead(ADC_PIN);
    vReal[i] = val;
    vImag[i] = 0.0;
    sumBefore += val;
    while ((micros() - startTime) < SAMPLING_PERIOD_US) {
      // Wait for next sample interval based on SAMPLING_FREQUENCY
    }
  }

  // Remove DC bias (average value)
  double mean = sumBefore / SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] -= mean;
  }

  // Apply Windowing to minimize spectral leakage (Blackman-Harris is good for
  // high dynamic range)
  FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);

  // Compute FFT
  FFT.compute(FFTDirection::Forward);

  // Convert complex to magnitude
  FFT.complexToMagnitude();

  // Variables to calculate SNR, SNDR, ENOB
  double max_mag = 0;
  int fund_bin = 0;

  // Find fundamental frequency bin (ignore very low frequency noise and DC
  // around bin 0-2)
  for (int i = 3; i < (SAMPLES / 2); i++) {
    if (vReal[i] > max_mag) {
      max_mag = vReal[i];
      fund_bin = i;
    }
  }

  double signal_energy = 0;
  double noise_energy = 0;
  double distortion_energy = 0;
  double max_spur_energy = 0;

  // Use squared magnitude for power/energy calculations
  int wing_size = 4; // Spectral leakage wings around the peak

  for (int i = 3; i < (SAMPLES / 2); i++) {
    double energy = vReal[i] * vReal[i];

    // Check if within fundamental signal band
    if (i >= fund_bin - wing_size && i <= fund_bin + wing_size) {
      signal_energy += energy;
    } else {
      // Find maximum spur energy for SFDR
      if (energy > max_spur_energy) {
        max_spur_energy = energy;
      }

      // Check if it's a harmonic (2nd to 10th harmonics typical for distortion
      // calculation)
      bool isHarmonic = false;
      for (int h = 2; h <= 10; h++) {
        int harm_bin = fund_bin * h;
        if (harm_bin >= (SAMPLES / 2))
          break;
        if (i >= harm_bin - wing_size && i <= harm_bin + wing_size) {
          isHarmonic = true;
          break;
        }
      }

      if (isHarmonic) {
        distortion_energy += energy;
      } else {
        noise_energy += energy;
      }
    }
  }

  // Prevent divide by zero issues
  if (noise_energy <= 0)
    noise_energy = 1e-10;
  if (distortion_energy <= 0)
    distortion_energy = 1e-10;
  if (signal_energy <= 0)
    signal_energy = 1e-10;
  if (max_spur_energy <= 0)
    max_spur_energy = 1e-10;

  double snr = 10 * log10(signal_energy / noise_energy);
  double thd = 10 * log10(distortion_energy / signal_energy);
  double sndr = 10 * log10(signal_energy / (noise_energy + distortion_energy));
  double enob = (sndr - 1.76) / 6.02;
  double sfdr = 10 * log10((max_mag * max_mag) / max_spur_energy);

  double fund_freq = (fund_bin * SAMPLING_FREQUENCY) / SAMPLES;

  Serial.println("--- Analysis Results ---");
  Serial.printf("Fundamental Freq: %.2f Hz\n", fund_freq);
  Serial.printf("SNR:              %.2f dB\n", snr);
  Serial.printf("THD:              %.2f dB\n", thd);
  Serial.printf("SNDR(SINAD):      %.2f dB\n", sndr);
  Serial.printf("ENOB:             %.2f bits\n", enob);
  Serial.printf("SFDR:             %.2f dBc\n", sfdr);
  Serial.println("------------------------\n");
}

void loop() {
  analyzeADC();
  delay(3000); // Wait 3 seconds before next capture and analysis
}