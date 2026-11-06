#pragma once

#include <stdint.h>

// Spectrum and sound-level analysis for the ES8311 microphone stream.
//
// FFT settings follow the measurement setup used as reference:
//   4096 points, Hann window, 50 % overlap, RMS amplitude scaling.
// With fs = 44.1 kHz that gives 10.77 Hz per bin.
//
// SPL is relative until the microphone is calibrated: a 94 dB SPL calibrator
// at 1 kHz gives the offset to subtract from dBFS. The default assumes a MEMS
// microphone with -26 dBFS sensitivity at 94 dB SPL (94 + 26 = 120 dB).
namespace spectrum {

constexpr uint16_t kFftSize = 4096;
constexpr uint16_t kBinCount = kFftSize / 2 + 1;
// 120 dB base + 20 dB sensitivity boost requested for the bench display.
constexpr float kSplOffsetDb = 140.0f;

void begin(float sample_rate_hz);

// Feed one block of raw codec samples (24-bit values in the low bits of each
// int32). Every block of kFftSize / 2 samples produces a new spectrum.
void pushSamples(const int32_t *samples, uint16_t n);

// True once after each new spectrum; cleared by this call.
bool hasNewSpectrum();

const float *dbBins();   // kBinCount one-sided bins, dB SPL, RMS scaling
const float *dbaBins();  // same bins with A-weighting applied, dBA
float binHz();
float spl();      // unweighted total level
float dba();      // A-weighted total level
float peakSpl();  // highest unweighted level since boot

}  // namespace spectrum
