#include "spectrum.h"

#include <math.h>
#include <string.h>

#include <arduinoFFT.h>

namespace spectrum {
namespace {

// Mean of w[n]^2 for a periodic Hann window. Used to recover the mean square
// of the unwindowed signal from the windowed FFT (Parseval).
constexpr float kHannMeanSquare = 0.375f;

float gRaw[kFftSize];    // unwindowed samples, kept for the 50 % overlap
float gReal[kFftSize];
float gImag[kFftSize];
float gMag[kBinCount];   // raw FFT magnitude, not normalised
float gDb[kBinCount];
float gDbA[kBinCount];
float gAWeight[kBinCount];
float gAWeightDb[kBinCount];
float gBinHz = 44100.0f / kFftSize;
bool gFresh = false;
float gSplDb = 0.0f;
float gDbaDb = 0.0f;
float gPeakDb = -200.0f;

ArduinoFFT<float> gFft(gReal, gImag, kFftSize, 44100.0f);

void computeAWeights(float bin_hz) {
  // IEC 61672 A-weighting, normalised to 0 dB at 1 kHz.
  for (uint16_t k = 0; k < kBinCount; ++k) {
    const double f = k * static_cast<double>(bin_hz);
    if (f < 10.0) {
      gAWeight[k] = 0.0f;
      gAWeightDb[k] = -100.0f;
      continue;
    }
    const double f2 = f * f;
    const double num = 12194.0 * 12194.0 * f2 * f2;
    const double den = (f2 + 20.6 * 20.6) *
                       sqrt((f2 + 107.7 * 107.7) * (f2 + 737.9 * 737.9)) *
                       (f2 + 12194.0 * 12194.0);
    const double a_db = 20.0 * log10(num / den) + 2.0;
    gAWeightDb[k] = static_cast<float>(a_db);
    gAWeight[k] = static_cast<float>(pow(10.0, a_db / 10.0));
  }
}

}  // namespace

void begin(float sample_rate_hz) {
  gBinHz = sample_rate_hz / kFftSize;
  computeAWeights(gBinHz);
  memset(gRaw, 0, sizeof(gRaw));
  memset(gReal, 0, sizeof(gReal));
  memset(gImag, 0, sizeof(gImag));
  gFresh = false;
  gSplDb = 0.0f;
  gDbaDb = 0.0f;
  gPeakDb = -200.0f;
}

void pushSamples(const int32_t *samples, uint16_t n) {
  if (n > kFftSize / 2) {
    n = kFftSize / 2;
  }

  // New samples always land in the second half, so consecutive spectra share
  // exactly half of their input.
  for (uint16_t i = 0; i < n; ++i) {
    gRaw[kFftSize / 2 + i] =
        static_cast<float>(samples[i] >> 8) * (1.0f / 8388608.0f);
  }

  memcpy(gReal, gRaw, sizeof(gReal));
  memset(gImag, 0, sizeof(gImag));

  gFft.windowing(FFTWindow::Hann, FFTDirection::Forward);
  gFft.compute(FFTDirection::Forward);
  gFft.complexToMagnitude();  // raw |X[k]| in gReal[0..kFftSize/2]

  for (uint16_t k = 0; k < kBinCount; ++k) {
    gMag[k] = gReal[k];
  }

  // RMS amplitude scaling: a Hann window has a coherent gain of 0.5, so a
  // sine of amplitude A gives |X| = A*N/4. RMS is A/sqrt(2).
  const float rms_scale = 2.0f * 1.41421356f / static_cast<float>(kFftSize);
  for (uint16_t k = 0; k < kBinCount; ++k) {
    const float rms = gMag[k] * rms_scale;
    gDb[k] = 20.0f * log10f(fmaxf(rms, 1e-9f)) + kSplOffsetDb;
    gDbA[k] = gDb[k] + gAWeightDb[k];
  }
  gDb[0] = gDb[1];   // DC is not meaningful for a level display
  gDbA[0] = gDbA[1];

  // Total levels from Parseval. The one-sided sum doubles every bin except
  // DC and Nyquist; DC is skipped because the codec already cancels it.
  const double norm = 1.0 / (static_cast<double>(kFftSize) * kFftSize *
                             kHannMeanSquare);
  double acc = 0.0;
  double acc_a = 0.0;
  for (uint16_t k = 1; k < kBinCount - 1; ++k) {
    const double p = static_cast<double>(gMag[k]) * gMag[k];
    acc += 2.0 * p;
    acc_a += 2.0 * p * gAWeight[k];
  }
  const double nyq = static_cast<double>(gMag[kBinCount - 1]) *
                     gMag[kBinCount - 1];
  acc += nyq;
  acc_a += nyq * gAWeight[kBinCount - 1];

  const double ms = acc * norm;
  const double ms_a = acc_a * norm;
  gSplDb = static_cast<float>(10.0 * log10(fmax(ms, 1e-18))) + kSplOffsetDb;
  gDbaDb = static_cast<float>(10.0 * log10(fmax(ms_a, 1e-18))) + kSplOffsetDb;
  if (gSplDb > gPeakDb) {
    gPeakDb = gSplDb;
  }

  // The second half becomes the first half of the next window.
  memmove(gRaw, gRaw + kFftSize / 2, sizeof(float) * (kFftSize / 2));
  gFresh = true;
}

bool hasNewSpectrum() {
  const bool fresh = gFresh;
  gFresh = false;
  return fresh;
}

const float *dbBins() { return gDb; }
const float *dbaBins() { return gDbA; }
float binHz() { return gBinHz; }
float spl() { return gSplDb; }
float dba() { return gDbaDb; }
float peakSpl() { return gPeakDb; }

}  // namespace spectrum
