#pragma once
#include <math.h>
#include <stdint.h>

// Gate noise before tracking; never average together unrelated spectral peaks.
namespace display {
class PeakTracker {
 public:
  static constexpr float kMinLevel = 20.0f;
  static constexpr float kAcquireMargin = 10.0f;
  static constexpr float kKeepMargin = 6.0f;
  static constexpr uint8_t kConfirmFrames = 3;
  static constexpr uint32_t kLostMs = 500;

  bool valid = false;
  float hz = 0.0f;
  float dba = -INFINITY;

  bool expire(uint32_t now) {
    if (!valid || static_cast<uint32_t>(now - last_seen_) < kLostMs) return false;
    valid = false;
    hz = 0.0f;
    dba = -INFINITY;
    pending_frames_ = 0;
    return true;
  }

  void update(const float *bins, uint16_t count, float bin_hz, uint32_t now) {
    expire(now);
    if (!bins || count < 2 || !isfinite(bin_hz) || bin_hz <= 0.0f) {
      pending_frames_ = 0;
      return;
    }
    int peak = -1;
    float best = -INFINITY;
    for (uint16_t k = 1; k < count; ++k) {
      const float f = k * bin_hz;
      if (f >= 20.0f && f <= 20000.0f && isfinite(bins[k]) && bins[k] > best) {
        peak = k;
        best = bins[k];
      }
    }
    // Median of nearby bins, excluding the peak's Hann main lobe (+/-3 bins).
    // A narrow tone cannot pull this local noise estimate upwards.
    float noise[64];
    int n = 0;
    if (peak >= 0) {
      for (int k = peak - 32; k <= peak + 32; ++k) {
        if (k <= 0 || k >= count || (k >= peak - 3 && k <= peak + 3)) continue;
        const float f = k * bin_hz;
        if (f < 20.0f || f > 20000.0f || !isfinite(bins[k])) continue;
        int j = n++;
        while (j > 0 && noise[j - 1] > bins[k]) {
          noise[j] = noise[j - 1];
          --j;
        }
        noise[j] = bins[k];
      }
    }
    const float candidate_hz = peak * bin_hz;
    const bool nearby = valid && fabsf(candidate_hz - hz) <= 2.0f * bin_hz;
    const float required = nearby ? kKeepMargin : kAcquireMargin;
    if (peak < 0 || n < 8 || best < kMinLevel || best - noise[n / 2] < required) {
      pending_frames_ = 0;
      return;
    }
    if (nearby) {
      // Only smooth within two FFT bins, never through a jump to another tone.
      hz += 0.25f * (candidate_hz - hz);
      dba += 0.25f * (best - dba);
      last_seen_ = now;
      pending_frames_ = 0;
      return;
    }
    if (pending_frames_ && fabsf(candidate_hz - pending_hz_) <= 2.0f * bin_hz) {
      ++pending_frames_;
    } else {
      pending_hz_ = candidate_hz;
      pending_frames_ = 1;
    }
    if (pending_frames_ >= kConfirmFrames) {
      hz = candidate_hz;
      dba = best;
      valid = true;
      last_seen_ = now;
      pending_frames_ = 0;
    }
  }

 private:
  uint32_t last_seen_ = 0;
  float pending_hz_ = 0.0f;
  uint8_t pending_frames_ = 0;
};
}  // namespace display
