#pragma once

#include <stdint.h>

// 2.8" 240x320 ILI9341V display on the ES3C28P board, used in landscape.
namespace display {

// Aligned live spectrum above a scrolling waterfall; readings/settings on the left.
void begin();

// Call from the same task as update(); Wire is shared with the audio codec.
void pollTouch();

// dba_bins: one-sided A-weighted spectrum (spectrum::dbaBins()).
void update(const float *dba_bins, uint16_t bin_count, float bin_hz,
            float dba, float spl, float peak);

}  // namespace display
