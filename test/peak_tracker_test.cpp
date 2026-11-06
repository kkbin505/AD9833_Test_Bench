#include "../src/peak_tracker.h"

extern "C" int runPeakTests() {
  using display::PeakTracker;
  static float bins[2049];
  for (int k = 0; k < 2049; ++k) bins[k] = 15.0f;
  PeakTracker t;
  // Flat background and a peak below the prominence gate must not acquire.
  for (unsigned ms = 0; ms < 500; ms += 50) t.update(bins, 2049, 10, ms);
  if (t.valid) return 1;
  bins[50] = 23;
  for (unsigned ms = 500; ms < 1000; ms += 50) t.update(bins, 2049, 10, ms);
  if (t.valid) return 2;
  // Three consistent frames acquire a tone, never the first two.
  bins[50] = 36;
  t.update(bins, 2049, 10, 1000);
  t.update(bins, 2049, 10, 1050);
  if (t.valid) return 3;
  t.update(bins, 2049, 10, 1100);
  if (!t.valid || t.hz != 500 || t.dba != 36) return 4;
  // A single louder distant peak must not move the RPM/marker.
  bins[100] = 45;
  t.update(bins, 2049, 10, 1150);
  if (t.hz != 500) return 5;
  bins[100] = 15;
  // Maintain an acquired tone at 7 dB margin (hysteresis).
  bins[50] = 22;
  t.update(bins, 2049, 10, 1200);
  if (!t.valid || t.hz != 500) return 6;
  // Short losses hold; 500 ms without confirmation clears.
  bins[50] = 15;
  t.update(bins, 2049, 10, 1500);
  if (!t.valid) return 7;
  if (!t.expire(1700) || t.valid || t.hz != 0) return 8;
  // Alternating unrelated noise peaks cannot pass consecutive confirmation.
  for (unsigned ms = 1800; ms < 2800; ms += 50) {
    bins[50] = ((ms / 50) & 1) ? 40 : 15;
    bins[100] = ((ms / 50) & 1) ? 15 : 40;
    t.update(bins, 2049, 10, ms);
    if (t.valid) return 9;
  }
  bins[50] = 15; bins[100] = 44;
  t.update(bins, 2049, 10, 2850);
  t.update(bins, 2049, 10, 2900);
  t.update(bins, 2049, 10, 2950);
  if (!t.valid || t.hz != 1000) return 10;
  // Unsigned timer wrap must still expire correctly.
  PeakTracker wrap;
  wrap.update(bins, 2049, 10, 0xffffff00u);
  wrap.update(bins, 2049, 10, 0xffffff32u);
  wrap.update(bins, 2049, 10, 0xffffff64u);
  if (!wrap.valid || !wrap.expire(400) || wrap.valid) return 11;
  return 0;
}
