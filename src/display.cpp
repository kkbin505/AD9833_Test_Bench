#include "display.h"
#include "peak_tracker.h"

#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

namespace display {
namespace {

// --- Board wiring (ES3C28P / ES3N28P, see the module pinout) --------------
constexpr int kPinSck = 12;
constexpr int kPinMosi = 11;
constexpr int kPinMiso = 13;
constexpr int kPinCs = 10;
constexpr int kPinDc = 46;
constexpr int kPinBl = 45;
constexpr uint32_t kSpiHz = 40000000;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 panel_;
  lgfx::Bus_SPI bus_;
  lgfx::Light_PWM light_;

 public:
  LGFX() {
    {
      auto cfg = bus_.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = kSpiHz;
      cfg.pin_sclk = kPinSck;
      cfg.pin_mosi = kPinMosi;
      cfg.pin_miso = kPinMiso;
      cfg.pin_dc = kPinDc;
      bus_.config(cfg);
      panel_.setBus(&bus_);
    }
    {
      auto cfg = panel_.config();
      cfg.pin_cs = kPinCs;
      cfg.pin_rst = -1;  // shares the ESP32 reset line
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.bus_shared = true;
      cfg.invert = true;  // IPS panel needs colour inversion
      panel_.config(cfg);
    }
    {
      auto cfg = light_.config();
      cfg.pin_bl = kPinBl;
      light_.config(cfg);
      panel_.setLight(&light_);
    }
    setPanel(&panel_);
  }
};

LGFX tft;
LGFX_Sprite plot(&tft);
LGFX_Sprite curve(&tft);
LGFX_Sprite sidebar(&tft);

// Landscape: 320 x 240 after setRotation(1).
constexpr int kScreenW = 320;
constexpr int kScreenH = 240;

// Left column: readings/settings. Right: spectrum (about 1/4), then waterfall.
constexpr int kSidebarW = 70;
constexpr int kPlotX = kSidebarW + 22;
constexpr int kPlotW = kScreenW - kPlotX - 4;
constexpr int kLineY = 4;
constexpr int kLineH = 50;
constexpr int kPlotY = 64;
constexpr int kPlotH = 154;
constexpr int kLabelY = 224;

// Logarithmic frequency axis, 20 Hz .. 20 kHz.
constexpr float kFreqMin = 20.0f;
constexpr float kFreqMax = 20000.0f;

// Line mode: dBA scale of the plot. Levels are relative until the microphone
// is calibrated, so treat these as display limits rather than absolute SPL.
constexpr float kDbMin = 20.0f;
constexpr float kDbMax = 80.0f;

// Heat map mode: black at or below kHeatFloorDb, brightest colour at kHeatTopDb.
// Change kHeatTopDb to rescale the colour range.
constexpr float kHeatFloorDb = 30.0f;
constexpr float kHeatTopDb = 60.0f;
constexpr uint32_t kHeatRowMs = 500;  // one spectrum row every 0.5 s

const uint16_t kBg = lgfx::color565(6, 8, 12);
const uint16_t kBlack = lgfx::color565(0, 0, 0);
const uint16_t kGrid = lgfx::color565(38, 44, 54);
const uint16_t kLine = lgfx::color565(90, 200, 245);
const uint16_t kGrey = lgfx::color565(140, 150, 160);
const uint16_t kWhite = lgfx::color565(235, 240, 245);
const uint16_t kGreen = lgfx::color565(80, 220, 120);
const uint16_t kPeakRed = lgfx::color565(255, 64, 64);

uint16_t gLoBin[kPlotW];
uint16_t gHiBin[kPlotW];
bool gMapped = false;

// Heat map: per-column maximum over all spectra in the current 0.5 s window.
float gRowMax[kPlotW];

// FT6336G shares the already initialised Wire bus with the ES8311.
constexpr uint8_t kTouchAddr = 0x38;
constexpr int kOrderX = (kSidebarW - 24) / 2;
constexpr int kOrderUpY = 140;
constexpr int kOrderDownY = 194;
constexpr int kButtonSize = 24;
uint16_t gOrder = 10;
float gPeakHz = 0.0f;
float gPeakDba = -INFINITY;
PeakTracker gTracker;
float gDba = 0.0f, gSpl = 0.0f, gPeak = 0.0f;
bool gTouchDown = false;
bool gTouchAvailable = false;
bool gReady = false;
const float *gLatestBins = nullptr;

int yOf(float db) {
  float t = (kDbMax - db) / (kDbMax - kDbMin);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return static_cast<int>(t * (kLineH - 1));
}

int xOf(float freq) {
  if (freq <= kFreqMin) return kPlotX;
  const float t = log10f(freq / kFreqMin) / log10f(kFreqMax / kFreqMin);
  return kPlotX + static_cast<int>(lroundf(t * (kPlotW - 1)));
}

// inferno-style palette: black -> purple -> magenta -> orange -> pale yellow.
uint16_t heatColor(float db) {
  float t = (db - kHeatFloorDb) / (kHeatTopDb - kHeatFloorDb);
  if (t <= 0.0f) return kBlack;
  if (t > 1.0f) t = 1.0f;

  static const uint8_t kStops[5][3] = {
      {0, 0, 4}, {87, 16, 110}, {188, 55, 84}, {249, 142, 9}, {252, 255, 164}};
  const float pos = t * 4.0f;
  int i = static_cast<int>(pos);
  if (i >= 4) i = 3;
  const float f = pos - i;
  uint8_t rgb[3];
  for (int c = 0; c < 3; ++c) {
    const float a = kStops[i][c];
    const float b = kStops[i + 1][c];
    rgb[c] = static_cast<uint8_t>(lroundf(a + (b - a) * f));
  }
  return lgfx::color565(rgb[0], rgb[1], rgb[2]);
}

// Frequency labels under the plot; shared by both modes.
void drawFreqLabels() {
  tft.setFont(&lgfx::fonts::Font0);  // small: 10k and 20k sit close together
  tft.setTextColor(kGrey, kBg);
  const float ticks[] = {20.0f, 100.0f, 1000.0f, 10000.0f, 20000.0f};
  const char *labels[] = {"20", "100", "1k", "10k", "20k"};
  for (int i = 0; i < 5; ++i) {
    const int x = xOf(ticks[i]);
    if (i == 0) {
      tft.drawString(labels[i], x, kLabelY);
    } else if (i == 4) {
      tft.drawRightString(labels[i], x, kLabelY);
    } else {
      tft.drawCenterString(labels[i], x, kLabelY);
    }
  }
}

void drawAxes() {
  tft.setFont(&lgfx::fonts::Font0);
  tft.setTextColor(kGrey, kBg);
  for (int db = static_cast<int>(kDbMin); db <= static_cast<int>(kDbMax); db += 20) {
    tft.drawRightString(String(db), kPlotX - 4,
                        kLineY + yOf(static_cast<float>(db)) - 3);
  }
  for (int seconds = 0; seconds <= 60; seconds += 20) {
    char buf[12];
    snprintf(buf, sizeof(buf), seconds ? "-%ds" : "0s", seconds);
    tft.drawRightString(buf, kPlotX - 4, kPlotY + seconds * 2);
  }
  tft.drawFastVLine(kSidebarW + 2, 0, kScreenH, kGrid);
  tft.drawFastHLine(kPlotX, kPlotY - 5, kPlotW, kGrid);
  drawFreqLabels();
}

void drawSidebar(float dba, float spl, float peak) {
  char buf[24];
  // Compose off-screen, then transfer the completed image without a blank phase.
  sidebar.fillSprite(kBg);
  sidebar.setFont(&lgfx::fonts::Font2);
  sidebar.setTextColor(kGrey, kBg);
  sidebar.drawString("dBA", 4, 2);
  sidebar.setFont(&lgfx::fonts::Font4);
  sidebar.setTextColor(kWhite, kBg);
  snprintf(buf, sizeof(buf), "%.1f", dba);
  sidebar.drawString(buf, 4, 20);

  sidebar.setFont(&lgfx::fonts::Font0);
  sidebar.setTextColor(kGrey, kBg);
  snprintf(buf, sizeof(buf), "SPL %.1f", spl);
  sidebar.drawString(buf, 4, 52);
  snprintf(buf, sizeof(buf), "pk %.1f", peak);
  sidebar.drawString(buf, 4, 73);

  sidebar.setTextColor(kGrey, kBg);

  sidebar.setFont(&lgfx::fonts::Font2);
  sidebar.drawCenterString("Order", kSidebarW / 2, 116);
  sidebar.drawRoundRect(kOrderX, kOrderUpY, kButtonSize, kButtonSize, 4, kGrey);
  sidebar.drawRoundRect(kOrderX, kOrderDownY, kButtonSize, kButtonSize, 4, kGrey);
  const int cx = kSidebarW / 2;
  sidebar.fillTriangle(cx, kOrderUpY + 5,
                       cx - 7, kOrderUpY + 17,
                       cx + 7, kOrderUpY + 17, gOrder < UINT16_MAX ? kWhite : kGrid);
  sidebar.fillTriangle(cx, kOrderDownY + 19,
                       cx - 7, kOrderDownY + 7,
                       cx + 7, kOrderDownY + 7, gOrder > 1 ? kWhite : kGrid);
  sidebar.setTextColor(kWhite, kBg);
  snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(gOrder));
  sidebar.drawCenterString(buf, cx, 171);
  sidebar.pushSprite(0, 0);
}

void buildMapping(float bin_hz, uint16_t bin_count) {
  const int last_bin = static_cast<int>(bin_count) - 1;
  for (int x = 0; x < kPlotW; ++x) {
    // Each column covers a half-column either side of its centre, on the
    // log scale, so the bins inside it are the ones it is drawn from.
    const float ratio = kFreqMax / kFreqMin;
    const float f_lo = kFreqMin * powf(ratio, (x - 0.5f) / (kPlotW - 1));
    const float f_hi = kFreqMin * powf(ratio, (x + 0.5f) / (kPlotW - 1));
    int lo = static_cast<int>(ceilf(f_lo / bin_hz));
    int hi = static_cast<int>(floorf(f_hi / bin_hz));
    if (lo < 1) lo = 1;  // skip DC
    if (lo > last_bin) lo = last_bin;
    if (hi > last_bin) hi = last_bin;
    if (hi < lo) hi = lo;
    gLoBin[x] = static_cast<uint16_t>(lo);
    gHiBin[x] = static_cast<uint16_t>(hi);
  }
  gMapped = true;
}

// Largest dBA bin inside screen column x.
float columnLevel(const float *dba_bins, int x) {
  float best = -200.0f;
  for (int k = gLoBin[x]; k <= gHiBin[x]; ++k) {
    if (dba_bins[k] > best) best = dba_bins[k];
  }
  return best;
}

void drawLineCurve(const float *dba_bins) {
  curve.fillSprite(kBg);
  int ys[kPlotW];
  for (int x = 0; x < kPlotW; ++x) {
    ys[x] = yOf(columnLevel(dba_bins, x));
  }
  // Dark blue area, fading toward the baseline like a spectrum analyser.
  // Fill before drawing the grid and trace so both remain visible.
  for (int y = 0; y < kLineH; ++y) {
    const float brightness = 1.0f - static_cast<float>(y) / (kLineH - 1);
    const uint16_t color = lgfx::color565(3, 8 + 22 * brightness,
                                          28 + 65 * brightness);
    for (int x = 0; x < kPlotW; ++x) {
      if (y >= ys[x]) curve.drawPixel(x, y, color);
    }
  }
  for (int db = static_cast<int>(kDbMin); db <= static_cast<int>(kDbMax); db += 20) {
    curve.drawFastHLine(0, yOf(static_cast<float>(db)), kPlotW, kGrid);
  }
  const float grid_hz[] = {100.0f, 1000.0f, 10000.0f};
  for (float f : grid_hz) {
    curve.drawFastVLine(xOf(f) - kPlotX, 0, kLineH, kGrid);
  }
  for (int x = 0; x < kPlotW; ++x) {
    if (x > 0) curve.drawLine(x - 1, ys[x - 1], x, ys[x], kLine);
    else curve.drawPixel(x, ys[x], kLine);
  }
  if (gPeakHz > 0.0f && isfinite(gPeakDba)) {
    const int peak_x = xOf(gPeakHz) - kPlotX;
    const int peak_y = yOf(gPeakDba);
    curve.drawFastVLine(peak_x, 0, kLineH, kPeakRed);
    curve.fillCircle(peak_x, peak_y, 2, kPeakRed);

  }
  char label[48];
  if (gTracker.valid) {
    snprintf(label, sizeof(label), "%.0fdBA @ %.0fHz %.0fRPM",
             gPeakDba, gPeakHz, gPeakHz * 60.0f / gOrder);
  } else {
    snprintf(label, sizeof(label), "--dBA @ --Hz --RPM");
  }
  curve.setFont(&lgfx::fonts::Font0);
  curve.setTextSize(1.0f);
  constexpr int label_x = 4;
  constexpr int label_y = 5 - kLineY;
  curve.fillRect(label_x - 2, label_y, curve.textWidth(label) + 4,
                 curve.fontHeight() + 2, kBg);
  curve.setTextColor(kWhite, kBg);
  curve.drawString(label, label_x, label_y);
  // Fill, marker and text share one off-screen frame, avoiding erase flicker.
  curve.pushSprite(kPlotX, kLineY);
}

// Heat map: shift the whole sprite down one row (raw byte move, sprite is
// 16 bpp with no padding), then write the newest spectrum into row 0.
void pushHeatRow(const float *row_db) {
  uint8_t *buf = static_cast<uint8_t *>(plot.getBuffer());
  const size_t row_bytes = kPlotW * sizeof(uint16_t);
  memmove(buf + row_bytes, buf, row_bytes * (kPlotH - 1));

  for (int x = 0; x < kPlotW; ++x) {
    plot.drawPixel(x, 0, heatColor(row_db[x]));
  }
  plot.pushSprite(kPlotX, kPlotY);
}

}  // namespace

void begin() {
  for (int x = 0; x < kPlotW; ++x) gRowMax[x] = -200.0f;
  tft.init();
  tft.setRotation(3);  // landscape, 320 x 240, rotated 180 degrees from rotation 1
  tft.setBrightness(255);
  tft.fillScreen(kBg);

  pinMode(18, OUTPUT);  // FT6336G reset; does not reset the audio codec
  digitalWrite(18, LOW);
  delay(10);
  digitalWrite(18, HIGH);
  delay(100);
  Wire.beginTransmission(kTouchAddr);
  gTouchAvailable = Wire.endTransmission() == 0;
  if (!gTouchAvailable) Serial.println("# FT6336G touch not found at 0x38");

  plot.setColorDepth(16);
  plot.setPsram(true);
  if (plot.createSprite(kPlotW, kPlotH) == nullptr) {
    plot.setPsram(false);
    if (plot.createSprite(kPlotW, kPlotH) == nullptr) {
      tft.setFont(&lgfx::fonts::Font2);
      tft.setTextColor(kWhite, kBg);
      tft.drawString("plot buffer alloc failed", 8, 40);
      return;
    }
  }
  curve.setColorDepth(16);
  curve.setPsram(true);
  if (curve.createSprite(kPlotW, kLineH) == nullptr) {
    curve.setPsram(false);
    if (curve.createSprite(kPlotW, kLineH) == nullptr) {
      tft.setFont(&lgfx::fonts::Font2);
      tft.setTextColor(kWhite, kBg);
      tft.drawString("spectrum buffer alloc failed", 8, 40);
      return;
    }
  }
  sidebar.setColorDepth(16);
  sidebar.setPsram(true);
  if (sidebar.createSprite(kSidebarW, kScreenH) == nullptr) {
    sidebar.setPsram(false);
    if (sidebar.createSprite(kSidebarW, kScreenH) == nullptr) {
      tft.setFont(&lgfx::fonts::Font2);
      tft.setTextColor(kWhite, kBg);
      tft.drawString("sidebar buffer alloc failed", 8, 40);
      return;
    }
  }
  plot.fillSprite(kBlack);
  plot.pushSprite(kPlotX, kPlotY);
  curve.fillSprite(kBg);
  curve.pushSprite(kPlotX, kLineY);
  drawAxes();
  drawSidebar(0.0f, 0.0f, 0.0f);
  gReady = true;
}

void pollTouch() {
  if (!gReady) return;
  if (gTracker.expire(millis())) {
    gPeakHz = gTracker.hz;
    gPeakDba = gTracker.dba;
    if (gLatestBins != nullptr) drawLineCurve(gLatestBins);
  }
  if (!gTouchAvailable) return;
  Wire.beginTransmission(kTouchAddr);
  Wire.write(static_cast<uint8_t>(0x02));
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(static_cast<uint8_t>(kTouchAddr), static_cast<uint8_t>(5)) != 5) return;
  uint8_t b[5];
  for (auto &v : b) v = Wire.read();
  const bool down = (b[0] & 0x0f) != 0;
  const bool pressed = down && !gTouchDown;
  gTouchDown = down;  // One step per press; holding or dragging does not repeat.
  if (!pressed || (b[0] & 0x0f) != 1) return;
  const int raw_x = ((b[1] & 0x0f) << 8) | b[2];
  const int raw_y = ((b[3] & 0x0f) << 8) | b[4];
  if (raw_x >= 240 || raw_y >= 320) return;
  // Rotation 3 plus the board's 180-degree touch offset (same as badge firmware).
  const int x = raw_y;
  const int y = 239 - raw_x;
  if (x < kOrderX || x >= kOrderX + kButtonSize) return;
  if (y >= kOrderUpY && y < kOrderUpY + kButtonSize && gOrder < UINT16_MAX) {
    ++gOrder;
  } else if (y >= kOrderDownY && y < kOrderDownY + kButtonSize && gOrder > 1) {
    --gOrder;
  } else {
    return;
  }
  drawSidebar(gDba, gSpl, gPeak);
  if (gLatestBins != nullptr) drawLineCurve(gLatestBins);
}

void update(const float *dba_bins, uint16_t bin_count, float bin_hz,
            float dba, float spl, float peak) {
  if (!gReady || dba_bins == nullptr || bin_count < 2 || !isfinite(bin_hz) || bin_hz <= 0.0f) return;
  gLatestBins = dba_bins;
  gDba = dba;
  gSpl = spl;
  gPeak = peak;
  gTracker.update(dba_bins, bin_count, bin_hz, millis());
  gPeakHz = gTracker.hz;
  gPeakDba = gTracker.dba;
  if (!gMapped) {
    if (bin_hz <= 0.0f) return;
    buildMapping(bin_hz, bin_count);
  }
  if (plot.getBuffer() == nullptr) return;

  const uint32_t now = millis();

  // Both plots use exactly the same column-to-bin mapping and horizontal extent.
  drawLineCurve(dba_bins);
  for (int x = 0; x < kPlotW; ++x) {
    const float v = columnLevel(dba_bins, x);
    if (v > gRowMax[x]) gRowMax[x] = v;
  }
  static uint32_t last_row = 0;
  if (now - last_row >= kHeatRowMs) {
    pushHeatRow(gRowMax);
    for (int x = 0; x < kPlotW; ++x) gRowMax[x] = -200.0f;
    last_row = now;
  }

  static uint32_t last_text = 0;
  if (now - last_text >= 200) {
    drawSidebar(dba, spl, peak);
    last_text = now;
  }
}

}  // namespace display
