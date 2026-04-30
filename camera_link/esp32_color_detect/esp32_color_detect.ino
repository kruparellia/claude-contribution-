// ============================================================================
// ESP32-WROVER CAM colour detector for the EE6003 robotic arm
//
// Captures a small RGB565 frame from the OV2640, converts each pixel to HSV
// in fixed point, and counts how many pixels fall inside one of four colour
// bins (RED / GREEN / BLUE / YELLOW). The dominant bin (if its share of the
// frame exceeds a threshold) is reported to the Arduino Mega over UART2.
//
// Message format (one ASCII line per frame, terminated with '\n'):
//   CAM,<state>,<colour>,<pct>\n
//     state  = 0 (no object) | 1 (object present)
//     colour = NONE | RED | GREEN | BLUE | YELLOW
//     pct    = integer 0..100, percentage of frame matching the dominant bin
//
// Wiring: ESP32 GPIO17 (TX2) -> Mega pin 19 (RX1), GND -> GND.
//
// Board in Arduino IDE: "ESP32 Wrover Module".  Partition: default.
// Required library: esp32 by Espressif (>= 2.x), which ships esp_camera.h.
// ============================================================================

#include "esp_camera.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Camera pin map for the Freenove ESP32-WROVER CAM (FNK0060 / FNK0061)
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM        4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ---------------------------------------------------------------------------
// UART link to the Mega
// ---------------------------------------------------------------------------
// Serial2 default pins on the WROVER are GPIO16 (RX2) and GPIO17 (TX2).
// We only need TX2 -> Mega RX1, but both are mapped for completeness.
static const int UART_RX_PIN = 16;
static const int UART_TX_PIN = 17;
static const uint32_t LINK_BAUD = 115200;

// ---------------------------------------------------------------------------
// Detection tuning
// ---------------------------------------------------------------------------
// Frame is captured at QQVGA (160x120) for speed. We further subsample by
// stepping every Nth pixel so the inner loop runs in a few ms.
static const int SUBSAMPLE_STEP = 2;        // 1 = every pixel, 2 = every other

// Minimum percentage of frame pixels that must match the winning colour
// before we declare an object present. Raise this in cluttered scenes,
// lower it for small targets at distance.
static const int MIN_PRESENCE_PCT = 4;

// HSV thresholds (H in 0..359, S/V in 0..255). Tune for your lighting.
struct ColourBin {
  const char* name;
  int  h_lo, h_hi;     // hue range; if h_lo > h_hi, range wraps around 360
  int  s_min;          // minimum saturation (rejects greys/whites)
  int  v_min;          // minimum value      (rejects shadows/black)
};

static const ColourBin BINS[] = {
  // Red wraps the hue circle, so we list two segments and merge them below.
  { "RED",    340, 360, 110, 70 },
  { "RED",      0,  15, 110, 70 },
  { "YELLOW",  20,  45,  90, 90 },
  { "GREEN",   60, 150,  80, 60 },
  { "BLUE",   190, 250,  90, 60 },
};
static const int NUM_BINS = sizeof(BINS) / sizeof(BINS[0]);

// Per-frame counters, indexed by canonical colour name.
enum Colour { C_NONE = 0, C_RED, C_GREEN, C_BLUE, C_YELLOW, NUM_COLOURS };
static const char* COLOUR_NAMES[NUM_COLOURS] = {
  "NONE", "RED", "GREEN", "BLUE", "YELLOW"
};

static Colour colourFromName(const char* n) {
  for (int i = 0; i < NUM_COLOURS; i++) {
    if (strcmp(n, COLOUR_NAMES[i]) == 0) return (Colour)i;
  }
  return C_NONE;
}

// ---------------------------------------------------------------------------
// RGB565 -> 8-bit RGB -> HSV. All integer math, no floats in the hot loop.
// ---------------------------------------------------------------------------
static inline void rgb565_to_rgb888(uint16_t px, uint8_t& r, uint8_t& g, uint8_t& b) {
  // OV2640 outputs big-endian RGB565 in the framebuffer; swap bytes here.
  uint16_t v = (px << 8) | (px >> 8);
  r = (v >> 11) & 0x1F;  r = (r << 3) | (r >> 2);
  g = (v >>  5) & 0x3F;  g = (g << 2) | (g >> 4);
  b =  v        & 0x1F;  b = (b << 3) | (b >> 2);
}

// Hue in degrees (0..359), saturation/value in 0..255.
static inline void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                              int& h, int& s, int& v) {
  uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
  uint8_t mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
  v = mx;
  int delta = mx - mn;
  if (mx == 0 || delta == 0) { h = 0; s = 0; return; }
  s = (delta * 255) / mx;
  int hh;
  if (mx == r)      hh = 60 * (g - b) / delta;
  else if (mx == g) hh = 60 * (b - r) / delta + 120;
  else              hh = 60 * (r - g) / delta + 240;
  if (hh < 0) hh += 360;
  h = hh;
}

static inline bool inBin(const ColourBin& b, int h, int s, int v) {
  if (s < b.s_min || v < b.v_min) return false;
  if (b.h_lo <= b.h_hi) return h >= b.h_lo && h <= b.h_hi;
  return h >= b.h_lo || h <= b.h_hi;   // wrap (not used currently)
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                 // USB debug
  Serial2.begin(LINK_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;
  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;
  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;
  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk  = XCLK_GPIO_NUM;
  cfg.pin_pclk  = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href  = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn  = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_RGB565;   // we want raw colour, not JPEG
  cfg.frame_size   = FRAMESIZE_QQVGA;    // 160x120 — plenty for blob detection
  cfg.fb_count     = 1;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) { delay(1000); }
  }

  // Optional: tweak the sensor for more saturated colours under indoor light.
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_saturation(s, 1);    // -2..2
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
  }

  Serial.println("ESP32 colour detector ready.");
}

// ---------------------------------------------------------------------------
// Main loop: capture, classify, report. ~5–10 frames/sec at QQVGA.
// ---------------------------------------------------------------------------
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("fb_get failed");
    delay(50);
    return;
  }

  uint32_t counts[NUM_COLOURS] = {0};
  uint32_t sampled = 0;

  const uint16_t* pixels = (const uint16_t*)fb->buf;
  const int W = fb->width;
  const int H = fb->height;

  for (int y = 0; y < H; y += SUBSAMPLE_STEP) {
    for (int x = 0; x < W; x += SUBSAMPLE_STEP) {
      uint16_t px = pixels[y * W + x];
      uint8_t r, g, b;
      rgb565_to_rgb888(px, r, g, b);
      int h, s, v;
      rgb_to_hsv(r, g, b, h, s, v);

      sampled++;
      for (int i = 0; i < NUM_BINS; i++) {
        if (inBin(BINS[i], h, s, v)) {
          counts[colourFromName(BINS[i].name)]++;
          break;     // first match wins; bins are disjoint by design
        }
      }
    }
  }

  esp_camera_fb_return(fb);

  // Pick the strongest non-NONE bin.
  int   bestIdx = C_NONE;
  uint32_t bestN = 0;
  for (int i = 1; i < NUM_COLOURS; i++) {
    if (counts[i] > bestN) { bestN = counts[i]; bestIdx = i; }
  }

  int pct = sampled ? (int)((bestN * 100UL) / sampled) : 0;
  bool present = (bestIdx != C_NONE) && (pct >= MIN_PRESENCE_PCT);

  // Emit to the Mega and to USB for debugging.
  char line[48];
  snprintf(line, sizeof(line), "CAM,%d,%s,%d\n",
           present ? 1 : 0,
           present ? COLOUR_NAMES[bestIdx] : "NONE",
           pct);
  Serial2.print(line);
  Serial.print(line);

  delay(50);   // ~20 Hz cap; the camera is the real bottleneck
}
