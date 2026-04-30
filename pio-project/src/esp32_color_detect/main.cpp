// esp32_color_detect/main.cpp
// ----------------------------------------------------------------------
// Runs on the Freenove ESP32-WROVER CAM (FNK0060/FNK0061).
//
// Responsibilities:
//   1. Initialise the OV2640 camera in JPEG mode (QQVGA, 160x120).
//   2. Each frame: decode JPEG -> RGB888 in RAM, classify pixels into
//      HSV colour bins (RED / GREEN / BLUE / YELLOW), pick the dominant
//      colour, and report it to the Arduino Mega over UART2 as
//          CAM,<state>,<colour>,<pct>\n
//      where state is 0/1 and colour is NONE/RED/GREEN/BLUE/YELLOW.
//   3. Bring up a Wi-Fi soft-AP and a tiny HTTP server so a laptop can
//      open http://192.168.4.1/ and watch what the camera sees, with a
//      live overlay of the current detection. This is purely for
//      bring-up and demo — useful when the threshold values aren't
//      catching your real-world colours.
//
// Wiring (see camera_link/WIRING.md):
//   ESP32 GPIO 13 (TX2) -> Mega pin 17 (RX2)   [3.3V into 5V is fine]
//   ESP32 GND           -> Mega GND
//
// NOTE: do NOT use GPIO 16/17 on this board. On many ESP32-WROVER modules
// (including some Freenove revisions) GPIO 16/17 are wired to the PSRAM
// chip's CS/CLK lines. Re-muxing them with Serial2.begin() silently
// corrupts the PSRAM heap, and the very next malloc() crashes with
// LoadProhibited inside the TLSF allocator. GPIO 13 is exposed on the
// Freenove header and is safe to use as a UART output.
//
// Build (PlatformIO):
//   pio run -e esp32_color_detect -t upload
//   pio device monitor -e esp32_color_detect -b 115200
// ----------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"   // fmt2rgb888

// ---------------------------------------------------------------------
// Camera pin map for the Freenove ESP32-WROVER CAM
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// UART link to the Mega. We remap Serial2 onto GPIO 13 (TX) — the default
// Serial2 pins (GPIO 16 RX2 / GPIO 17 TX2) overlap with PSRAM CS/CLK on
// WROVER modules and corrupt the heap. The Mega only listens, so RX is
// disabled (-1).
// ---------------------------------------------------------------------
static const int      UART_RX_PIN = -1;
static const int      UART_TX_PIN = 13;
static const uint32_t LINK_BAUD   = 115200;

// ---------------------------------------------------------------------
// Wi-Fi soft-AP for the laptop view
// ---------------------------------------------------------------------
// AP_SSID/AP_PASS are what your laptop will look for. Change here if you
// want. WPA2 needs >=8 chars or it falls back to open. The IP that the
// browser opens is the standard ESP-IDF soft-AP gateway: 192.168.4.1.
static const char* AP_SSID = "RoboArmCam";
static const char* AP_PASS = "robotarm";

WebServer server(80);

// fmt2rgb888 (JPEG decoder) uses ~10 KB of stack on its own; the default
// Arduino loopTask stack is 8 KB. The proper override in arduino-esp32 is
// the SET_LOOP_TASK_STACK_SIZE macro, which provides a strong override of
// the weak getArduinoLoopTaskStackSize(). Without this the loopTask blows
// its stack the first time fmt2rgb888 runs and the board reboots with
// LoadProhibited at a tiny EXCVADDR.
SET_LOOP_TASK_STACK_SIZE(20 * 1024);

// ---------------------------------------------------------------------
// Detection tuning — tune for your bench lighting using the live view
// ---------------------------------------------------------------------
static const int SUBSAMPLE_STEP   = 2;   // 1 = every pixel, 2 = every other
static const int MIN_PRESENCE_PCT = 4;   // % below this -> "no object"

struct ColourBin {
    const char* name;
    int  h_lo, h_hi;   // hue 0..359; if h_lo > h_hi, range wraps around 360
    int  s_min;        // minimum saturation (rejects greys/whites)
    int  v_min;        // minimum value      (rejects shadows/black)
};

// Red wraps the hue circle; list two non-wrapping segments so the loop
// stays simple. Tweak under your actual lighting.
static const ColourBin BINS[] = {
    { "RED",    340, 360, 110, 70 },
    { "RED",      0,  15, 110, 70 },
    { "YELLOW",  20,  45,  90, 90 },
    { "GREEN",   60, 150,  80, 60 },
    { "BLUE",   190, 250,  90, 60 },
};
static const int NUM_BINS = sizeof(BINS) / sizeof(BINS[0]);

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

// Latest-detection state (updated by the detector, read by the HTTP
// handlers). Volatile because the HTTP code runs from the same loop()
// thread, but if we ever move HTTP onto its own task this hint helps.
struct Detection {
    bool    present = false;
    Colour  colour  = C_NONE;
    int     pct     = 0;
    uint32_t lastMs = 0;
};
static Detection detection;

// ---------------------------------------------------------------------
// RGB -> HSV (integer math, no floats in the hot loop)
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// Run HSV classification over an RGB888 buffer. Updates `detection` and
// returns the dominant colour for convenience.
// ---------------------------------------------------------------------
static void classifyFrame(const uint8_t* rgb, int w, int h_) {
    uint32_t counts[NUM_COLOURS] = {0};
    uint32_t sampled = 0;

    for (int y = 0; y < h_; y += SUBSAMPLE_STEP) {
        const uint8_t* row = rgb + (size_t)y * w * 3;
        for (int x = 0; x < w; x += SUBSAMPLE_STEP) {
            uint8_t r = row[x * 3 + 0];
            uint8_t g = row[x * 3 + 1];
            uint8_t b = row[x * 3 + 2];
            int hh, ss, vv;
            rgb_to_hsv(r, g, b, hh, ss, vv);

            sampled++;
            for (int i = 0; i < NUM_BINS; i++) {
                if (inBin(BINS[i], hh, ss, vv)) {
                    counts[colourFromName(BINS[i].name)]++;
                    break;
                }
            }
        }
    }

    int      bestIdx = C_NONE;
    uint32_t bestN   = 0;
    for (int i = 1; i < NUM_COLOURS; i++) {
        if (counts[i] > bestN) { bestN = counts[i]; bestIdx = i; }
    }

    int pct = sampled ? (int)((bestN * 100UL) / sampled) : 0;
    bool present = (bestIdx != C_NONE) && (pct >= MIN_PRESENCE_PCT);

    detection.present = present;
    detection.colour  = present ? (Colour)bestIdx : C_NONE;
    detection.pct     = pct;
    detection.lastMs  = millis();
}

// Send one report line to the Mega and to the USB monitor.
static void reportToMega() {
    char line[48];
    snprintf(line, sizeof(line), "CAM,%d,%s,%d\n",
             detection.present ? 1 : 0,
             detection.present ? COLOUR_NAMES[detection.colour] : "NONE",
             detection.pct);
    Serial2.print(line);
    Serial.print(line);
}

// Decode one JPEG frame to RGB888, run classification, push the result
// to the Mega. Used both by the standalone detection tick in loop() and
// by the streaming handler (so detection keeps running while a browser
// is watching). Throttled internally so we never spam the UART faster
// than ~10 Hz regardless of how often it's called.
static void detectFromFrame(camera_fb_t* fb) {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < 100) return;
    lastMs = now;

    static uint8_t* rgbBuf = nullptr;
    static size_t   rgbBufSz = 0;
    size_t needed = (size_t)fb->width * fb->height * 3;
    if (rgbBuf == nullptr || rgbBufSz != needed) {
        if (rgbBuf) { free(rgbBuf); rgbBuf = nullptr; rgbBufSz = 0; }
        // Prefer PSRAM for the ~57 KB QQVGA RGB buffer so we don't starve
        // internal DRAM (WiFi/WebServer live there).
        if (psramFound()) rgbBuf = (uint8_t*)ps_malloc(needed);
        if (!rgbBuf)      rgbBuf = (uint8_t*)malloc(needed);
        rgbBufSz = rgbBuf ? needed : 0;
    }
    if (!rgbBuf) return;

    if (fmt2rgb888(fb->buf, fb->len, fb->format, rgbBuf)) {
        classifyFrame(rgbBuf, fb->width, fb->height);
        reportToMega();
    }
}

// ---------------------------------------------------------------------
// HTTP handlers
//
// "/"          -> simple HTML page that <img>'s the MJPEG stream and
//                 polls /status once a second for the live overlay text.
// "/stream"    -> multipart MJPEG, served from the same camera buffer.
// "/snapshot"  -> one-off JPEG (handy for screenshots in the report).
// "/status"    -> JSON {"present":..,"colour":"..","pct":..,"ms":..}
// ---------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html><head><meta charset="utf-8">
<title>RoboArmCam</title>
<style>
  body { background:#111; color:#eee; font-family:system-ui,sans-serif; margin:0; padding:16px; }
  h1   { margin:0 0 12px 0; font-size:18px; font-weight:600; }
  .wrap { max-width:720px; }
  .stream { width:100%; image-rendering:pixelated; border:1px solid #333; background:#000; }
  .badge { display:inline-block; padding:6px 10px; border-radius:6px;
           font-weight:600; margin-right:8px; }
  .ok    { background:#1f6f3f; }
  .none  { background:#444; }
  .red   { background:#a83232; }
  .green { background:#3a9b3a; }
  .blue  { background:#2f6fb8; }
  .yellow{ background:#b89a2f; color:#111; }
  .row   { margin-top:10px; font-size:14px; }
  code   { background:#222; padding:2px 6px; border-radius:4px; }
</style></head>
<body><div class="wrap">
  <h1>RoboArmCam — live view + detection</h1>
  <img class="stream" src="/stream" alt="camera stream">
  <div class="row">
    <span id="present" class="badge none">--</span>
    <span id="colour"  class="badge none">--</span>
    <span id="pct">--</span>
  </div>
  <div class="row" style="color:#888;">
    Tip: hold a target so it fills the centre of the frame, then tune
    the HSV bins in <code>main.cpp</code> until the badge above turns
    on for the right colour.
  </div>
</div>
<script>
async function tick() {
  try {
    const r = await fetch('/status', {cache:'no-store'});
    const j = await r.json();
    const p = document.getElementById('present');
    const c = document.getElementById('colour');
    p.textContent = j.present ? 'object: PRESENT' : 'object: absent';
    p.className   = 'badge ' + (j.present ? 'ok' : 'none');
    c.textContent = 'colour: ' + j.colour;
    c.className   = 'badge ' + j.colour.toLowerCase();
    document.getElementById('pct').textContent = j.pct + '% of frame';
  } catch(e) { /* ignore — stream might be saturating the link */ }
}
setInterval(tick, 500);
tick();
</script>
</body></html>
)HTML";

static void handleIndex() {
    server.send_P(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"present\":%s,\"colour\":\"%s\",\"pct\":%d,\"ms\":%lu}",
             detection.present ? "true" : "false",
             detection.present ? COLOUR_NAMES[detection.colour] : "NONE",
             detection.pct,
             (unsigned long)detection.lastMs);
    server.send(200, "application/json", buf);
}

static void handleSnapshot() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { server.send(500, "text/plain", "fb_get failed"); return; }
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Disposition", "inline; filename=snapshot.jpg");
    server.setContentLength(fb->len);
    server.send(200);
    WiFiClient client = server.client();
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

// MJPEG stream. We drive this from the same camera buffer that the
// detector uses on its own tick — here we just push frames as fast as
// the Wi-Fi link will carry them. The detection loop in loop() runs
// independently and never blocks on the stream.
//
// Note: this handler blocks for as long as the browser stays connected.
// Browsers open additional TCP connections for /status polls, so the
// overlay still updates while the stream is live. If you ever see the
// overlay frozen, hard-refresh the browser tab.
static void handleStream() {
    WiFiClient client = server.client();
    if (!client) return;

    static const char STREAM_HEAD[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n\r\n";
    client.write((const uint8_t*)STREAM_HEAD, sizeof(STREAM_HEAD) - 1);

    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { delay(20); continue; }

        char part[96];
        int n = snprintf(part, sizeof(part),
                         "--frame\r\nContent-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n\r\n",
                         (unsigned)fb->len);
        client.write((const uint8_t*)part, n);
        client.write(fb->buf, fb->len);
        client.write((const uint8_t*)"\r\n", 2);
        esp_camera_fb_return(fb);

        delay(1);
    }
}

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
static void initCamera() {
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
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;     // streamable + fmt2rgb888-decodable
    cfg.frame_size   = FRAMESIZE_QQVGA;    // 160x120
    cfg.jpeg_quality = 12;                 // 10..15 is a good range
    // PSRAM is present on the WROVER module (BOARD_HAS_PSRAM is set in
    // platformio.ini). Park the framebuffers there so internal DRAM stays
    // free for WiFi/WebServer/the rgbBuf below. CAMERA_GRAB_LATEST also
    // needs fb_count >= 2 to actually drop stale frames.
    cfg.fb_count     = psramFound() ? 2 : 1;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[cam] init failed: 0x%x\n", err);
        while (true) { delay(1000); }
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_saturation(s, 1);
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
    }
    Serial.println("[cam] camera ready");
}

static void initWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("[wifi] AP up. SSID=");
    Serial.print(AP_SSID);
    Serial.print(" pass=");
    Serial.print(AP_PASS);
    Serial.print(" -> http://");
    Serial.println(ip);
}

static void initHTTP() {
    server.on("/",         HTTP_GET, handleIndex);
    server.on("/stream",   HTTP_GET, handleStream);
    server.on("/snapshot", HTTP_GET, handleSnapshot);
    server.on("/status",   HTTP_GET, handleStatus);
    server.begin();
    Serial.println("[http] server listening on :80");
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(LINK_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.println();
    Serial.println("[boot] esp32_color_detect");

    initCamera();
    initWiFi();
    initHTTP();
}

// ---------------------------------------------------------------------
// Main loop:
//   - Service HTTP first. While a /stream client is connected this call
//     blocks inside handleStream(), but that handler itself drives
//     detection per-frame, so the UART link to the Mega keeps reporting.
//   - When no stream is open, grab a frame ourselves and run detection.
// ---------------------------------------------------------------------
void loop() {
    server.handleClient();

    // The streaming handler already runs detection on every pushed frame
    // (it's internally rate-limited), so during an active stream this
    // path effectively short-circuits. When no client is connected we
    // grab our own frames here.
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return;
    detectFromFrame(fb);
    esp_camera_fb_return(fb);
}
