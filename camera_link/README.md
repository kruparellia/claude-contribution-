# Camera link — bring-up & calibration

The ESP32 firmware is now part of the PlatformIO project at
`pio-project/src/esp32_color_detect/main.cpp`. The `arm_mega` env already
contains the receiver; the standalone Arduino-IDE sketches in this
folder are kept as a no-PlatformIO fallback only.

What's where:

- `WIRING.md` — pin map and level-shifting notes.
- `pio-project/src/esp32_color_detect/main.cpp` — runs on the Freenove
  ESP32-WROVER CAM. Captures frames, classifies the dominant colour,
  pushes one ASCII line per frame over UART2 to the Mega, **and** serves
  a live MJPEG view on its own Wi-Fi soft-AP for laptop debugging.
- `pio-project/src/arm_mega/main.cpp` — receives the UART lines on
  Serial2 and exposes a `cam` struct the servo code reads.
- `esp32_color_detect/esp32_color_detect.ino` — older Arduino-IDE
  version of the firmware, no Wi-Fi viewer. Kept for reference; ignore
  if you're using PlatformIO.
- `mega_camera_rx/mega_camera_rx.ino` — older Arduino-IDE standalone
  receiver. Same caveat.

## Building with PlatformIO

```
# from pio-project/
pio run -e esp32_color_detect            # compile
pio run -e esp32_color_detect -t upload  # flash
pio device monitor -e esp32_color_detect # USB serial logs at 115200
```

PSRAM, partition scheme, and the `esp32-camera` library are configured
in `platformio.ini` for you — no Arduino-IDE Tools-menu fiddling.

If uploading hangs at "Connecting…", hold **BOOT/IO0** on the WROVER
while the IDE retries.

## Watching the camera from your laptop

After flashing, the ESP32 brings up its own Wi-Fi access point:

- **SSID**: `RoboArmCam`
- **Password**: `robotarm`
- **URL**: <http://192.168.4.1/>

Connect your laptop to that network and open the URL in any browser.
You'll see the live MJPEG stream plus three badges that update twice a
second:

- "object: PRESENT / absent"
- "colour: RED / GREEN / BLUE / YELLOW / NONE"
- the percentage of the frame matching the dominant colour

Other endpoints on the same server, useful for the report or for
scripting tests:

- `/snapshot` — single JPEG frame (right-click → Save As).
- `/status`   — JSON: `{"present":true,"colour":"RED","pct":42,"ms":...}`
- `/stream`   — raw MJPEG (this is what `/` embeds).

The viewer is purely for bring-up and demos. The UART link to the Mega
runs the same regardless of whether anyone is watching.

> The soft-AP isolates the camera from your home/campus Wi-Fi, so it
> doesn't matter what network you're on. While the laptop is connected
> to `RoboArmCam`, it has no internet access — that's normal.
> Disconnect when you're done debugging.

## Bring-up checklist

1. **Wire as in `WIRING.md`** — the only mandatory connections to the
   Mega are ESP32 GPIO 17 → Mega pin 17 (RX2), plus a shared GND.
   Serial1 on the Mega is reserved for the HC-05.
2. Power the ESP32 over USB. Open the ESP32 serial monitor and confirm
   you see `[cam] camera ready`, `[wifi] AP up …`, and `CAM,…` lines.
3. From your laptop, join `RoboArmCam` and open <http://192.168.4.1/>.
   You should see live video and the badges updating.
4. Flash the Mega (`pio run -e arm_mega -t upload`) and open its serial
   monitor. Within a second you should see `[cam] link: OK` followed by
   `[cam] present colour=…` lines whenever a coloured target enters or
   leaves the frame.
5. Wave each colour target in front of the camera in turn and confirm
   the reported colour matches.

## Calibrating the colour bins

Bench lighting changes everything. The thresholds in `BINS[]` (in
`esp32_color_detect/main.cpp`) are a reasonable starting point under
neutral indoor light, but you'll almost certainly need to nudge them.

With the Wi-Fi viewer, the workflow is much faster than before:

1. Open <http://192.168.4.1/> on your laptop.
2. Hold each target in front of the camera and watch the badges. If a
   target lights up the wrong colour (or nothing), you've found a bin
   to fix.
3. To get exact HSV numbers under your lighting, temporarily add this
   block at the top of `loop()` after `classifyFrame(...)`:

   ```cpp
   // Print centre-pixel HSV once a second
   static uint32_t lastDbg = 0;
   if (millis() - lastDbg > 1000) {
       lastDbg = millis();
       int cx = fb->width / 2, cy = fb->height / 2;
       const uint8_t* p = rgb + (cy * fb->width + cx) * 3;
       int hh, ss, vv;
       rgb_to_hsv(p[0], p[1], p[2], hh, ss, vv);
       Serial.printf("centre HSV: %d %d %d\n", hh, ss, vv);
   }
   ```

4. Hold the target so it fills the centre crosshair, read off the HSV
   value, and update the matching `BINS[]` entry. Re-flash and retest.
5. Once happy, delete the debug block — it slows the detection loop.

## Tuning notes / gotchas

- The detector runs at ~10 Hz (capped in `loop()`); the MJPEG stream
  pushes faster than that. If detection feels sluggish, drop the cap or
  lower `SUBSAMPLE_STEP`.
- `MIN_PRESENCE_PCT` is the noise floor that stops the link from
  flickering "present" on stray pixels. 4 % works for a target that
  fills ~10 % of the frame; raise to 8–10 % for a closer target, lower
  to 2 % if the object is small/distant.
- Red wraps the hue circle (0° and 360° are both red). The code lists
  red as two `BINS[]` entries to handle this — keep both when re-tuning.
- Yellow and green hues sit close together; if they get confused,
  tighten the high end of YELLOW (e.g. 20–35°) before widening GREEN.
- The UART link is one-way (ESP32 → Mega). If you later want the Mega
  to request a one-shot capture, add Mega TX2 → ESP32 RX2 with the
  divider in `WIRING.md` and read commands from `Serial2` on the ESP32
  side.

## Where this fits in the EE6003 build

For sort-mode you only need: `cam.linkOk && cam.present` to gate the
"pick" action, and `cam.colour` to choose which destination bin the arm
moves to. The closed-loop joystick mode can ignore the camera entirely
— `cameraPoll()` is non-blocking, so it costs effectively nothing in
the main loop.

For the report, on-device computer vision + the Wi-Fi viewer + a
custom UART protocol design are all defensible "advanced embedded
features" — useful when arguing past the 60 % core-only cap.
