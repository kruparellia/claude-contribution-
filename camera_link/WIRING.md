# ESP32-WROVER CAM ↔ Arduino Mega — wiring reference

One-way UART link: the ESP32 reports detections to the Mega. The Mega does
not need to send anything back, so we only wire ESP32 TX → Mega RX (plus
ground). This is the simplest reliable option and keeps the HC-05 on the
Uno untouched.

## Pin choice

ESP32-WROVER CAM (Freenove FNK0060) — the camera occupies GPIO 4, 5, 18,
19, 21, 22, 23, 25, 26, 27, 34, 35, 36, 39, and the on-board SD card uses
GPIO 2, 12, 13, 14, 15.

> ⚠️ **Do not use GPIO 16 or 17.** On many ESP32-WROVER modules these
> pins are tied to the on-package PSRAM's CS/CLK lines. Re-muxing them
> with `Serial2.begin()` corrupts the PSRAM heap and the firmware
> crashes inside `malloc` (LoadProhibited at `EXCVADDR=0xf`). Even
> though GPIO 16 / 17 are the *default* Serial2 pins on the ESP32, they
> are unsafe on PSRAM-equipped WROVER boards.

We remap `Serial2` onto **GPIO 13 (TX)** instead. The Mega only listens,
so RX is disabled. GPIO 13 is broken out on the Freenove header, has no
strapping role, and does not collide with the camera or PSRAM. Avoid
GPIO 1 / GPIO 3 — they're shared with the USB-serial bridge used to
flash the board.

On the **Mega side**, `Serial1` is already in use for the HC-05 link
(pins 18 / 19, 9600 baud) by `arm_mega/main.cpp`. The camera therefore
lands on **`Serial2`** (Mega pin 17 = RX2). `Serial3` (pins 14 / 15) is
left free for future expansion.

| Signal              | ESP32-WROVER pin | Arduino Mega pin |
|---------------------|------------------|------------------|
| ESP32 TX → Mega RX2 | GPIO 13          | pin 17 (RX2)     |
| GND                 | GND              | GND              |

## 5 V ↔ 3.3 V level shifting

The Mega's UART pins idle at 5 V. The ESP32's GPIOs are **not 5 V
tolerant** — feeding 5 V into a GPIO will eventually damage it. Since
we're only using ESP32 → Mega in this build, the line that matters is
ESP32 TX (3.3 V) → Mega RX (5 V), which works directly because 3.3 V is
above the Mega's logic-high threshold.

If you ever wire the reverse direction (Mega TX → ESP32 RX), drop the
voltage with either:

- **Resistor divider** (cheap): Mega TX → 1 kΩ → ESP32 RX, and ESP32 RX →
  2 kΩ → GND. That gives ~3.3 V on the ESP32 side.
- **Bidirectional level shifter board** (cleaner): e.g. a 4-channel
  MOSFET-based shifter. Pick this if you're going to add more signals
  later.

## Power

Powering the ESP32 from the Mega's 5 V rail is fine for bring-up (the
WROVER has its own 3.3 V regulator), but the camera draws current in
bursts — if you see brown-outs or random reboots, give the ESP32 its own
USB power supply and only share **GND** with the Mega.

## Quick check before powering on

1. Continuity check: ESP32 GND ↔ Mega GND.
2. Confirm ESP32 GPIO 13 goes to Mega pin 17 (RX2), **not** pin 19 — pin
   19 is the HC-05's RX line.
3. With both boards powered, open the Mega's USB serial monitor at 115200
   baud — once the ESP32 boots you should see `[cam]` log lines arriving.
