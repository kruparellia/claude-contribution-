# EE6003 Robotic Arm — Controller Variant **v2** Wiring Addendum

This file documents *only what's different* from `WIRING.md` for the second controller firmware (`src/controller_uno_r4_v2/`). Everything else — power architecture, breadboard rails, HC-05 wiring, first-power checklist — stays exactly as in `WIRING.md`. Read that first.

---

## 1. Control mapping at a glance

| Input                              | Action                                  |
|------------------------------------|-----------------------------------------|
| Joystick 1 (left)  Y axis          | Shoulder (rate control)                 |
| Joystick 1 (left)  X axis          | *unused* (idle in this firmware)        |
| Joystick 1 (left)  SW held down    | Base rotates **anti-clockwise (CCW)**   |
| Joystick 2 (right) Y axis          | Elbow ("the arm", rate control)         |
| Joystick 2 (right) X axis          | *unused*                                |
| Joystick 2 (right) SW held down    | Base rotates **clockwise (CW)**         |
| Rotary potentiometer (wiper on A4) | Sets claw angle (absolute, see §3)      |
| `h` / `H` on USB Serial            | Snap arm to home pose                   |

Every input does exactly one thing — no time-shared gestures. The joystick SWs are pure base-spin (press = spin, release = stop); the pot dials the claw position directly; homing lives on the laptop keyboard.

---

## 2. Joystick wiring — IDENTICAL to v1

Both joysticks are wired exactly as in `WIRING.md §5`. No changes here.

| Joystick     | VRx | VRy | SW  |
|--------------|-----|-----|-----|
| 1 (left)     | A0  | A1  | D2  |
| 2 (right)    | A2  | A3  | D3  |

In v2 the X axes are read but not used to drive any joint — they remain available if you later want to add e.g. fine-trim or speed scaling.

---

## 3. NEW — Rotary potentiometer for the claw

You need **1 × 10 kΩ linear potentiometer** (the standard 3-leg, breadboard-friendly kind from any Arduino starter kit — sometimes labelled "B10K"). Any value from ~1 kΩ to ~100 kΩ works in principle, but 10 kΩ is the sweet spot: low enough to swamp ADC noise, high enough to draw negligible current from the 5 V rail.

### How a pot is wired

A pot has 3 legs. The two outer legs are the ends of a resistor track. The middle leg is the **wiper** — a contact that slides along the track as you turn the knob. The voltage on the wiper depends on where it is along the track:

```
       5 V  ─────┐
                 │
                 ●    ←  one outer leg of the pot
                 ║
                 ║ resistive track
                 ●    ←  wiper (middle leg) -> A4 on the Uno R4
                 ║
                 ║
                 ●    ←  other outer leg
                 │
       GND  ─────┘
```

Turn the knob fully CCW → wiper at 0 V → ADC reads ~0.
Turn the knob fully CW  → wiper at 5 V → ADC reads ~1023.
Midway → ~2.5 V → ADC reads ~512.

### Connections

| Pot leg            | Wire to                       |
|--------------------|-------------------------------|
| One outer leg      | GND (top blue rail)           |
| Other outer leg    | 5 V (top red rail)            |
| Middle leg (wiper) | **A4** on the Uno R4          |

It doesn't matter electrically which outer leg goes to 5 V and which to GND — the only consequence is whether "turn CW" opens or closes the claw. If the direction feels backwards once you bench-test, swap the two outer legs (or invert the mapping in `potToClawDeg()` — one line change).

### Why A4?

A0–A3 are the joystick axes. A5 is left free for future use (e.g. a second pot for shoulder trim). A4 is the next pin on the board after A3, so it sits naturally next to the joystick wiring on the breadboard.

### Why this beats two open/close push-buttons

A pot is *absolute*: knob at position X means claw at angle X, every time. You can pre-set the grip width before you reach the object — particularly useful for the pick-and-place demos. Buttons, by contrast, only let you nudge the claw open/closed and you have to watch where you end up.

### The "only commit while turning" trick

A naive implementation would slam the claw to the pot's reading on every tick. That makes `'h'` for home useless — the very next tick after homing, the pot reading would yank the claw straight back to wherever the knob is sitting.

So the firmware only overwrites `angClaw` from the pot **while the knob is being actively turned**. Concretely: a reading must differ from the last accepted reading by more than ~6 ADC counts to count as "movement". Once the knob has been still for 500 ms, the firmware stops driving the claw from the pot. That means:

- Twist the knob → claw tracks it instantly.
- Stop twisting → claw holds the picked angle.
- Type `h` → all four joints snap to 90°. The claw stays homed because the pot is "parked".
- Twist the knob again → claw snaps to the new knob position.

Both timing values are tunable in the firmware (`POT_NOISE` and `POT_STILL_MS`). The defaults are conservative — if you find the claw chatters slightly while the knob sits still, bump `POT_NOISE` from 6 to 10 or so.

### Shopping list (in addition to what `WIRING.md` lists)

- 1 × 10 kΩ linear potentiometer (B10K, 3-leg, breadboard-friendly)
- 3 × jumper wires (M-M, ~10 cm)

That's it. No tact switches, no resistors, no caps.

---

## 4. Pin usage summary (Uno R4 Minima, v2 firmware)

| Pin    | Direction | Used for                                 |
|--------|-----------|------------------------------------------|
| D0     | RX1       | HC-05 TXD (unchanged from v1)            |
| D1     | TX1       | HC-05 RXD via 1 kΩ/2 kΩ divider          |
| D2     | input PU  | Joystick 1 SW — hold = base CCW          |
| D3     | input PU  | Joystick 2 SW — hold = base CW           |
| A0–A1  | analog in | Joystick 1 VRx / VRy                     |
| A2–A3  | analog in | Joystick 2 VRx / VRy                     |
| A4     | analog in | **NEW** claw potentiometer wiper         |
| A5     | —         | free for future use                      |
| 5V/GND | power     | Joysticks + HC-05 + pot ends             |

D4, D5, D6 are now free. If you later want a *physical* home button, D6 is the obvious place (was used in an earlier draft of this firmware).

---

## 5. Build & flash

```bash
# New v2 scheme — joysticks = shoulder/elbow + SW base spin, pot = claw, 'h' = home
pio run -e controller_uno_r4_v2 -t upload

# Original 4-axis joystick scheme, still available
pio run -e controller_uno_r4 -t upload
```

Same HC-05 gotcha as before: **unplug the HC-05 TXD wire from D0 before uploading**, then reconnect after the flash completes.

---

## 6. Quick functional check (post-flash)

1. Open Serial Monitor at 115200 baud — you should see `[ctl-v2] boot OK — sticks=shoulder/elbow, SW=base spin, pot=claw, 'h'=home`.
2. Wiggle **Joystick 1 Y** — shoulder servo should move; pushing the stick **up** lowers the shoulder angle.
3. Wiggle **Joystick 2 Y** — elbow servo should move similarly.
4. **Press and hold** Joystick 1 SW — base rotates CCW. Release → stops.
5. **Press and hold** Joystick 2 SW — base rotates CW. Release → stops.
6. **Turn the pot knob** — claw should track the knob position. Stop turning, the claw stops moving.
7. **Type `h` in Serial Monitor** — all four joints snap to 90°. Importantly, the claw should *stay* at 90° even though the knob is still in whatever position you left it.
8. **Turn the pot knob again** — claw should snap to track the new knob position.

If the claw direction feels backwards (CW closes when you expected it to open), the easiest fix is to swap the two outer pot legs at the breadboard — that physically inverts the wiper voltage. Alternatively, invert the mapping in `potToClawDeg()` in the firmware.

---

## 7. Tuning knobs (in `controller_uno_r4_v2/main.cpp`)

| Constant       | Default | What it changes                                            |
|----------------|---------|------------------------------------------------------------|
| `MAX_RATE`     | 90 °/s  | How fast shoulder/elbow move at full joystick deflection   |
| `BASE_RATE`    | 60 °/s  | How fast the base rotates while a joystick SW is held      |
| `POT_NOISE`    | 6 ADC counts | Below this much change, the knob is treated as still  |
| `POT_STILL_MS` | 500 ms  | How long the knob must be still before we stop driving claw |
| `DEBOUNCE_MS`  | 25 ms   | How long a button must hold its new state to be accepted   |
| `JOY_DEADZONE` | 60      | Wider = more tolerant of sloppy sticks                     |
