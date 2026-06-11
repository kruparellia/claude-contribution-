# EE6003 Robotic Arm — Future Work & Implementation Guides

Three planning documents:

1. [Replacing the on-screen UI with a physical LED + button panel](#1-physical-control-panel-leds--buttons)
2. [Unresolved issues and gaps still to hit](#2-unresolved-issues--brief-gaps)
3. [Adding more colours (multi-class sort to per-colour drop zones)](#3-adding-more-colours--per-colour-drop-zones)

---

## 1. Physical control panel (LEDs + buttons)

**Goal:** run the autonomous pick-and-place cycle with no screen and no keyboard —
just a few buttons to drive it and a few LEDs to read its state.

### 1.1 Key idea: separate *setup* from *run*

The current `pickup_runner.py` mixes two jobs in one screen:

- **Setup / calibration (needs a screen, done occasionally):** the trackbars
  (`V_min`, `S_max`, `V_max`, `MinArea`, `MaxArea`), clicking the base centre,
  flipping the servo sign, the ArUco/Jacobian probe, phi trim.
- **Runtime operation (the daily loop):** essentially one human action —
  press **GO** at `READY` — plus *stop* and *re-arm*.

Almost everything setup-related is **already persisted to disk**
(`refs/base_centre.json`, `refs/claw_phi.json`, `refs/servo_jacobian.json`).
So the plan is: keep a screen-based **setup mode** for calibration, and add a
**headless run mode** that loads those saved values and uses GPIO instead of the
keyboard/HUD. The spot auto-locks from the clean scene, so no screen is needed at
run time once the camera/arm are fixed.

> One missing piece to persist: the five HSV/area threshold trackbar values.
> Add a `refs/thresholds.json` that setup mode writes and run mode loads (mirrors
> how the centre/phi/Jacobian are already saved/loaded).

### 1.2 Where it connects

The state machine runs **on the Raspberry Pi**, so the panel wires to the **Pi's
GPIO header** (not the Mega). No conflict with existing hardware — the camera and
the Mega are both on USB, leaving the GPIO free. Use the **`gpiozero`** library
(`Button`, `LED`/`PWMLED`, optionally `RGBLED`) — it has clean event callbacks and
handles pull-ups and debouncing for you.

> The firmware-level **e-stop rocker stays on the Mega** (D2). That is the hard
> safety stop and must remain independent of the Pi — do not move it to the Pi
> panel. The panel buttons below are *soft* controls.

### 1.3 Buttons (momentary push-to-make)

| Button | Colour | Replaces | Action |
|--------|--------|----------|--------|
| **GO / AUTHORISE** | green | `SPACE` | At `READY`: authorise the pickup (→ hover → servo → grasp → drop, hands-off). If `AUTO_GRASP=False`, also fires the grasp at `ALIGNED`. |
| **STOP / RESET** | red | `X` | Soft abort: cancel the current cycle, send `x`/home, return to `IDLE`. (This is *not* the e-stop — that's the rocker.) |
| **MODE / HOME** *(optional)* | blue/black | `A` / `H` | Re-arm auto + home the arm (recover after an e-stop or a STOP). |

Two buttons (GO, STOP) are the minimum; the third makes recovery one-touch.

**Wiring each button:** one leg → a GPIO pin, other leg → GND. Use the internal
pull-up (`Button(pin)` defaults to `pull_up=True`), so a press reads as a
falling edge. No external resistor needed.

### 1.4 LEDs (status without a screen)

Drive each LED from a GPIO pin through a **~330 Ω** resistor to GND. Suggested set:

| LED | Colour | Meaning |
|-----|--------|---------|
| **ARMED** | blue | auto mode is ON / system alive (blink while homing or no spot lock yet) |
| **READY** | green | rock on the spot, **press GO** (steady or slow blink to draw the eye) |
| **BUSY** | amber | a cycle is running (HOVERING/SERVOING/GRASPING/VERIFYING) |
| **RESULT** | green/red | last verification: **green = PICKUP OK**, **red = PICKUP FAILED** (hold ~3 s) |
| **FAULT** | red | e-stop engaged (from the arm's `estop=yes`) or a servo/grasp timeout |

If you want fewer wires, replace the lot with **one RGB LED** and encode state by
colour + blink pattern (e.g. white=idle, green pulse=ready, amber=busy,
green=ok, red=fail, fast-red=e-stop). One RGB LED = 3 GPIO + 3 resistors.

**Suggested BCM pins** (all free on a standard Pi; adjust to taste):

```
Buttons:  GO=GPIO17   STOP=GPIO27   MODE=GPIO22
LEDs:     ARMED=GPIO5 READY=GPIO6   BUSY=GPIO13  RESULT_G=GPIO19  RESULT_R=GPIO26  FAULT=GPIO16
```

### 1.5 State → LED mapping

| Runner state | ARMED | READY | BUSY | RESULT | FAULT |
|--------------|:----:|:----:|:----:|:------:|:----:|
| HOMING | blink | – | – | – | – |
| IDLE (no spot lock) | blink | – | – | – | – |
| IDLE (locked, waiting rock) | on | – | – | – | – |
| LOCKING | on | blink | – | – | – |
| READY | on | **on** | – | – | – |
| HOVERING / SERVOING / ALIGNED / GRASPING | on | – | **on** | – | – |
| VERIFYING | on | – | blink | – | – |
| COOLDOWN (after OK) | on | – | – | **green 3 s** | – |
| COOLDOWN (after FAIL) | on | – | – | **red 3 s** | – |
| E-stop engaged (`estop=yes`) | off | off | off | – | **fast blink** |

### 1.6 Software changes (no rewrite, just an I/O swap)

1. **`vision/panel.py`** — a small hardware-abstraction module:
   - `class Panel`: constructs the gpiozero `Button`/`LED` objects.
   - `set_state(name, pickup_ok=None)` — maps a runner state string to the LED
     pattern table above (LED blink can be done with `PWMLED.blink()` /
     `LED.blink()`, which run in the background).
   - Button events: register `when_pressed` callbacks that set thread-safe flags
     (`go_pressed`, `stop_pressed`, `mode_pressed`) the main loop polls and clears.
   - A `--no-gpio` fallback / stub so the file still imports on a dev machine.
2. **`pickup_runner.py`** — add a `--headless` flag:
   - Skip `cv2.namedWindow` / trackbars / `imshow`; still `cap.read()` each frame.
   - Load thresholds from `refs/thresholds.json` instead of `getTrackbarPos`.
   - Replace the `cv2.waitKey` key handling: `GO → the SPACE branch`,
     `STOP → the X branch`, `MODE → the A/H branch`.
   - On every `transition()`, call `panel.set_state(...)`; on a verification
     verdict, call `panel.set_state("COOLDOWN", pickup_ok=...)`.
   - Poll the arm's `?` `estop=` field (already emitted by the firmware) and drive
     the FAULT LED.
3. **Auto-start (truly screen/keyboard-free):** install a small **systemd
   service** that runs `python3 vision/pickup_runner.py --headless` on boot, so
   the rig powers up straight into run mode. (`Restart=on-failure`,
   `After=multi-user.target`.)

### 1.7 Bring-up order

1. Wire one LED + one button, test in isolation with a 5-line `gpiozero` script.
2. Build `panel.py`; verify the LED table by stepping states by hand.
3. Run `pickup_runner.py --headless` with the arm in `--dry-run` first (LEDs +
   buttons only, no motion), then live.
4. Add the systemd service once it's reliable.

---

## 2. Unresolved issues & brief gaps

> I don't have the assignment brief text, so the "brief gaps" below are inferred
> from the project goals on record (4-DOF wireless arm, camera, **autonomous
> sort**, **closed-loop control**, 60 %+ band). **Cross-check against the actual
> brief** — anything it names that isn't ticked here is a priority.

### 2.1 Likely brief requirements not yet met (highest priority)

- **Autonomous colour sorting — NOT done.** Only one colour (red) → one drop
  zone. If the brief says "sort," this is the biggest gap. See §3 for the plan.
- **Joint-level closed-loop control (PID with feedback) — NOT done.** The only
  closed loop today is the *vision* servo on the base; the four joints run
  **open-loop** (no positional feedback — the firmware reports *commanded*, not
  measured, angles). A potentiometer/encoder + PID on one joint would
  substantiate a "closed-loop control" claim and lift the grade band.
- **Battery / power telemetry — NOT done.** No voltage/current monitoring
  (planned via the Uno R4 ADC). Easy marks if the brief wants telemetry.

### 2.2 Technical debt / known bugs

- **Temporary `[rx]` debug code still in the firmware** (`g_rxDebug`, default
  **ON**, 2 Hz dump). Strip it before the final build/demo — it clutters the
  serial log and wastes cycles.
- **Noisy claw potentiometer.** Worked around on the arm side (claw excluded from
  the takeover check, `FLAG_BUTTON` dropped from "driving"), but **not fixed at
  source** — the claw still jitters ±3 in *manual* mode. Proper cure is
  controller-side: raise `POT_NOISE` and/or smooth the pot reading in
  `controller_uno_r4`.
- **Shape-gate flicker at some viewpoints.** Mitigated (rock debounce 2/5, wider
  refs); backup lever is raising `MATCH_MAX_TOTAL` (12 → ~15) if live detection
  is still marginal at the final lighting.
- **Base rotation centre is approximate** (rough disc-ellipse / click-set, biased
  while the disc is clipped at the frame edge). Tolerated because the visual servo
  re-measures and self-corrects, but it's a known inaccuracy.
- **`HOVER` shoulder = 97° sits in the stall zone.** Holding the hover for a long
  servo loop risks a shoulder stall / rail brown-out. Mitigation if it stalls:
  raise the hover (lower shoulder angle / fold the elbow more) and re-record.

### 2.3 Robustness / edge-case gaps

- **No grasp-fail recovery.** Verification *detects* a failed pickup (rock still
  on the spot) but only **reports** it — there's no auto-retry. Add a
  retry-on-FAIL (re-servo + re-grasp, N attempts) for a stronger demo.
- **No drop confirmation.** We confirm the rock *left* the pickup spot, not that
  it *landed* in the drop zone. A shape-gated check over the drop region would
  close the loop (the shape gate beats the red-tape-vs-red-rock colour clash).
- **Single object at a time.** No handling of multiple rocks queued on/near the
  spot, or a rock that arrives mid-cycle.
- **Rig-specific calibration is fragile.** Any camera/arm reposition invalidates
  the centre, phi, Jacobian, thresholds **and** the recorded keyframes (drop
  zone, hover, grasp). Document a single "re-calibrate after moving the rig"
  checklist so a bump before the demo is recoverable.
- **ArUco markers only decode over a front arc** (planar foreshortening). Fine for
  the fixed front pickup zone, but limits any future full-rotation tracking;
  bigger markers help more than detector-param tuning (already tested & rejected).
- **Stall/timeout handling is coarse.** A stuck sequence relies on the Pi's
  `GRASP_TIMEOUT_S` → home. A stalled servo still browns out the 6 V rail until
  the interlock/home relieves it; there's no current sensing to catch it actively.

### 2.4 Process / deliverables (verify against the brief)

- Final report write-up, wiring diagrams, and a demo video — not assessable here;
  make sure the brief's documentation deliverables are covered.
- Confirm the **wireless** requirement is satisfied as specified (HC-05 BT link is
  in place) and that **safety** (now: damped e-stop) is documented.

---

## 3. Adding more colours — per-colour drop zones

**Goal:** several rock colours, all **picked from the same spot**, each **dropped
in its own designated zone**.

### 3.1 What changes (and what doesn't)

- **Pickup is colour-independent** — same spot, same hover, same grasp. *Nothing*
  in the pickup/grasp phase changes.
- **Only two things are new:** (a) **classify** the rock's colour at authorise
  time, and (b) **select that colour's drop zone** for the drop phase.

### 3.2 Detection: detect by shape, classify by colour

The cleanest design, because all rocks are the **same object in different
colours**:

- **Make the shape gate geometry-only.** The current 7-feature signature includes
  `mean_h/s/v`, which is colour-specific. Drop the three colour features (or build
  a separate geometry-only signature) so **one** shape reference set matches a
  rock of *any* colour. Geometry (aspect, solidity, extent, circularity) is what
  identifies "it's a rock"; colour then says *which* rock.
- **Iterate the colour masks.** `colour_detect.py` already has a `COLOURS` list of
  HSV ranges and `_make_mask(hsv, colour)`. For each entry, build its mask → find
  contours → shape-gate. The mask that yields a valid rock blob *is* the colour.
- **`find_rock()` returns `(centroid, colour_name)`** instead of just a centroid.
  Handle the rare "two colours both match" case by taking the larger / better
  shape score.

**Setup per new colour:**
1. Add the colour's HSV range to `COLOURS` in `colour_detect.py` (tune with
   `colour_detect.py`'s live view).
2. (If you keep colour in the shape signature instead of going geometry-only)
   capture refs per colour with `capture_refs.py` and rebuild with
   `build_shape_refs.py`. **Geometry-only is recommended** to avoid this per-colour
   recapture.

### 3.3 Drop zones: one selectable base angle per colour

All zones at the **same radius** from the base axis differ only by **base angle**,
so the drop keyframes can be shared and only the base swung per colour.

**Firmware (recommended — runtime-selectable drop base):**
- Add a second sentinel next to `BASE_KEEP`, e.g. **`BASE_DROP = 254`**.
- In the drop-phase keyframes (`SWING`, `DROPDN`, `RELEASE`, `LIFTOUT`), set
  `base = BASE_DROP`.
- Add a global `g_dropBase` and, in `seqApplyKeyframe()`, substitute it when a
  keyframe's base is `BASE_DROP` (just like `BASE_KEEP` is skipped).
- Extend the grasp command to carry the drop angle: **`g<deg>`** → parse the int,
  set `g_dropBase`, then `seqStartGrasp()`. (Mirror the existing `b<deg>` parser.)

This keeps **one** grasp+drop sequence; the drop direction is just a number sent
at launch. Adding a colour = adding a number, no new keyframes.

> Alternative (simpler to picture, more to record): store a **separate full
> `GRASP_DROP_SEQUENCE` per colour** and select with `g<n>` indexing an array.
> Heavier to record and maintain — prefer the parameterised base above.

**Config:** a `refs/drop_zones.json` mapping colour → base angle, e.g.
`{"red": 180, "blue": 120, "green": 60}`. Record each by jogging the claw over
that zone and reading the `[pos]` `base=` value (same method as the original
`B_DROP`).

### 3.4 Pi orchestration

- In `arm_link.py`: add `grasp(drop_base: int)` → `self.send_line(f"g{drop_base}")`.
- In `pickup_runner.py`:
  - `find_rock()` now yields the colour; show it on the HUD/LED.
  - At authorise (`READY`/SPACE/GO), look up `drop_zones[colour]` and pass it into
    the grasp call (and freeze the chosen colour for the cycle).
  - Verification is unchanged (pickup spot empty). Optionally add a shape-gated
    check over the selected colour's drop region to confirm the *drop*.

### 3.5 Constraints / gotchas

- **Zones must not overlap** each other or the pickup area, and all must be within
  the base range (`LIM_BASE` 0–180) and ideally the **same radius** (so the shared
  drop keyframes' shoulder/elbow reach is correct). If a zone needs a different
  radius, it needs its own `DROPDN`/`RELEASE` shoulder-elbow — fall back to the
  per-colour-sequence alternative for that zone.
- **Swing path:** check the arm clears obstacles when swinging from the pickup
  base to the furthest drop zone; the carry pose (shoulder up, elbow tucked) is
  what keeps it high during the swing.
- **Ambiguous colour** (e.g. red rock vs red drop tape in frame): the geometry
  shape gate already rejects flat tape, so classification keys off the rock blob,
  not the tape.

### 3.6 Add-a-colour checklist (once the framework above is in)

1. Add the HSV range to `COLOURS`; tune in `colour_detect.py`.
2. Jog the claw over the new drop zone, read `[pos] base=`, add it to
   `drop_zones.json`.
3. (Geometry-only gate → nothing else.) Test: place that colour on the spot →
   confirm it's classified and dropped in the right zone.
