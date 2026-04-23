# EE6003 Robotic Arm — Wiring, Power & Bring-up Guide

Read this **before** plugging anything into power. Servos can pull enough current to brown-out an Arduino or reboot the HC-05 mid-frame, so the order of operations matters.

> 📐 **Visual diagrams**: open [`diagrams/wiring.html`](diagrams/wiring.html) in a browser for the full-colour wiring pictures that go with the sections below. Individual SVGs are in the same folder if you want to reuse them in your report.

---

## 1. Power architecture (the most important part)

The golden rule for servo projects:

> **Servos get their own power rail. Arduinos get USB. The grounds meet in the middle.**

```
    4xAA pack (~6 V, fresh alkalines)
        +  ───────────────┬─────────────────┐
                          │                 │
                          │                 │
                     servo VCC         (100–470 µF
                     (red wires)       electrolytic,
                          │            + to V+, - to GND)
                          │
        –  ───────────────┴──────────────┬──────────── Mega GND
                                         │
                                         └────────── Uno R4 GND (once
                                                     you move to the
                                                     handheld controller)

    USB cable from laptop ────────── Mega +5V/GND (Arduino-only)
```

Key points:

- **Do NOT connect the 4xAA pack's + to the Mega's 5V or VIN.** The pack is only for the servos. The Mega takes its 5V from USB during bring-up.
- **Common ground is mandatory.** Run a wire from the battery pack’s `–` rail to a GND pin on the Mega, otherwise the servo pulse signal has no reference and the servos will twitch randomly.
- **Bulk capacitor on the servo rail.** Solder or breadboard at least a 220 µF electrolytic between servo V+ and GND, close to the servos. It absorbs current spikes when all 4 servos accelerate at once.

### Can the 4xAA pack actually run the arm?

Short answer: yes for bring-up and demos, marginal for aggressive motion.

- 4× fresh alkaline AAs ≈ 6.0 V off-load, ~5.5 V under 1 A load. MG996R spec window is 4.8–7.2 V.
- Peak stall current for one MG996R ≈ 2.5 A. Four stalling at once = ~10 A, which no AA pack will deliver — the voltage will collapse and the Mega will reset.
- Typical moving current per MG996R ≈ 0.5–0.9 A. Four moving smoothly ≈ 2–3 A, which alkalines can supply but will drain the pack in under an hour.
- **For the final demo**, consider a 2S LiPo (7.4 V) + 5 V BEC rated at 5 A, or a bench PSU set to 6.0 V with 3 A limit. This is an easy “advanced feature” you can mention in the report.

If at any point during testing you hear the arm buzz, then see the Mega’s red LED dim, then reboot — that’s brown-out. Stop, the pack is sagging under load.

---

## 2. Breadboard layout

Your breadboard has two power rails along each long edge. Use them like this:

```
  Top red rail   ──── +5V from Mega (for HC-05 and joystick VCC)
  Top blue rail  ──── GND  (tied to Mega GND AND battery pack GND)
  Bottom red rail ─── +6V from 4xAA pack (servos only)
  Bottom blue rail ── GND  (same net as top blue rail — link them!)
```

Put a jumper wire between the two blue (GND) rails so they’re the same node. **Never** jumper the two red rails — they are different voltages.

---

## 3. Servo wiring (Mega = arm side)

Servo cables are three wires. Convention:

| Wire colour | Meaning | Goes to                     |
|-------------|---------|-----------------------------|
| Brown / black | GND   | Bottom blue rail (GND)      |
| Red         | V+      | Bottom red rail (+6V)       |
| Orange / yellow | Signal | Mega digital pin (below) |

| Joint     | Servo    | Mega pin | Notes                                   |
|-----------|----------|----------|-----------------------------------------|
| Base      | MG996R   | **D9**   | Base yaw                                |
| Shoulder  | MG996R   | **D10**  | Lower arm pitch                         |
| Elbow     | MG996R   | **D11**  | Upper arm pitch (via parallelogram)     |
| Claw      | SG90     | **D6**   | Gripper open/close                      |

If your mechanical build has the elbow servo driven through a linkage (the EEZYbotARM pattern), that's fine — the firmware still treats it as one degree of freedom.

---

## 4. HC-05 wiring (both modules, same recipe)

The HC-05's **RX pin is 3.3 V logic**. The TX pins on both the Mega and the Uno R4 Minima are 5 V. Sending 5 V straight into the HC-05's RX works for a while and then kills the module. Use a **voltage divider** on the RX line:

```
   Mega D18 (TX1) ──[ R1 = 1 kΩ ]──┬──→  HC-05  RXD
                                   │
                               [ R2 = 2 kΩ ]
                                   │
                                  GND
```

Output at the junction: `V_out = 5 V × R2 / (R1 + R2) = 5 × 2/3 ≈ 3.3 V`. The HC-05's TX line goes straight into the Mega's RX1 (5 V boards read 3.3 V as logic HIGH fine, so no divider needed on that side).

**See also**: [diagrams/02_hc05_voltage_divider.svg](diagrams/02_hc05_voltage_divider.svg) for the close-up.

### Arm side (Mega + slave HC-05):

| HC-05 pin | Mega pin       |
|-----------|----------------|
| VCC       | 5V             |
| GND       | GND rail       |
| TXD       | **D19 (RX1)**  |
| RXD       | **D18 (TX1)** via 1 kΩ / 2 kΩ divider |
| EN / KEY  | leave floating, or wire to a tact switch for AT-mode entry |
| STATE     | optional — D4 if you want a "connected" LED |

### Controller side (Uno R4 Minima + master HC-05):

| HC-05 pin | Uno R4 pin     |
|-----------|----------------|
| VCC       | 5V             |
| GND       | GND            |
| TXD       | **D0 (RX1)**   |
| RXD       | **D1 (TX1)** via 1 kΩ / 2 kΩ divider |

**⚠ Gotcha:** the Uno R4's D0/D1 are shared with the USB-Serial bridge. When the HC-05 TX is wired to D0, the Arduino IDE/PlatformIO can't upload firmware — the bootloader sees traffic from the Bluetooth module and gets confused. **Unplug the HC-05 TXD wire before uploading**, then reconnect it after upload finishes.

---

## 5. Joystick HW-504 wiring (controller side)

| HW-504 pin | Uno R4 pin    |
|------------|---------------|
| GND        | GND           |
| +5V        | 5V            |
| VRx        | **A0**        |
| VRy        | **A1**        |
| SW         | **D2**        |

`SW` is an open-drain push-button; the firmware uses `INPUT_PULLUP` so no external resistor is needed.

---

## 6. First-power checklist

Before touching any battery, verify with a multimeter:

1. **Continuity** — battery pack `–` → Mega GND → breadboard top blue rail → bottom blue rail. All one net.
2. **Isolation** — battery pack `+` must NOT be connected to Mega 5V, Mega VIN, or the top red rail.
3. **Divider direction** — on both HC-05 RX lines, the **1 kΩ** resistor is on the Arduino side (R1, upper leg) and the **2 kΩ** resistor is on the GND side (R2, lower leg). Swap them and the module sees ~1.7 V instead of 3.3 V, and won't respond.
4. **Signal pins, not power pins** — triple-check that each servo's orange/yellow wire lands on D6/9/10/11, not on 5V or GND.

Then, in order:

1. Plug Mega into laptop via USB. Red LED on Mega steady. Upload `arm_mega` firmware.
2. Open Serial Monitor at 115200. You should see `[arm] boot OK — waiting for frames on Serial1`.
3. Unplug Mega USB. Plug 4xAA pack into the bottom rail. Plug Mega USB back in.
4. All four servos should twitch to their initial pose (90°). If one is way off, the linkage is offset — note the offset, we'll calibrate in firmware.
5. Flash `controller_uno_r4` to the Uno R4 Minima (remember: disconnect HC-05 TXD from D0 before upload).
6. Reconnect HC-05 TXD. Power both Arduinos. Within a few seconds the master HC-05's LED should go from fast-blink to slow double-blink = paired.
7. Wiggle the joystick. Arm should move smoothly. Press SW to swap to elbow/claw mode.

---

## 7. Troubleshooting

| Symptom                                         | Likely cause                                 | Fix                                                   |
|-------------------------------------------------|----------------------------------------------|-------------------------------------------------------|
| Servos twitch violently on power-up, Mega reboots | Mega 5V powering servos (brown-out)        | Move servo V+ to the 4xAA rail. Check common ground.  |
| Arm moves but shudders / jitters                | Missing bulk capacitor, or loose GND        | Add 220–470 µF across the servo rail; re-seat GND.    |
| HC-05 LEDs never pair (both fast-blink)         | Master not bound to slave's MAC             | Re-run `hc05_configure`, redo `AT+BIND=<slave addr>`. |
| Paired but arm doesn't move                     | Baud mismatch or RX/TX swapped              | Confirm both modules set to 9600 via `AT+UART?`. Swap RX/TX if needed. |
| Can't upload to Uno R4                          | HC-05 TXD still wired to D0                  | Disconnect D0 wire, upload, reconnect.                |
| Joystick axis reversed from what you expected   | Physical orientation of the module          | Either rotate the joystick 90°, or negate `jx`/`jy` in `controller_uno_r4/main.cpp`. |
| Arm drifts slowly even with joystick centred    | Deadzone too small for your joystick         | Increase `JOY_DEADZONE` from 60 to ~100 in `controller_uno_r4/main.cpp`. |
