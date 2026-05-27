# EE6003 Robotic Arm — Firmware

PlatformIO workspace for the 4-DOF wireless arm: handheld two-joystick controller (Arduino Uno R4 Minima) → HC-05 Bluetooth pair → arm (Elegoo Mega 2560 R3) driving 4× MG996R servos.

## Layout

```
pio-project/
├── platformio.ini            # 3 envs: arm_mega, controller_uno_r4, hc05_configure
├── WIRING.md                 # read this FIRST — power, HC-05 divider, pin map
├── lib/
│   └── ArmProtocol/          # shared 7-byte framed packet + decoder
├── scripts/                  # python helpers (flash both envs, home hotkey)
└── src/
    ├── arm_mega/             # firmware for the Mega (on the arm)
    ├── controller_uno_r4/    # firmware for the Uno R4 (handheld)
    └── hc05_configure/       # one-time HC-05 AT-mode pairing helper
```

## Quick start

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) (or the VS Code extension — the extension wraps the same `pio` CLI).

```bash
cd "pio-project"

# Build everything — catches errors in all three sketches up front
pio run

# Upload the arm firmware
pio run -e arm_mega -t upload

# Upload the controller firmware (disconnect HC-05 TXD from D0 first!)
pio run -e controller_uno_r4 -t upload

# Watch debug output
pio device monitor -e arm_mega -b 115200
```

### One-shot orchestrators

Two Python helpers in `scripts/` (see [`scripts/README.md`](scripts/README.md)):

```bash
# Build + upload both environments back-to-back, with a prompt in the
# middle to disconnect/reconnect the HC-05 TXD jumper on the Uno R4.
python scripts/flash_all.py

# Laptop hotkey for the home pose. Press 'h' to snap home, 'q' to quit.
# Requires `pip install pyserial` once.
python scripts/home.py
```

## Bring-up order

1. **Read [WIRING.md](WIRING.md)** end to end. Wire the servos + Mega with USB power only.
2. Flash `arm_mega`, verify all 4 servos move to their initial pose when the 6 V bench PSU is switched on.
3. (Already done in this build — the HC-05 pair is configured and bonded. If you ever swap modules: flash `hc05_configure` to the Mega and walk the AT-command sequence to set one as SLAVE → "ARM", one as MASTER → "CTRL", then `AT+BIND` the master to the slave's MAC.)
4. Wire up the controller (Uno R4 + **two** KY-023 joysticks + master HC-05).
5. Flash `controller_uno_r4`. Power both sides — LEDs on both HC-05s should go to slow double-blink within ~5 seconds. Both joysticks drive the arm.

## Known pitfalls

- **HC-05 RX is 3.3 V logic.** The wiring guide shows the 1 kΩ (top) / 2 kΩ (bottom) voltage divider. Skipping this slowly destroys the module.
- **Uno R4 Minima D0/D1** are shared with USB upload. Unplug the HC-05 TXD from D0 while uploading, then reconnect.
- **Servo power ≠ Arduino power.** 4× MG996R can briefly pull >5 A. The Mega's 5 V rail cannot supply that — power servos from the bench PSU only, with common ground back to the Mega.

## Control scheme (two joysticks + claw pot, 4 DOF)

| Input                              | Joint                          |
|------------------------------------|--------------------------------|
| Joystick 1 (left) — Y axis         | shoulder (rate-controlled)     |
| Joystick 2 (right) — Y axis        | elbow (rate-controlled)        |
| Joystick 1 SW held                 | base rotates CCW               |
| Joystick 2 SW held                 | base rotates CW                |
| Claw potentiometer (A4)            | claw (absolute, "only while turning") |

- Shoulder/elbow are **rate-controlled** with a cubic expo curve so small deflections produce gentle motion and full deflection hits `MAX_RATE`.
- The claw pot only overwrites the commanded claw angle while the knob is *actively turning* (see the `POT_STILL_MS` note in `controller_uno_r4/main.cpp`). Lets the `h` USB hotkey home the claw without the pot immediately stomping it.
- Type `h` (or `H`) in the controller's USB serial monitor to snap all four joints back to the HOME pose. There is no long-press home gesture in this firmware.

## Next steps (for the EE6003 60%+ bands)

The brief caps you at 60% unless you demonstrate advanced features. Once this baseline is working, easy additions that earn the higher band:

1. **Closed-loop on one joint** — add a potentiometer (or the MG996R's internal feedback if you open one up and solder to its wiper) and run a PID loop on the Mega. You already have the slew step to build on.
2. **Autonomous sort mode** — a second mode on the controller that ignores the joystick and plays back a recorded pose sequence. The `ArmProtocol::Frame` is already the right shape to log.
3. **Battery telemetry** — Uno R4's ADC can read the 4xAA pack through a divider. Stream voltage back in an unused flags bit or a new protocol field and warn when it sags.
