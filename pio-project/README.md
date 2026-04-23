# EE6003 Robotic Arm — Firmware

PlatformIO workspace for the 4-DOF wireless arm: handheld joystick controller (Arduino Uno R4 Minima) → HC-05 Bluetooth pair → arm (Elegoo Mega 2560 R3) driving 3× MG996R + 1× SG90 servos.

## Layout

```
pio-project/
├── platformio.ini            # 3 envs: arm_mega, controller_uno_r4, hc05_configure
├── WIRING.md                 # read this FIRST — power, HC-05 divider, pin map
├── lib/
│   └── ArmProtocol/          # shared 7-byte framed packet + decoder
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

## Bring-up order

1. **Read [WIRING.md](WIRING.md)** end to end. Wire the servos + Mega with USB power only.
2. Flash `arm_mega`, verify all 4 servos move to their initial 90° pose when powered from the 4xAA pack.
3. Flash `hc05_configure` to the Mega, run through the AT-command sequence for each HC-05 module (one as SLAVE → "ARM", one as MASTER → "CTRL"). Write down the slave's MAC address and `AT+BIND` the master to it.
4. Re-flash `arm_mega`. Wire up the controller (Uno R4 + joystick + master HC-05).
5. Flash `controller_uno_r4`. Power both sides — LEDs on both HC-05s should go to slow double-blink within ~5 seconds. Joystick drives the arm.

## Known pitfalls

- **HC-05 RX is 3.3 V logic.** The wiring guide shows the 1 kΩ (top) / 2 kΩ (bottom) voltage divider. Skipping this slowly destroys the module.
- **Uno R4 Minima D0/D1** are shared with USB upload. Unplug the HC-05 TXD from D0 while uploading, then reconnect.
- **Servo power ≠ Arduino power.** 4 servos easily pull > 1 A, which a 500 mA USB port cannot supply without browning out the Mega mid-frame. Always power servos from the battery pack, with common ground to the Mega.

## Control scheme (one joystick, 4 DOF)

- Joystick **X / Y**: rate control (velocity) on the current axis pair.
- **Short press SW**: toggle between axis pair A (base + shoulder) and pair B (elbow + claw).
- **Long press SW (≥ 1 s)**: snap the arm back to the home pose (all axes 90°).

## Next steps (for the EE6003 60%+ bands)

The brief caps you at 60% unless you demonstrate advanced features. Once this baseline is working, easy additions that earn the higher band:

1. **Closed-loop on one joint** — add a potentiometer (or the MG996R's internal feedback if you open one up and solder to its wiper) and run a PID loop on the Mega. You already have the slew step to build on.
2. **Autonomous sort mode** — a second mode on the controller that ignores the joystick and plays back a recorded pose sequence. The `ArmProtocol::Frame` is already the right shape to log.
3. **Battery telemetry** — Uno R4's ADC can read the 4xAA pack through a divider. Stream voltage back in an unused flags bit or a new protocol field and warn when it sags.
