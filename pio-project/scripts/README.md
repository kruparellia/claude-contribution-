# scripts/

Helper Python scripts for everyday workflows.

## `flash_all.py` — build + upload both environments

Sequential flash of the arm and the controller, with a pause in the
middle so you can disconnect the HC-05 TXD jumper on the Uno R4.

```bash
# from anywhere (script auto-locates platformio.ini):
python scripts/flash_all.py
python scripts/flash_all.py --skip-build
python scripts/flash_all.py --only controller_uno_r4
```

Requires PlatformIO Core on `PATH` (`pio` or `platformio`). No Python
deps beyond the stdlib.

## `home.py` — laptop home-pose hotkey

Opens the controller's USB serial port and forwards keypresses to the
firmware. Press **`h`** to snap the arm home, **`q`** to quit. Useful
during demos and tuning.

```bash
pip install pyserial   # one-time

python scripts/home.py             # interactive (h = home, q = quit)
python scripts/home.py --once      # send one home and exit
python scripts/home.py --port COM5 # override auto-detection
```

The script just sends the byte `h` over USB serial. The firmware
(`src/controller_uno_r4/main.cpp`) drains `Serial` every loop tick and
calls `snapHome()` when it sees `'h'` or `'H'`. The Bluetooth link is
untouched — the home command rides over the same 50 Hz frame stream
that the joysticks use.
