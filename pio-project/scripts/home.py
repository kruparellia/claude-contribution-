#!/usr/bin/env python3
"""
EE6003 robotic arm — laptop-side home-pose hotkey.

Opens the Uno R4 controller's USB serial port and forwards keyboard
presses to the firmware. Press 'h' (any time) to snap the arm to the
home pose. Press 'q' to quit.

Two modes:

    python scripts/home.py                # interactive: 'h' = home, 'q' = quit
    python scripts/home.py --once         # send a single home command and exit

The controller firmware listens on USB Serial for the byte 'h' (or 'H').
That's the same wire you use for the PlatformIO serial monitor — it does
NOT touch the Bluetooth link, so the arm gets the home command via the
HC-05 normally.

Port detection:
- By default we look for the first USB-serial port whose description /
  USB VID:PID looks like an Arduino Uno R4 Minima.
- Override with --port COM5 (Windows) or --port /dev/ttyACM0 (Linux/Mac).

Requires `pyserial` (pip install pyserial).
"""
from __future__ import annotations

import argparse
import sys

try:
    import serial  # type: ignore[import-not-found]
    from serial.tools import list_ports  # type: ignore[import-not-found]
except ImportError:
    sys.exit("error: pyserial not installed. Run: pip install pyserial")

UNO_R4_HINTS = ("Uno R4", "Renesas", "DFU", "ACM", "USB Serial")
# Renesas RA4M1 (Uno R4 Minima) USB CDC PID
UNO_R4_VIDPIDS = {(0x2341, 0x0069), (0x2341, 0x1002)}


def autodetect_port() -> str | None:
    candidates = list(list_ports.comports())
    for p in candidates:
        if p.vid is not None and p.pid is not None:
            if (p.vid, p.pid) in UNO_R4_VIDPIDS:
                return p.device
    for p in candidates:
        desc = (p.description or "") + " " + (p.manufacturer or "")
        if any(h.lower() in desc.lower() for h in UNO_R4_HINTS):
            return p.device
    return None


def read_one_keypress() -> str:
    """Return a single character from stdin without waiting for Enter."""
    try:
        import msvcrt  # Windows
    except ImportError:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    else:
        ch = msvcrt.getwch()
        return ch


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    ap.add_argument("--once", action="store_true", help="send one home command and exit")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if port is None:
        sys.exit(
            "error: could not auto-detect the Uno R4 controller port.\n"
            "Pass --port explicitly. Available ports:\n  "
            + "\n  ".join(f"{p.device}  {p.description}" for p in list_ports.comports())
        )

    try:
        ser = serial.Serial(port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"error: could not open {port}: {e}")

    print(f"connected to {port} @ {args.baud} baud")

    try:
        if args.once:
            ser.write(b"h\n")
            ser.flush()
            print("home command sent.")
            return

        print("press 'h' to home, 'q' to quit.")
        while True:
            ch = read_one_keypress()
            if not ch:
                continue
            if ch in ("q", "Q", "\x03"):  # q or Ctrl-C
                print("\nbye.")
                return
            if ch in ("h", "H"):
                ser.write(b"h\n")
                ser.flush()
                print("home.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
