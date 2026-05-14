#!/usr/bin/env python3
"""
EE6003 robotic arm — interactive keyframe recorder.

You drive the arm with the v2 controller (joysticks + pot) as normal.
This script listens to the Mega 2560's USB serial log, tracks the live
joint angles from the `[pos] base=X shoulder=X elbow=X claw=X` lines
the firmware already prints once per second, and walks you through a
pickup sequence one pose at a time.

When you're done, it writes `src/arm_mega/keyframes.h` — a header you can
`#include` from main.cpp containing the captured poses plus an ordered
`PICKUP_SEQUENCE[]` array a state machine can walk through.

Usage:
    python scripts/record_keyframes.py             # auto-detect Mega
    python scripts/record_keyframes.py --port /dev/ttyACM0
    python scripts/record_keyframes.py --dry-run   # don't write the header

While at a prompt:
    Enter   capture the live pose (waits up to ~1.5 s for a fresh [pos] line)
    r       redo the previous keyframe
    s       skip this keyframe (leave previous value, or 0,0,0,0)
    q       quit without writing
    ?       reprint the keyframe description

Requires `pyserial` (pip install pyserial).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import re
import sys
import threading
import time
from pathlib import Path

try:
    import serial  # type: ignore[import-not-found]
    from serial.tools import list_ports  # type: ignore[import-not-found]
except ImportError:
    sys.exit("error: pyserial not installed. Run: pip install pyserial")


# ---------------------------------------------------------------------------
# Keyframe definitions. Edit this list if you want a different sequence.
# `name` becomes the C++ identifier (KF_<NAME>). `hint` is shown at the prompt.
# `claw_hint` is just a reminder; you set the actual claw angle with the pot.
# ---------------------------------------------------------------------------
KEYFRAMES = [
    ("APPROACH",     "Hover above the object, claw OPEN. Roughly 5–10 cm clearance."),
    ("GRASP_DOWN",   "Lower onto the object, claw still OPEN, ready to close."),
    ("GRASP_CLOSE",  "Same arm pose as GRASP_DOWN — just close the claw on the object."),
    ("LIFT",         "Lift back up with the object held, claw CLOSED."),
    ("DROP_OVER",    "Move over the drop zone / bin, claw still CLOSED."),
    ("DROP_RELEASE", "Same arm pose as DROP_OVER — open the claw to release."),
]

# The home pose is hard-coded in firmware (LIM_*.initial). We bookend the
# generated sequence with it so PICKUP_SEQUENCE starts and ends at home.
HOME_POSE = (80, 105, 110, 90)  # base, shoulder, elbow, claw  — matches arm_mega/main.cpp

POS_RE = re.compile(
    r"\[pos\]\s+base=(\d+)\s+shoulder=(\d+)\s+elbow=(\d+)\s+claw=(\d+)"
)

# Genuine Mega 2560 R3, plus the common CH340 clone. Description fallback
# below catches the rest.
MEGA_VIDPIDS = {(0x2341, 0x0042), (0x2341, 0x0010), (0x2A03, 0x0042)}
MEGA_HINTS = ("Mega", "2560", "CH340", "USB-SERIAL", "ACM")


# ---------------------------------------------------------------------------
# Serial plumbing
# ---------------------------------------------------------------------------
def autodetect_port() -> str | None:
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid is not None and p.pid is not None and (p.vid, p.pid) in MEGA_VIDPIDS:
            return p.device
    for p in ports:
        desc = f"{p.description or ''} {p.manufacturer or ''}"
        if any(h.lower() in desc.lower() for h in MEGA_HINTS):
            return p.device
    return None


class PosReader(threading.Thread):
    """Background thread: parse [pos] lines, update shared state under a lock."""

    def __init__(self, ser: "serial.Serial") -> None:
        super().__init__(daemon=True)
        self._ser = ser
        self._lock = threading.Lock()
        self._latest: tuple[int, int, int, int] | None = None
        self._latest_ts: float = 0.0
        self._stop = threading.Event()

    def run(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
            except serial.SerialException:
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                try:
                    text = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                m = POS_RE.search(text)
                if m:
                    pose = tuple(int(x) for x in m.groups())  # type: ignore[assignment]
                    with self._lock:
                        self._latest = pose  # type: ignore[assignment]
                        self._latest_ts = time.monotonic()

    def latest(self) -> tuple[tuple[int, int, int, int] | None, float]:
        with self._lock:
            return self._latest, self._latest_ts

    def wait_fresh(self, timeout: float = 1.5) -> tuple[int, int, int, int] | None:
        """Wait for a [pos] line newer than 'now', up to `timeout` seconds."""
        t0 = time.monotonic()
        deadline = t0 + timeout
        while time.monotonic() < deadline:
            pose, ts = self.latest()
            if pose is not None and ts >= t0:
                return pose
            time.sleep(0.05)
        # Fall back to the most recent cached value if no fresh one arrived.
        pose, _ = self.latest()
        return pose

    def stop(self) -> None:
        self._stop.set()


# ---------------------------------------------------------------------------
# Capture loop
# ---------------------------------------------------------------------------
def prompt_keyframe(name: str, hint: str, reader: PosReader) -> tuple[int, int, int, int] | str:
    """Returns a captured pose, or one of: 'skip', 'redo', 'quit'."""
    print()
    print(f"--- {name} ---")
    print(f"    {hint}")
    while True:
        choice = input("    [Enter]=capture  r=redo last  s=skip  q=quit  ?=help > ").strip().lower()
        if choice == "":
            pose = reader.wait_fresh()
            if pose is None:
                print("    ! no [pos] line received yet. Is the Mega connected and printing?")
                continue
            confirm = input(
                f"    captured base={pose[0]} shoulder={pose[1]} elbow={pose[2]} claw={pose[3]}"
                f" — keep? [Y/n] "
            ).strip().lower()
            if confirm in ("", "y", "yes"):
                return pose
            print("    discarded; try again.")
            continue
        if choice == "r":
            return "redo"
        if choice == "s":
            return "skip"
        if choice == "q":
            return "quit"
        if choice == "?":
            print(f"    hint: {hint}")
            continue
        print(f"    unknown input: {choice!r}")


def capture_all(reader: PosReader) -> list[tuple[str, tuple[int, int, int, int]]] | None:
    """Walk the KEYFRAMES list. Returns None if the user quit."""
    captured: list[tuple[str, tuple[int, int, int, int]]] = []
    i = 0
    while i < len(KEYFRAMES):
        name, hint = KEYFRAMES[i]
        result = prompt_keyframe(name, hint, reader)
        if result == "quit":
            return None
        if result == "redo":
            if i == 0:
                print("    nothing to redo yet.")
                continue
            i -= 1
            captured.pop()
            continue
        if result == "skip":
            print(f"    skipping {name} — will write (0, 0, 0, 0); edit by hand.")
            captured.append((name, (0, 0, 0, 0)))
            i += 1
            continue
        captured.append((name, result))  # type: ignore[arg-type]
        i += 1
    return captured


# ---------------------------------------------------------------------------
# Header generation
# ---------------------------------------------------------------------------
HEADER_TEMPLATE = """\
// pio-project/src/arm_mega/keyframes.h
// ----------------------------------------------------------------------
// Auto-generated by scripts/record_keyframes.py on {timestamp}.
// Re-run the script to recapture, or edit the values below by hand
// (the struct layout is base, shoulder, elbow, claw).
// ----------------------------------------------------------------------
#pragma once
#include <stdint.h>

struct Pose {{
    uint8_t base;
    uint8_t shoulder;
    uint8_t elbow;
    uint8_t claw;
}};

// Home pose — must match LIM_*.initial in main.cpp.
static constexpr Pose KF_HOME {{ {home[0]:3d}, {home[1]:3d}, {home[2]:3d}, {home[3]:3d} }};

{pose_defs}

// Ordered pickup sequence. A state machine in main.cpp can step through
// this array, advancing once all four joints are within tolerance of the
// current target. Edit the order or drop entries to taste.
static constexpr Pose PICKUP_SEQUENCE[] = {{
    KF_HOME,
{seq_lines}
    KF_HOME,
}};
static constexpr uint8_t PICKUP_SEQUENCE_LEN =
    sizeof(PICKUP_SEQUENCE) / sizeof(PICKUP_SEQUENCE[0]);
"""


def render_header(captured: list[tuple[str, tuple[int, int, int, int]]]) -> str:
    pose_defs = "\n".join(
        f"static constexpr Pose KF_{name:<13} {{ {p[0]:3d}, {p[1]:3d}, {p[2]:3d}, {p[3]:3d} }};"
        for name, p in captured
    )
    seq_lines = "\n".join(f"    KF_{name}," for name, _ in captured)
    return HEADER_TEMPLATE.format(
        timestamp=_dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        home=HOME_POSE,
        pose_defs=pose_defs,
        seq_lines=seq_lines,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", help="Mega USB serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=115200, help="baud (default 115200)")
    ap.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parent.parent / "src" / "arm_mega" / "keyframes.h"),
        help="output header path (default: src/arm_mega/keyframes.h)",
    )
    ap.add_argument("--dry-run", action="store_true", help="print the header but don't write it")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if port is None:
        sys.exit(
            "error: could not auto-detect the Mega 2560 port.\n"
            "Pass --port explicitly. Available ports:\n  "
            + "\n  ".join(f"{p.device}  {p.description}" for p in list_ports.comports())
        )

    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: could not open {port}: {e}")

    print(f"connected to {port} @ {args.baud} baud")
    print("listening for [pos] lines from arm_mega (1 Hz)...")

    reader = PosReader(ser)
    reader.start()

    # Wait briefly for the first [pos] line so we know the link is alive.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        pose, _ = reader.latest()
        if pose is not None:
            print(f"link ok — initial pose: base={pose[0]} shoulder={pose[1]}"
                  f" elbow={pose[2]} claw={pose[3]}")
            break
        time.sleep(0.1)
    else:
        print("warning: no [pos] line received in 3 s. Is arm_mega running?")
        print("         (continuing — you can still try capturing once it shows up)")

    print()
    print("You'll be walked through these poses. Drive the arm with the v2")
    print("controller as usual, then press Enter at each step.")

    try:
        captured = capture_all(reader)
    finally:
        reader.stop()
        ser.close()

    if captured is None:
        print("\nquit — nothing written.")
        return

    header = render_header(captured)
    print("\n" + "=" * 60)
    print(header)
    print("=" * 60)

    if args.dry_run:
        print("\n--dry-run set — header not written.")
        return

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(header)
    print(f"\nwrote {out_path}")
    print("next step: `#include \"keyframes.h\"` in arm_mega/main.cpp and wire up the sequence runner.")


if __name__ == "__main__":
    main()
