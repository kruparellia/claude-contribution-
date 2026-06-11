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

# Grasp+drop keyframes (--grasp). Recorded from the SERVOED HOVER pose: the Pi
# has already swung the base over the rock, so the base you capture during the
# pickup phase is IGNORED and written as BASE_KEEP (the firmware holds the
# servoed base). The drop phase captures a real base over the red tape.
# `keep_base=True` marks the poses whose base is replaced with BASE_KEEP.
GRASP_KEYFRAMES = [
    ("GD_DESCEND", "Lower onto the rock from the hover, claw OPEN.",          True),
    ("GD_CLOSE",   "Same pose — close the claw on the rock.",                 True),
    ("GD_LIFT",    "Lift: FOLD THE ELBOW BACK (>=165) then raise the "
                   "shoulder, claw CLOSED. The interlock needs the elbow "
                   "tucked or the shoulder won't rise.",                      True),
    ("GD_SWING",   "Rotate the base over the RED-TAPE drop zone, carrying.",  False),
    ("GD_DROPDN",  "Lower over the drop zone, claw still CLOSED.",            False),
    ("GD_RELEASE", "Same pose — open the claw to release the rock.",          False),
    ("GD_LIFTOUT", "Lift clear of the drop zone, claw OPEN.",                 False),
]

# Sentinel written for the base field of keep_base poses — must match the
# BASE_KEEP constant in keyframes.h / main.cpp.
BASE_KEEP = 255

# The home pose is hard-coded in firmware (LIM_*.initial). We bookend the
# generated PICKUP_SEQUENCE with it so it starts and ends at home. The
# grasp+drop sequence does NOT bookend home (it starts mid-air at the hover
# and ends with the runner sending a staged 'h').
HOME_POSE = (90, 36, 55, 5)  # base, shoulder, elbow, claw — matches arm_mega/main.cpp

# Fallback poses for a section the user doesn't record this run (the recorder
# always writes a COMPLETE keyframes.h, so the un-recorded sequence falls back
# to these placeholders rather than vanishing). Mirror the checked-in
# keyframes.h. Format: (name, (base, shoulder, elbow, claw), keep_base).
PICKUP_DEFAULTS = [
    ("APPROACH",     (142, 30, 113, 21), False),
    ("GRASP_DOWN",   (142, 30, 113, 21), False),
    ("GRASP_CLOSE",  (142, 30, 113, 21), False),
    ("LIFT",         (142, 30, 113, 21), False),
    ("DROP_OVER",    (142, 30, 113, 21), False),
    ("DROP_RELEASE", (142, 30, 113, 21), False),
]
GRASP_DEFAULTS = [
    ("GD_DESCEND", (BASE_KEEP, 110,  40,  5), True),
    ("GD_CLOSE",   (BASE_KEEP, 110,  40, 90), True),
    ("GD_LIFT",    (BASE_KEEP,  60, 170, 90), True),
    ("GD_SWING",   (45,         60, 170, 90), False),
    ("GD_DROPDN",  (45,        110,  40, 90), False),
    ("GD_RELEASE", (45,        110,  40,  5), False),
    ("GD_LIFTOUT", (45,         60, 170,  5), False),
]

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


def capture_all(reader: PosReader, keyframes: list) -> list[tuple] | None:
    """Walk a keyframe spec list. Returns None if the user quit.

    Each spec is (name, hint) or (name, hint, keep_base). Returns a list of
    (name, pose, keep_base) — keep_base poses have their base field replaced
    with BASE_KEEP (the firmware holds the servoed base for those)."""
    captured: list[tuple] = []
    i = 0
    while i < len(keyframes):
        name, hint = keyframes[i][0], keyframes[i][1]
        keep_base = keyframes[i][2] if len(keyframes[i]) > 2 else False
        note = "  (base held = BASE_KEEP)" if keep_base else ""
        result = prompt_keyframe(name, hint + note, reader)
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
            captured.append((name, (0, 0, 0, 0), keep_base))
            i += 1
            continue
        pose = result
        if keep_base:                      # base is servoed at runtime — ignore it
            pose = (BASE_KEEP, pose[1], pose[2], pose[3])
        captured.append((name, pose, keep_base))
        i += 1
    return captured


# ---------------------------------------------------------------------------
# Shoulder lift interlock staging (must match arm_mega/main.cpp)
# ---------------------------------------------------------------------------
# The firmware refuses to raise the shoulder while it's in the stall zone
# (shoulder angle > SHOULDER_GATE) unless the elbow is folded back past
# ELBOW_LIFT_SAFE. A single keyframe that lifts the shoulder AND unfolds the
# elbow would re-trip the veto mid-move and hang. So before any such lift we
# inject two frames — FOLD (tuck the elbow, shoulder held down) then UP (raise
# the shoulder with the elbow still tucked) — then the recorded frame does the
# unfold with the shoulder already safe. Same fold/lift/place dance as staged
# home, applied automatically so recorded sequences can't hang.
SHOULDER_GATE   = 80    # = SHOULDER_LIFT_GATE_DEG
ELBOW_LIFT_SAFE = 165   # = ELBOW_LIFT_SAFE_DEG
ELBOW_TUCK      = 175    # what we fold to; >= ELBOW_LIFT_SAFE, just off the stop


def stage_grasp(grasp: list[tuple]) -> list[tuple]:
    """Insert FOLD + tucked-lift frames before every shoulder lift that crosses
    the stall gate with the elbow untucked. Returns a new, interlock-safe list."""
    out: list[tuple] = []
    for name, pose, keep in grasp:
        if out:
            pbase, psh, pel, pclaw = out[-1][1]
            cur_sh = pose[1]
            lifting     = cur_sh < psh                 # lower angle = arm up
            crosses     = psh > SHOULDER_GATE          # starting in the stall zone
            untucked    = pel < ELBOW_LIFT_SAFE
            if lifting and crosses and untucked:
                pkeep = out[-1][2]
                # FOLD: tuck the elbow at the previous shoulder/base, claw unchanged.
                out.append((f"{name}_FOLD",
                            (pbase, psh, ELBOW_TUCK, pclaw), pkeep))
                # UP: raise the shoulder to target with the elbow still tucked,
                # base held where it was (the recorded frame does any base move).
                out.append((f"{name}_UP",
                            (pbase, cur_sh, ELBOW_TUCK, pose[3]), pkeep))
        out.append((name, pose, keep))
    return out


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

// Sentinel: a keyframe whose base == BASE_KEEP leaves the base target
// untouched when applied — the grasp+drop sequence uses it to preserve the
// visually-servoed base through the pickup phase. 255 is outside LIM_BASE.
static constexpr uint8_t BASE_KEEP = 255;

// Home pose — must match LIM_*.initial in main.cpp.
static constexpr Pose KF_HOME {{ {home[0]:3d}, {home[1]:3d}, {home[2]:3d}, {home[3]:3d} }};

// ---- Pickup sequence (legacy 'p': full home -> pick -> drop -> home) ------
{pickup_defs}

static constexpr Pose PICKUP_SEQUENCE[] = {{
    KF_HOME,
{pickup_seq}
    KF_HOME,
}};
static constexpr uint8_t PICKUP_SEQUENCE_LEN =
    sizeof(PICKUP_SEQUENCE) / sizeof(PICKUP_SEQUENCE[0]);

// ---- Grasp + drop sequence (USB 'g': run from the servoed HOVER pose) -----
// Pickup-phase keyframes carry base == BASE_KEEP so the servoed alignment is
// preserved; the drop phase snaps the base to the recorded drop-zone angle.
// KF_GD_LIFT MUST tuck the elbow (>= ELBOW_LIFT_SAFE_DEG) or the shoulder
// lift interlock hangs the sequence — see main.cpp.
{grasp_defs}

static constexpr Pose GRASP_DROP_SEQUENCE[] = {{
{grasp_seq}
}};
static constexpr uint8_t GRASP_DROP_SEQUENCE_LEN =
    sizeof(GRASP_DROP_SEQUENCE) / sizeof(GRASP_DROP_SEQUENCE[0]);
"""


def _pose_def(name: str, pose: tuple, keep_base: bool) -> str:
    """One 'static constexpr Pose KF_<name> { base, sh, el, claw };' line.
    keep_base poses print BASE_KEEP in the base column instead of a number."""
    base = "BASE_KEEP" if keep_base else f"{pose[0]:3d}"
    return (f"static constexpr Pose KF_{name:<11} "
            f"{{ {base:>9}, {pose[1]:3d}, {pose[2]:3d}, {pose[3]:3d} }};")


def render_header(pickup: list[tuple], grasp: list[tuple]) -> str:
    grasp = stage_grasp(grasp)            # inject interlock-safe fold/lift frames
    pickup_defs = "\n".join(_pose_def(n, p, kb) for n, p, kb in pickup)
    pickup_seq  = "\n".join(f"    KF_{n}," for n, _, _ in pickup)
    grasp_defs  = "\n".join(_pose_def(n, p, kb) for n, p, kb in grasp)
    grasp_seq   = "\n".join(f"    KF_{n}," for n, _, _ in grasp)
    return HEADER_TEMPLATE.format(
        timestamp=_dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        home=HOME_POSE,
        pickup_defs=pickup_defs,
        pickup_seq=pickup_seq,
        grasp_defs=grasp_defs,
        grasp_seq=grasp_seq,
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
    ap.add_argument("--grasp-only", action="store_true",
                    help="record only the grasp+drop sequence (pickup uses defaults)")
    ap.add_argument("--pickup-only", action="store_true",
                    help="record only the legacy pickup sequence (grasp uses defaults)")
    ap.add_argument("--grasp", dest="grasp_only", action="store_true",
                    help="alias for --grasp-only")
    args = ap.parse_args()
    if args.grasp_only and args.pickup_only:
        sys.exit("error: --grasp-only and --pickup-only are mutually exclusive")

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

    pickup_caps: list[tuple] = list(PICKUP_DEFAULTS)
    grasp_caps: list[tuple] = list(GRASP_DEFAULTS)
    quit_early = False
    try:
        if not args.grasp_only:
            print("\n== PICKUP sequence (legacy home->pick->drop->home) ==")
            r = capture_all(reader, KEYFRAMES)
            if r is None:
                quit_early = True
            else:
                pickup_caps = r
        if not quit_early and not args.pickup_only:
            print("\n== GRASP+DROP sequence (from the servoed hover) ==")
            print("Servo the arm over the rock first (pickup_runner ALIGNED),")
            print("or jog it to a representative hover — the base you capture in")
            print("the pickup phase is ignored (written as BASE_KEEP).")
            r = capture_all(reader, GRASP_KEYFRAMES)
            if r is None:
                quit_early = True
            else:
                grasp_caps = r
    finally:
        reader.stop()
        ser.close()

    if quit_early:
        print("\nquit — nothing written.")
        return

    header = render_header(pickup_caps, grasp_caps)
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
