#!/usr/bin/env python3
"""
EE6003 robotic arm — orchestrated flash of both PlatformIO environments.

Builds arm_mega + the controller env first (catches compile errors before
disturbing any hardware), then uploads them in order, pausing in between
so you can disconnect/reconnect the HC-05 TXD jumper on the Uno R4.

By default this flashes the v2 controller firmware (controller_uno_r4_v2)
— joysticks for shoulder/elbow, SW hold for base CCW/CW, pot on A4 for
the claw, 'h' over USB Serial for home. The original 4-axis joystick
firmware (controller_uno_r4) is still built and uploadable via --only.

Usage (from anywhere — script auto-locates platformio.ini):

    python scripts/flash_all.py
    python scripts/flash_all.py --skip-build
    python scripts/flash_all.py --only arm_mega
    python scripts/flash_all.py --only controller_uno_r4_v2
    python scripts/flash_all.py --only controller_uno_r4      # original firmware

Requires the `pio` CLI on PATH (PlatformIO Core or the VS Code extension).
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ENV_ARM    = "arm_mega"
ENV_CTL    = "controller_uno_r4_v2"   # default controller env flashed by this script
ENV_CTL_V1 = "controller_uno_r4"      # original firmware, available via --only


def find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for cand in (here, *here.parents):
        if (cand / "platformio.ini").is_file():
            return cand
    sys.exit("error: could not find platformio.ini above this script")


def require_pio() -> str:
    pio = shutil.which("pio") or shutil.which("platformio")
    if not pio:
        sys.exit(
            "error: 'pio' not found on PATH.\n"
            "Install PlatformIO Core: https://docs.platformio.org/en/latest/core/installation/index.html"
        )
    return pio


def run(cmd: list[str], cwd: Path) -> None:
    print(f"\n$ {' '.join(cmd)}\n", flush=True)
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        sys.exit(f"command failed: {' '.join(cmd)}")


def confirm(prompt: str) -> None:
    try:
        input(f"\n>>> {prompt}\n    Press Enter to continue (Ctrl+C to abort) ... ")
    except (EOFError, KeyboardInterrupt):
        sys.exit("\naborted by user")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-build", action="store_true", help="skip the up-front 'pio run' build step")
    ap.add_argument(
        "--only",
        choices=[ENV_ARM, ENV_CTL, ENV_CTL_V1],
        help="upload only one specific environment",
    )
    args = ap.parse_args()

    root = find_project_root()
    pio = require_pio()
    print(f"project root: {root}")

    if not args.skip_build:
        # Building all envs first means we don't half-flash the arm only to
        # find the controller doesn't compile.
        run([pio, "run"], cwd=root)

    targets = (
        [ENV_ARM, ENV_CTL] if args.only is None
        else [args.only]
    )

    # Any controller-side env triggers the HC-05 disconnect prompt — both
    # variants share the same Uno R4 board with D0 tied to the HC-05's TXD.
    controller_envs = {ENV_CTL, ENV_CTL_V1}

    for env in targets:
        if env in controller_envs:
            confirm(
                f"About to upload {env} to the Uno R4 Minima.\n"
                "    DISCONNECT the HC-05 TXD wire from D0 first, otherwise\n"
                "    the bootloader will refuse the upload."
            )
        run([pio, "run", "-e", env, "-t", "upload"], cwd=root)
        if env in controller_envs:
            print("\n>>> Upload complete. Reconnect HC-05 TXD on D0.")

    print("\nAll done.")


if __name__ == "__main__":
    main()
