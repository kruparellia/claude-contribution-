#!/usr/bin/env python3
"""
EE6003 robotic arm — orchestrated flash of both PlatformIO environments.

Builds arm_mega + controller_uno_r4 first (catches compile errors before
disturbing any hardware), then uploads them in order. If a board isn't
plugged in, that env is skipped (not fatal) so the other board still
gets flashed. Per-env failures are collected and reported at the end
with the exact pio return code, so nothing fails silently.

Usage (from anywhere — script auto-locates platformio.ini):

    python scripts/flash_all.py
    python scripts/flash_all.py --skip-build
    python scripts/flash_all.py --only arm_mega
    python scripts/flash_all.py --only controller_uno_r4
    python scripts/flash_all.py --no-prompt                   # skip HC-05 prompt
    python scripts/flash_all.py --force                       # try upload even if device not detected

Exit codes:
    0  all attempted uploads succeeded (skipped-missing-device counts as success)
    1  one or more uploads failed, or no devices were detected at all
    2  precondition failure (no pio, no platformio.ini, build failed)

Requires the `pio` CLI on PATH (PlatformIO Core or the VS Code extension).
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

ENV_ARM = "arm_mega"
ENV_CTL = "controller_uno_r4"

# USB VID:PID fingerprints that identify each board. Strings are matched
# case-insensitively as substrings against the `hwid` field returned by
# `pio device list --json-output`. Covers the common official + clone
# variants we've actually seen on this project.
DEVICE_FINGERPRINTS: dict[str, list[str]] = {
    ENV_ARM: [
        "2341:0042",  # Arduino Mega 2560 R3 (official)
        "2341:0010",  # Arduino Mega 2560 R1/R2
        "2A03:0042",  # Arduino.org Mega 2560
        "1A86:7523",  # CH340 clone (very common Elegoo Mega)
        "1A86:55D4",  # CH9102 clone
        "0403:6001",  # FTDI FT232 clone
    ],
    ENV_CTL: [
        "2341:0069",  # Uno R4 Minima
        "2341:1002",  # Uno R4 WiFi (just in case)
        "2341:0369",  # Uno R4 Minima DFU mode
    ],
}

BOARD_LABEL: dict[str, str] = {
    ENV_ARM: "Elegoo/Arduino Mega 2560",
    ENV_CTL: "Arduino Uno R4 Minima (controller)",
}


def find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for cand in (here, *here.parents):
        if (cand / "platformio.ini").is_file():
            return cand
    print("error: could not find platformio.ini above this script", file=sys.stderr)
    sys.exit(2)


def require_pio() -> str:
    pio = shutil.which("pio") or shutil.which("platformio")
    if not pio:
        print(
            "error: 'pio' not found on PATH.\n"
            "Install PlatformIO Core: https://docs.platformio.org/en/latest/core/installation/index.html",
            file=sys.stderr,
        )
        sys.exit(2)
    return pio


def list_devices(pio: str) -> list[dict]:
    """Return the parsed output of `pio device list --json-output`, or []."""
    try:
        proc = subprocess.run(
            [pio, "device", "list", "--json-output"],
            capture_output=True, text=True, timeout=15,
        )
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"warning: could not enumerate serial devices ({e}); proceeding blind", file=sys.stderr)
        return []
    if proc.returncode != 0:
        print(f"warning: 'pio device list' exited {proc.returncode}; proceeding blind", file=sys.stderr)
        if proc.stderr.strip():
            print(proc.stderr, file=sys.stderr)
        return []
    try:
        data = json.loads(proc.stdout or "[]")
    except json.JSONDecodeError as e:
        print(f"warning: could not parse 'pio device list' JSON ({e}); proceeding blind", file=sys.stderr)
        return []
    return data if isinstance(data, list) else []


def find_port_for_env(env: str, devices: list[dict]) -> str | None:
    fps = DEVICE_FINGERPRINTS.get(env, [])
    for dev in devices:
        hwid = (dev.get("hwid") or "").upper()
        desc = (dev.get("description") or "").upper()
        haystack = f"{hwid} {desc}"
        for fp in fps:
            if fp.upper() in haystack:
                return dev.get("port")
    return None


def run_capture(cmd: list[str], cwd: Path) -> int:
    """Run cmd, streaming output live, returning the exit code (never raises)."""
    print(f"\n$ {' '.join(cmd)}\n", flush=True)
    try:
        return subprocess.run(cmd, cwd=cwd).returncode
    except FileNotFoundError:
        print(f"error: executable not found: {cmd[0]}", file=sys.stderr)
        return 127
    except KeyboardInterrupt:
        print("\naborted by user", file=sys.stderr)
        return 130


def confirm(prompt: str, *, auto: bool) -> bool:
    if auto:
        print(f"\n>>> {prompt}\n    (--no-prompt set, continuing without confirmation)")
        return True
    try:
        input(f"\n>>> {prompt}\n    Press Enter to continue (Ctrl+C to abort) ... ")
        return True
    except (EOFError, KeyboardInterrupt):
        print("\nskipped by user", file=sys.stderr)
        return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--skip-build", action="store_true", help="skip the up-front 'pio run' build step")
    ap.add_argument(
        "--only",
        choices=[ENV_ARM, ENV_CTL],
        help="upload only one specific environment",
    )
    ap.add_argument("--no-prompt", action="store_true", help="don't pause for HC-05 disconnect confirmation")
    ap.add_argument("--force", action="store_true", help="try uploading even when no matching device is detected")
    args = ap.parse_args()

    root = find_project_root()
    pio = require_pio()
    print(f"project root: {root}")

    targets = [ENV_ARM, ENV_CTL] if args.only is None else [args.only]
    controller_envs = {ENV_CTL}

    if not args.skip_build:
        # Build every upload target up-front (one `pio run` invocation, multiple
        # -e flags) so a controller compile error doesn't surface only after the
        # mega has already been flashed. Bare `pio run` would respect
        # default_envs in platformio.ini and silently skip the controller.
        build_cmd = [pio, "run"]
        for env in targets:
            build_cmd += ["-e", env]
        rc = run_capture(build_cmd, cwd=root)
        if rc != 0:
            print(f"\nerror: build failed with exit code {rc} — aborting before any upload.", file=sys.stderr)
            return 2

    devices = list_devices(pio)
    if devices:
        print("\nDetected serial devices:")
        for d in devices:
            print(f"  - {d.get('port')}   {d.get('description', '')}   [{d.get('hwid', '')}]")
    else:
        print("\nNo serial devices reported by 'pio device list'.")

    results: list[tuple[str, str, int]] = []  # (env, status, returncode)

    for env in targets:
        label = BOARD_LABEL.get(env, env)
        port = find_port_for_env(env, devices)

        if port is None and not args.force:
            print(f"\n--- {env} ({label}): SKIPPED — board not detected on any serial port.")
            print("    Plug it in and re-run, or pass --force to attempt upload anyway.")
            results.append((env, "skipped (no device)", 0))
            continue

        if port:
            print(f"\n--- {env} ({label}): found on {port}")
        else:
            print(f"\n--- {env} ({label}): not auto-detected, --force given, letting pio pick a port")

        if env in controller_envs:
            ok = confirm(
                f"About to upload {env} to the Uno R4 Minima.\n"
                "    DISCONNECT the HC-05 TXD wire from D0 first, otherwise\n"
                "    the bootloader will refuse the upload.",
                auto=args.no_prompt,
            )
            if not ok:
                results.append((env, "skipped (user)", 0))
                continue

        cmd = [pio, "run", "-e", env, "-t", "upload"]
        if port:
            cmd += ["--upload-port", port]
        rc = run_capture(cmd, cwd=root)

        if rc == 0:
            print(f"\n>>> {env}: upload OK")
            if env in controller_envs:
                print(">>> Reconnect HC-05 TXD on D0.")
            results.append((env, "ok", 0))
        else:
            print(f"\nerror: {env} upload failed (pio exit code {rc})", file=sys.stderr)
            results.append((env, "FAILED", rc))

    print("\n========== Flash summary ==========")
    any_failed = False
    any_attempted = False
    for env, status, rc in results:
        line = f"  {env:<28} {status}"
        if rc:
            line += f"  (exit {rc})"
            any_failed = True
        print(line)
        if status == "ok" or status.startswith("FAILED"):
            any_attempted = True

    if any_failed:
        return 1
    if not any_attempted:
        # Nothing actually uploaded — usually means no boards were plugged in.
        print("\nNote: no uploads were performed. Plug in a board (or pass --force) and try again.")
        return 1
    print("\nAll attempted uploads succeeded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
