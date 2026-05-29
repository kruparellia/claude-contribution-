#!/usr/bin/env python3
"""
pickup_runner.py
----------------
End-to-end MVP: detect the rock inside the pickup zone, wait for the
operator to authorise, trigger the arm's pickup sequence over USB serial.

No visual servoing yet — this script proves the full chain (vision ->
permission -> trigger -> arm motion) using the existing keyframe runner
on the Mega.  Visual servoing layers on top later by computing a base-
angle nudge before sending `p`.

State machine
-------------
    IDLE        no rock in pickup zone yet
    LOCKING     rock seen in zone — counting consecutive frames
    READY       rock locked in — waiting for SPACE to authorise
    PICKING     `p` sent — draining arm log lines until COOLDOWN_S elapsed
    COOLDOWN    short pause so the same rock doesn't immediately re-trigger

Controls
--------
    SPACE   authorise pickup (only when state == READY)
    H       send `h` (home) to the arm
    A       toggle arm auto-mode
    X       send `x` (abort) to the arm
    L       log the current diagnostic state to the terminal
    Q       quit

Run
---
    python3 vision/pickup_runner.py
    python3 vision/pickup_runner.py --camera 1 --dry-run    # no serial open
"""
import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np

from arm_link import ArmLink
from colour_detect import (
    COLOURS, MIN_BLOB_AREA, TARGET_COLOUR, TARGET_REFS,
    MATCH_Z_TOL, MATCH_MIN_FEATS, MATCH_MAX_TOTAL,
    _make_mask, _clean_mask,
)
from shape_match import ShapeMatcher

# ---------------------------------------------------------------------------
# Rig-specific constants — RE-TUNE FOR THE FINAL SETUP
# ---------------------------------------------------------------------------
CAM_W, CAM_H = 640, 480

# Pickup zone in pixel coords (x, y, w, h). Mark the black-tape area on the
# bench, then run the script once and read the rock centroid coords to
# pick these numbers. Drawn in white on the HUD.
PICKUP_ZONE = (200, 180, 240, 180)

# Drop zone in pixel coords — display-only for now. Once visual servoing
# is in, this is also where a drop-side check could happen.
DROP_ZONE   = ( 80, 380, 120,  80)

# How many consecutive frames the rock must be inside the pickup zone
# before we promote to READY. ~10 frames at 30 fps = 0.33 s — long enough
# to ride out a single dropped detection, short enough to feel responsive.
LOCK_FRAMES = 10

# Max time PICKING is held before returning to COOLDOWN. The full sequence
# (KF_HOME -> APPROACH -> GRASP_DOWN -> GRASP_CLOSE -> LIFT -> DROP_OVER
# -> DROP_RELEASE -> KF_HOME) takes ~6-8 s in practice; 12 s leaves margin.
PICKUP_TIMEOUT_S = 12.0

# Pause after PICKUP_TIMEOUT_S before we'll accept the next trigger. Keeps
# us from immediately re-firing on the same rock the moment it's set back
# down. 3 s is enough for a human to clear / replace the workspace.
COOLDOWN_S = 3.0

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
@dataclass
class RunnerState:
    name: str = "IDLE"
    lock_count: int = 0
    state_started: float = 0.0
    last_arm_line: str = ""

def in_zone(pt: tuple[int, int], zone: tuple[int, int, int, int]) -> bool:
    x, y, w, h = zone
    return x <= pt[0] <= x + w and y <= pt[1] <= y + h

def find_rock(hsv: np.ndarray, matcher: ShapeMatcher) -> tuple[int, int] | None:
    """Return rock centroid in pixels, or None if not detected this frame."""
    red = next(c for c in COLOURS if c["name"] == TARGET_COLOUR)
    mask  = _make_mask(hsv, red)
    clean = _clean_mask(mask)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_BLOB_AREA:
            continue
        is_target, _n, total_z, _ = matcher.score(cnt, hsv)
        if not is_target:
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        cx = int(M["m10"] / M["m00"]); cy = int(M["m01"] / M["m00"])
        if best is None or total_z < best[0]:
            best = (total_z, (cx, cy))
    return best[1] if best else None

def drain_serial(arm: ArmLink | None) -> list[str]:
    """Read whatever the arm has emitted since the last call, non-blocking."""
    if arm is None or arm._ser is None or arm._ser.in_waiting == 0:
        return []
    data = arm._ser.read(arm._ser.in_waiting)
    return [
        line.rstrip("\r")
        for line in data.decode(errors="replace").split("\n")
        if line.strip()
    ]

def transition(state: RunnerState, new_name: str) -> None:
    state.name = new_name
    state.state_started = time.monotonic()
    state.lock_count = 0

# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------
def draw_zone(frame: np.ndarray, zone: tuple, label: str, colour: tuple) -> None:
    x, y, w, h = zone
    cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 1)
    cv2.putText(frame, label, (x + 4, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

STATE_COLOURS = {
    "IDLE":     (160, 160, 160),
    "LOCKING":  (  0, 200, 255),
    "READY":    (  0, 255,   0),
    "PICKING":  (255, 180,   0),
    "COOLDOWN": (130, 130, 200),
}

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--dry-run", action="store_true",
                   help="Skip the arm serial connection — vision-only test.")
    args = p.parse_args()

    matcher = ShapeMatcher(TARGET_REFS, z_tol=MATCH_Z_TOL,
                           min_features=MATCH_MIN_FEATS,
                           max_total_z=MATCH_MAX_TOTAL)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{args.camera}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    arm: ArmLink | None = None
    if not args.dry_run:
        arm = ArmLink().__enter__()
        print(f"[pickup_runner] connected to arm on {arm.port}")
        try:
            arm.set_auto(True)
            print("[pickup_runner] auto mode ON")
        except RuntimeError as e:
            print(f"[pickup_runner] WARN: {e} — arm may not respond")
    else:
        print("[pickup_runner] DRY RUN — no serial connection")

    state = RunnerState(state_started=time.monotonic())
    print("[pickup_runner] running — SPACE=go  H=home  A=auto  X=abort  L=log  Q=quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05); continue

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Vision
            rock = find_rock(hsv, matcher)
            corners, ids, _ = detector.detectMarkers(gray)
            marker_centroids: list[tuple[int, int]] = []
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for c in corners:
                    pts = c.reshape(-1, 2)
                    marker_centroids.append((int(pts[:, 0].mean()), int(pts[:, 1].mean())))
            claw = None
            if marker_centroids:
                claw = (int(np.mean([c[0] for c in marker_centroids])),
                        int(np.mean([c[1] for c in marker_centroids])))

            rock_in_pickup = rock is not None and in_zone(rock, PICKUP_ZONE)

            # ---- state machine ----
            now_t = time.monotonic()
            if state.name == "IDLE":
                if rock_in_pickup:
                    transition(state, "LOCKING"); state.lock_count = 1
            elif state.name == "LOCKING":
                if rock_in_pickup:
                    state.lock_count += 1
                    if state.lock_count >= LOCK_FRAMES:
                        transition(state, "READY")
                else:
                    transition(state, "IDLE")
            elif state.name == "READY":
                if not rock_in_pickup:
                    transition(state, "IDLE")
            elif state.name == "PICKING":
                if now_t - state.state_started > PICKUP_TIMEOUT_S:
                    transition(state, "COOLDOWN")
            elif state.name == "COOLDOWN":
                if now_t - state.state_started > COOLDOWN_S:
                    transition(state, "IDLE")

            # Pull anything the arm has logged since the last frame.
            for line in drain_serial(arm):
                state.last_arm_line = line
                print(f"  [arm] {line}")

            # ---- drawing ----
            draw_zone(frame, PICKUP_ZONE, "PICKUP", (255, 255, 255))
            draw_zone(frame, DROP_ZONE,   "DROP",   (60, 60, 220))

            if rock is not None:
                rcol = (0, 255, 0) if rock_in_pickup else (0, 200, 255)
                cv2.circle(frame, rock, 8, rcol, 2)
                cv2.circle(frame, rock, 2, rcol, -1)
                cv2.putText(frame, "ROCK", (rock[0] + 12, rock[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, rcol, 1, cv2.LINE_AA)
            if claw is not None:
                cv2.drawMarker(frame, claw, (0, 255, 255),
                               cv2.MARKER_CROSS, markerSize=22, thickness=2)

            scolour = STATE_COLOURS.get(state.name, (200, 200, 200))
            extra = ""
            if state.name == "LOCKING":
                extra = f" ({state.lock_count}/{LOCK_FRAMES})"
            elif state.name == "READY":
                extra = "  press SPACE to pick"
            elif state.name == "PICKING":
                extra = f"  ({now_t - state.state_started:4.1f}s)"
            elif state.name == "COOLDOWN":
                left = COOLDOWN_S - (now_t - state.state_started)
                extra = f"  ({left:3.1f}s)"

            cv2.rectangle(frame, (0, 0), (CAM_W, 30), (35, 35, 35), -1)
            cv2.putText(frame, f"STATE: {state.name}{extra}",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        scolour, 2, cv2.LINE_AA)

            if state.last_arm_line:
                cv2.putText(frame, f"arm: {state.last_arm_line[:60]}",
                            (10, CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (200, 200, 200), 1, cv2.LINE_AA)

            cv2.imshow("Pickup runner (q quit)", frame)

            # ---- input handling ----
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and state.name == "READY":
                if arm is not None:
                    arm.pickup()
                    print("[pickup_runner] -> pickup triggered")
                else:
                    print("[pickup_runner] (dry run) would trigger pickup")
                transition(state, "PICKING")
            elif key == ord("h") and arm is not None:
                arm.home(); print("[pickup_runner] -> home")
            elif key == ord("a") and arm is not None:
                arm.toggle_auto(); print("[pickup_runner] -> toggle auto")
            elif key == ord("x") and arm is not None:
                arm.abort(); print("[pickup_runner] -> abort")
                transition(state, "COOLDOWN")
            elif key == ord("l"):
                print(f"[log] state={state.name} lock={state.lock_count} "
                      f"rock={rock} claw={claw} in_zone={rock_in_pickup}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if arm is not None:
            arm.__exit__(None, None, None)

if __name__ == "__main__":
    main()
