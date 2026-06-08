#!/usr/bin/env python3
"""
servo_diagnostic.py
-------------------
Vision-side dry run of the pickup correction loop.  Combines rock detection
(shape + colour signature), ArUco-marker detection (claw-tip estimate) and
the white base disc (rotation-centre estimate) into one live view, and shows
what base rotation the visual-servoing step *would* apply — without moving
the arm.

The correction is pure geometry about the base centre:

    Δbase = atan2(rock − centre) − atan2(claw − centre)

i.e. how far to spin the base so the claw swings onto the rock's bearing.
This replaces the old PX_PER_DEG_BASE placeholder — no per-rig pixel-to-
degree constant is needed once the centre is known.

For each frame:
  - Detects the red rock via the shape+colour matcher and marks its centroid.
  - Detects every ArUco marker (DICT_4X4_50); the claw-tip estimate is the
    mean of all visible marker centroids (per-marker offsets calibrated later).
  - Segments the white base disc and fits an ellipse → rotation centre.
  - Draws radial lines centre→rock and centre→claw, and prints/overlays the
    Δbase that would be commanded.

Caveat: the camera is oblique, so "rotation about the base axis" is only
approximately "rotation about the image centre".  Δbase here is a good
direction and a near-right magnitude — the closed loop (which re-measures
each step) absorbs the rest.  Sign may need flipping for the servo's
polarity; that gets pinned down when the loop is actually wired in.

The V_min / S_max sliders tune the white-disc mask for room lighting.

Controls
--------
    Q  quit
    L  log the current state (centre, rock, claw, Δbase) to the terminal

Run
---
    python3 vision/servo_diagnostic.py
    python3 vision/servo_diagnostic.py --camera 1
"""
import argparse
import time

import cv2
import numpy as np

from colour_detect import (
    COLOURS, MIN_BLOB_AREA, TARGET_COLOUR, TARGET_REFS,
    MATCH_Z_TOL, MATCH_MIN_FEATS, MATCH_MAX_TOTAL,
    _make_mask, _clean_mask,
)
from shape_match import ShapeMatcher
from base_outline import largest_disc

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Marker roles: 0-3 are stuck on the arm (elbow spine) and estimate the claw
# bearing; 10 is the base-heading marker stuck on the BASE and must NOT be
# averaged into the claw estimate (it sits on the opposite side of the axis
# and is almost always visible, so it wrecks the claw position if included).
CLAW_IDS = {0, 1, 2, 3}
BASE_ID  = 10

# |Δbase| below this is considered aligned — no nudge needed before descent.
# In degrees of base rotation. Tuned later against the real claw tolerance.
ALIGN_DEG = 3.0

CAM_W, CAM_H = 640, 480

WIN = "Servo diagnostic (q quit, l log)"


def find_rock(frame_hsv: np.ndarray, matcher: ShapeMatcher) -> tuple[int, int] | None:
    """Return rock centroid in pixels, or None if not detected this frame."""
    red = next(c for c in COLOURS if c["name"] == TARGET_COLOUR)
    mask  = _make_mask(frame_hsv, red)
    clean = _clean_mask(mask)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None              # (total_z, centroid)
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_BLOB_AREA:
            continue
        is_target, _n_in, total_z, _ = matcher.score(cnt, frame_hsv)
        if not is_target:
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        if best is None or total_z < best[0]:
            best = (total_z, (cx, cy))
    return best[1] if best else None


def norm180(deg: float) -> float:
    """Wrap an angle to (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def bearing_deg(centre: tuple[int, int], pt: tuple[int, int]) -> float:
    """Image-frame bearing of pt as seen from centre, in degrees."""
    return np.degrees(np.arctan2(pt[1] - centre[1], pt[0] - centre[0]))


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    args = p.parse_args()

    matcher = ShapeMatcher(
        TARGET_REFS,
        z_tol=MATCH_Z_TOL,
        min_features=MATCH_MIN_FEATS,
        max_total_z=MATCH_MAX_TOTAL,
    )

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{args.camera}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    cv2.namedWindow(WIN)
    cv2.createTrackbar("V_min", WIN, 150, 255, lambda v: None)
    cv2.createTrackbar("S_max", WIN, 60,  255, lambda v: None)

    print("[servo_diag] running — q to quit, l to log state")
    prev_t = time.monotonic()

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        now = time.monotonic()
        fps = 1.0 / max(now - prev_t, 1e-9)
        prev_t = now

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ---- base disc -> rotation centre ----
        v_min = cv2.getTrackbarPos("V_min", WIN)
        s_max = cv2.getTrackbarPos("S_max", WIN)
        disc_mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))
        ellipse, _cnt, _clean = largest_disc(disc_mask)
        centre = None
        if ellipse is not None:
            cv2.ellipse(frame, ellipse, (0, 200, 0), 1)
            centre = tuple(map(int, ellipse[0]))
            cv2.drawMarker(frame, centre, (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            cv2.putText(frame, "BASE CENTRE", (centre[0] + 10, centre[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        # ---- rock ----
        rock = find_rock(hsv, matcher)
        if rock is not None:
            cv2.circle(frame, rock, 8, (0, 255, 0), 2)
            cv2.circle(frame, rock, 2, (0, 255, 0), -1)
            cv2.putText(frame, "ROCK", (rock[0] + 12, rock[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

        # ---- markers -> claw-tip estimate ----
        corners, ids, _ = detector.detectMarkers(gray)
        marker_centroids: list[tuple[int, int]] = []   # CLAW markers only (0-3)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_corners, mid in zip(corners, ids.flatten()):
                if int(mid) not in CLAW_IDS:          # skip base marker (10), etc.
                    continue
                pts = marker_corners.reshape(-1, 2)
                cx, cy = pts.mean(axis=0)
                marker_centroids.append((int(cx), int(cy)))

        claw = None
        if marker_centroids:
            xs = [c[0] for c in marker_centroids]
            ys = [c[1] for c in marker_centroids]
            claw = (int(sum(xs) / len(xs)), int(sum(ys) / len(ys)))
            cv2.drawMarker(frame, claw, (0, 255, 255),
                           cv2.MARKER_CROSS, markerSize=22, thickness=2)
            cv2.putText(frame, f"CLAW (n={len(marker_centroids)})",
                        (claw[0] + 12, claw[1] + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # ---- centre-based Δbase correction ----
        corr_text = "base corr: -"
        status    = "need centre + rock + marker"
        d_base    = None
        if centre is not None and rock is not None and claw is not None:
            cv2.line(frame, centre, rock, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.line(frame, centre, claw, (0, 255, 255), 1, cv2.LINE_AA)
            d_base = norm180(bearing_deg(centre, rock) - bearing_deg(centre, claw))
            corr_text = f"Δbase ~ {d_base:+.1f} deg  (rotate base to swing claw onto rock)"
            status = "ALIGNED — would descend" if abs(d_base) < ALIGN_DEG else "would rotate base"

        # ---- HUD ----
        hud_lines = [
            f"{fps:4.1f} fps",
            f"centre: {'yes' if centre else 'no'}   rock: {'yes' if rock else 'no'}"
            f"   claw: {'yes' if claw else 'no'} (n={len(marker_centroids)})",
            corr_text,
            f"status: {status}",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(frame, line, (8, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (240, 240, 240), 1, cv2.LINE_AA)

        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("l"):
            print(f"[log] centre={centre} rock={rock} claw={claw} "
                  f"markers={len(marker_centroids)}")
            if d_base is not None:
                print(f"      {corr_text}  |  {status}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
