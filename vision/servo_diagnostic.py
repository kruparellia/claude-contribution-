#!/usr/bin/env python3
"""
servo_diagnostic.py
-------------------
Vision-side dry run of the pickup correction loop.  Combines rock detection
(shape + colour signature) and ArUco-marker detection (claw-tip estimate)
into one live view, and shows what correction the visual-servoing step
*would* apply — without moving the arm.

For each frame:
  - Detects the red rock via the shape+colour matcher and marks its centroid.
  - Detects every ArUco marker in view (DICT_4X4_50).  The claw-tip estimate
    is the simple average of all visible marker centroids — good enough for
    a dry run; per-marker offsets get calibrated later.
  - Draws an arrow from the claw-tip estimate to the rock centroid.
  - Prints (and overlays) the pixel delta and a placeholder base-angle
    correction using a rough px-per-degree constant.  That constant will be
    measured empirically once the arm is wired in.

Controls
--------
    Q  quit
    L  log the current state (rock, claw, delta) to the terminal

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

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Placeholder: how many image pixels equal one degree of base rotation at
# the pickup distance. Wildly approximate — we'll measure this empirically
# by jogging the arm a known amount and counting pixel travel. Until then,
# treat the "deg" readout as illustrative, not actionable.
PX_PER_DEG_BASE_PLACEHOLDER = 6.0

# Magnitude below which the correction is considered "good enough" — no
# nudge needed before descent. Tuned later against the actual claw tip
# tolerance.
ALIGN_PX = 12

CAM_W, CAM_H = 640, 480


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

        # ---- rock ----
        rock = find_rock(hsv, matcher)
        if rock is not None:
            cv2.circle(frame, rock, 8, (0, 255, 0), 2)
            cv2.circle(frame, rock, 2, (0, 255, 0), -1)
            cv2.putText(frame, "ROCK", (rock[0] + 12, rock[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

        # ---- markers ----
        corners, ids, _ = detector.detectMarkers(gray)
        marker_centroids: list[tuple[int, int]] = []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_corners in corners:
                pts = marker_corners.reshape(-1, 2)
                cx, cy = pts.mean(axis=0)
                marker_centroids.append((int(cx), int(cy)))

        # ---- claw-tip estimate = mean of visible marker centroids ----
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

        # ---- delta + would-be correction ----
        delta_text = "delta: -"
        corr_text  = "base corr: -"
        status     = "waiting for rock + marker"
        if rock is not None and claw is not None:
            dx = rock[0] - claw[0]
            dy = rock[1] - claw[1]
            mag = (dx * dx + dy * dy) ** 0.5
            cv2.arrowedLine(frame, claw, rock, (255, 200, 0), 2, tipLength=0.15)
            delta_text = f"delta: dx={dx:+d} dy={dy:+d} |={mag:.0f}px"
            base_corr = dx / PX_PER_DEG_BASE_PLACEHOLDER
            corr_text = f"base corr ~ {base_corr:+.1f} deg  (px/deg={PX_PER_DEG_BASE_PLACEHOLDER:.1f})"
            status = "ALIGNED — would descend" if mag < ALIGN_PX else "would nudge base"

        # ---- HUD ----
        hud_lines = [
            f"{fps:4.1f} fps",
            f"markers visible: {len(marker_centroids)}",
            f"rock: {'yes' if rock else 'no'}   claw: {'yes' if claw else 'no'}",
            delta_text,
            corr_text,
            f"status: {status}",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(frame, line, (8, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (240, 240, 240), 1, cv2.LINE_AA)

        cv2.imshow("Servo diagnostic (q quit, l log)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("l"):
            print(f"[log] rock={rock}  claw={claw}  markers_visible={len(marker_centroids)}")
            if rock and claw:
                print(f"      {delta_text}  |  {corr_text}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
