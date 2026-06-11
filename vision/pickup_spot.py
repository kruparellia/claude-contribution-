#!/usr/bin/env python3
"""
pickup_spot.py
--------------
Detect the black-tape X that marks the real pickup spot, and report when the
rock is sitting on it.  Diagnostic / tuning tool — once it locks the X
reliably this logic drops into pickup_runner.py in place of the old fixed
PICKUP_ZONE rectangle.

How it works
------------
The X is a fixed black-tape mark on the table.  We segment dark pixels
(low V in HSV), take the best X-sized blob as the spot, and CACHE its pixel
location — the camera and tape don't move during a run.

Caching is the important bit: once the rock is placed it OCCLUDES the X, so
we can't see the mark at trigger time.  So we lock the spot from the clean
scene (before the rock is on it), then just test whether the rock centroid is
within SPOT_RADIUS_FACTOR * (mark size) of the locked spot.

Tuning
------
    V_max     max brightness counted as "tape" (raise if the X is missed,
              lower if shadows/arm get picked up)
    MinArea   smallest dark blob (px) accepted as the X
    MaxArea   largest dark blob accepted  (rejects big shadow patches)

Controls
--------
    K   lock / unlock the spot (auto-locks after LOCK_FRAMES stable frames)
    L   log the current spot + rock state
    Q   quit

Run
---
    python3 vision/pickup_spot.py
    python3 vision/pickup_spot.py --camera 1
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

CAM_W, CAM_H = 640, 480
WIN = "Pickup spot (q quit, k lock)"

# Bounding-box aspect ratio the X must fall within (it sits in a roughly
# square box). Rejects long thin dark streaks like wires / cable shadows.
ASPECT_LO, ASPECT_HI = 0.5, 2.0

# Spot radius = this fraction of the X's bounding-box half-diagonal. The rock
# counts as "on the spot" inside this radius. ~1.0 = the mark's own footprint.
SPOT_RADIUS_FACTOR = 1.1

# Frames the spot must be seen consistently before it auto-locks.
LOCK_FRAMES = 12

# How far (px) two detections can be apart and still count as "the same spot"
# for the stability counter.
SPOT_STABLE_PX = 25


def find_rock(hsv, matcher):
    """Return rock centroid in pixels, or None if not detected this frame."""
    red = next(c for c in COLOURS if c["name"] == TARGET_COLOUR)
    clean = _clean_mask(_make_mask(hsv, red))
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
        c = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if best is None or total_z < best[0]:
            best = (total_z, c)
    return best[1] if best else None


def find_dark_x(hsv, v_max, min_area, max_area):
    """Best X-sized dark blob as (centre, radius_px), or (None, None).

    Segments low-V pixels, closes the strokes into one blob, and picks the
    largest contour inside the area + aspect gates.
    """
    mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, v_max))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((9, 9), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None                       # (area, centre, radius)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if h == 0:
            continue
        aspect = w / h
        if not (ASPECT_LO <= aspect <= ASPECT_HI):
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        radius = int(SPOT_RADIUS_FACTOR * 0.5 * np.hypot(w, h))
        if best is None or area > best[0]:
            best = (area, centre, radius)
    if best is None:
        return None, None, mask
    return best[1], best[2], mask


def dist(a, b):
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
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

    cv2.namedWindow(WIN)
    cv2.createTrackbar("V_max",   WIN, 60,   255,   lambda v: None)
    cv2.createTrackbar("MinArea", WIN, 300,  5000,  lambda v: None)
    cv2.createTrackbar("MaxArea", WIN, 8000, 40000, lambda v: None)

    spot = None            # locked/cached spot centre (px)
    spot_r = 0             # cached spot radius (px)
    stable_count = 0
    locked = False
    print("[pickup_spot] running — k lock/unlock, l log, q quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05); continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        v_max    = cv2.getTrackbarPos("V_max", WIN)
        min_area = cv2.getTrackbarPos("MinArea", WIN)
        max_area = max(cv2.getTrackbarPos("MaxArea", WIN), min_area + 1)

        det, det_r, mask = find_dark_x(hsv, v_max, min_area, max_area)
        rock = find_rock(hsv, matcher)

        # ---- maintain the cached spot ----
        if not locked and det is not None:
            if spot is not None and dist(det, spot) < SPOT_STABLE_PX:
                stable_count += 1
            else:
                stable_count = 0
            spot, spot_r = det, det_r
            if stable_count >= LOCK_FRAMES:
                locked = True
                print(f"[pickup_spot] LOCKED spot={spot} r={spot_r}")

        on_spot = (spot is not None and rock is not None
                   and dist(rock, spot) <= spot_r)

        # ---- draw ----
        # dim dark-mask preview in the corner
        thumb = cv2.resize(mask, (160, 120))
        frame[0:120, CAM_W - 160:CAM_W] = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)

        if det is not None and not locked:
            cv2.circle(frame, det, det_r, (0, 180, 255), 1)
        if spot is not None:
            scol = (0, 255, 0) if locked else (0, 180, 255)
            cv2.circle(frame, spot, spot_r, scol, 2)
            cv2.drawMarker(frame, spot, scol, cv2.MARKER_TILTED_CROSS, 18, 2)
            cv2.putText(frame, "SPOT" + (" (locked)" if locked else ""),
                        (spot[0] + spot_r + 4, spot[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, scol, 1, cv2.LINE_AA)
        if rock is not None:
            rcol = (0, 255, 0) if on_spot else (0, 200, 255)
            cv2.circle(frame, rock, 8, rcol, 2)
            cv2.putText(frame, "ROCK", (rock[0] + 12, rock[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, rcol, 1, cv2.LINE_AA)

        status = ("ON SPOT" if on_spot else
                  "rock off spot" if (spot is not None and rock is not None) else
                  "lock the spot (clear the rock)" if not locked else
                  "no rock")
        scolour = (0, 255, 0) if on_spot else (200, 200, 200)
        cv2.rectangle(frame, (0, 0), (CAM_W - 160, 30), (35, 35, 35), -1)
        cv2.putText(frame, f"{status}   spot:{'locked' if locked else 'searching'}"
                    f"   stable:{stable_count}/{LOCK_FRAMES}",
                    (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, scolour, 1, cv2.LINE_AA)

        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("k"):
            locked = not locked
            stable_count = 0
            print(f"[pickup_spot] {'LOCKED' if locked else 'UNLOCKED'} spot={spot}")
        if key == ord("l"):
            print(f"[log] spot={spot} r={spot_r} locked={locked} "
                  f"rock={rock} on_spot={on_spot}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
