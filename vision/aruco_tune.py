#!/usr/bin/env python3
"""
aruco_tune.py
-------------
A/B test for ArUco detector tuning.  Runs TWO detectors on every frame —
OpenCV's stock DetectorParameters vs a tuned set aimed at steep (oblique)
viewing angles — and overlays how many markers each one finds.  Rotate the
arm through its travel and watch which detector keeps decoding the claw
markers further off-axis.

The tuned set loosens the quad/border/bit tolerances and uses APRILTAG
corner refinement.  These extend the usable viewing angle at the cost of a
slightly higher false-positive risk — acceptable here because we only care
about a known ID set (claw 0-3, base heading 10), so stray IDs are ignored.

If the tuned detector clearly wins, the same parameters get promoted into
the real pipeline (servo_diagnostic.py / pickup_runner.py).

Controls
--------
    Q  quit
    L  log both detectors' current ID lists to the terminal

Run
---
    python3 vision/aruco_tune.py
    python3 vision/aruco_tune.py --camera 0
"""
import argparse

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
KNOWN_IDS  = {0, 1, 2, 3, 10}        # claw faces + base heading
CAM_W, CAM_H = 640, 480


def tuned_params() -> cv2.aruco.DetectorParameters:
    """DetectorParameters tuned for oblique / foreshortened markers."""
    p = cv2.aruco.DetectorParameters()
    # subpixel corner refinement — APRILTAG is the most robust variant
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    # widen adaptive-threshold window range (uneven illumination at angle)
    p.adaptiveThreshWinSizeMin  = 3
    p.adaptiveThreshWinSizeMax  = 53
    p.adaptiveThreshWinSizeStep = 4
    # accept smaller (foreshortened) and more distorted quads
    p.minMarkerPerimeterRate      = 0.02     # default 0.03
    p.polygonalApproxAccuracyRate = 0.08     # default 0.03
    # tolerate more border/bit error from a slanted marker
    p.maxErroneousBitsInBorderRate = 0.5     # default 0.35
    p.errorCorrectionRate          = 0.8     # default 0.6
    return p


def ids_in(ids) -> set:
    return set(int(i) for i in ids.flatten()) if ids is not None else set()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{args.camera}")

    adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    det_default = cv2.aruco.ArucoDetector(adict, cv2.aruco.DetectorParameters())
    det_tuned   = cv2.aruco.ArucoDetector(adict, tuned_params())

    print("[aruco_tune] running — q quit, l log")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        c_d, ids_d, _ = det_default.detectMarkers(gray)
        c_t, ids_t, _ = det_tuned.detectMarkers(gray)
        set_d = ids_in(ids_d) & KNOWN_IDS
        set_t = ids_in(ids_t) & KNOWN_IDS

        # draw tuned detections (green) and mark the ones ONLY tuned found
        if ids_t is not None:
            cv2.aruco.drawDetectedMarkers(frame, c_t, ids_t, (0, 255, 0))
        only_tuned = set_t - set_d

        cv2.rectangle(frame, (0, 0), (CAM_W, 52), (35, 35, 35), -1)
        cv2.putText(frame, f"default: {sorted(set_d)}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"tuned:   {sorted(set_t)}"
                    + (f"   (+{sorted(only_tuned)})" if only_tuned else ""),
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (120, 255, 120), 1, cv2.LINE_AA)

        cv2.imshow("ArUco tune A/B (q quit, l log)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("l"):
            print(f"[log] default={sorted(set_d)}  tuned={sorted(set_t)}"
                  f"  tuned-only={sorted(set_t - set_d)}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
