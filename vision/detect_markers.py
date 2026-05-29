#!/usr/bin/env python3
"""
detect_markers.py
-----------------
Live ArUco-marker detection from the C270.  For each detected marker the
script draws the outline, marks the centroid, and labels the marker ID.

Used to verify that a printed marker is being picked up reliably under
the real lighting and at the actual working distance before wiring it
into the visual-servoing loop.

Controls
--------
    Q   quit

Run
---
    python3 vision/detect_markers.py
    python3 vision/detect_markers.py --camera 1
"""
import argparse

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50          # must match make_markers.py

def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{args.camera}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
    params     = cv2.aruco.DetectorParameters()
    detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # Built-in helper draws the four-corner outline + the ID.
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2)
                cx, cy = pts.mean(axis=0).astype(int)
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"({cx},{cy})",
                            (int(cx) + 8, int(cy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 1, cv2.LINE_AA)

        n = 0 if ids is None else len(ids)
        cv2.putText(frame, f"detected: {n}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("ArUco detect (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
