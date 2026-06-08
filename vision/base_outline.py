#!/usr/bin/env python3
"""
base_outline.py
---------------
Find the white acrylic base disc in the camera view, draw a green outline
around it, and mark its centre (≈ the robot's rotation axis).  The base is
segmented as the largest bright, low-saturation region; an ellipse is fitted
to it (robust to the disc running off-frame or being partly occluded by the
arm).  The ArUco base-heading marker (ID 10, known 50 mm) gives pixel scale,
so the base diameter and the centre offset are reported in millimetres.

Two sliders let you tune the white threshold for the room lighting:
    V_min  — minimum brightness counted as "white"
    S_max  — maximum saturation counted as "white"

Controls
--------
    Q  quit
    S  save the annotated frame to /tmp/base_outline.png

Run
---
    python3 vision/base_outline.py
    python3 vision/base_outline.py --camera 0
"""
import argparse

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
HEADING_ID = 10
MARKER_MM  = 50.0

CAM_W, CAM_H = 640, 480
MIN_DISC_AREA = 20000        # ignore contours smaller than this (px²)

WIN = "base outline"


def largest_disc(mask: np.ndarray):
    """Return (ellipse, contour) of the largest blob in mask, or (None, None)."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5),  np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8), iterations=3)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, mask
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_DISC_AREA or len(c) < 5:
        return None, None, mask
    return cv2.fitEllipse(c), c, mask


def marker_scale(detector, gray):
    """Return (px_per_mm, marker_centre_px) from ID 10, or (None, None)."""
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or HEADING_ID not in ids.flatten():
        return None, None, None
    idx = int(np.where(ids.flatten() == HEADING_ID)[0][0])
    pts = corners[idx].reshape(4, 2)
    sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
    px_per_mm = (sum(sides) / 4.0) / MARKER_MM
    centre = tuple(pts.mean(axis=0).astype(int))
    return px_per_mm, centre, (corners[idx], ids[idx:idx + 1])


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    args = p.parse_args()

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

    print("[base_outline] running — q quit, s save")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        v_min = cv2.getTrackbarPos("V_min", WIN)
        s_max = cv2.getTrackbarPos("S_max", WIN)
        mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))

        ellipse, _cnt, _clean = largest_disc(mask)
        px_per_mm, marker_c, marker_draw = marker_scale(detector, gray)
        if marker_draw is not None:
            cv2.aruco.drawDetectedMarkers(frame, [marker_draw[0]], marker_draw[1])

        centre = None
        diam_mm = None
        if ellipse is not None:
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
            centre = tuple(map(int, ellipse[0]))
            cv2.drawMarker(frame, centre, (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
            cv2.putText(frame, "BASE CENTRE", (centre[0] + 12, centre[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            if px_per_mm:
                diam_mm = max(ellipse[1]) / px_per_mm

        hud = [
            f"base: {'yes' if ellipse else 'no'}   marker(ID10): {'yes' if px_per_mm else 'no'}",
            f"scale: {px_per_mm:.2f} px/mm" if px_per_mm else "scale: -",
            f"base diam ~ {diam_mm:.0f} mm" if diam_mm else "base diam: -",
        ]
        if centre and marker_c and px_per_mm:
            dx = (marker_c[0] - centre[0]) / px_per_mm
            dy = (marker_c[1] - centre[1]) / px_per_mm
            hud.append(f"marker offset from centre: dx={dx:+.0f} dy={dy:+.0f} mm")
        for i, line in enumerate(hud):
            cv2.putText(frame, line, (8, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            cv2.imwrite("/tmp/base_outline.png", frame)
            print("[base_outline] saved /tmp/base_outline.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
