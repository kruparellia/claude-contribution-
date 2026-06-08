#!/usr/bin/env python3
"""
base_register.py  (step 1 — rectification diagnostic)
-----------------------------------------------------
The camera views the circular base at an OBLIQUE angle, so the base images
as an ellipse and the "circles stay circles" assumption no longer holds.
This step uses the base-heading marker (ArUco ID 10, known 50 mm square) to
compute an image -> base-plane homography and warp the live frame into a
rectified, top-down ("bird's-eye") view.

Purpose of THIS step: just look at whether the concentric hole rings become
round again after the warp.  If they do, the homography is good enough and
the next step fits the rings to find the base centre + px-per-mm.  If they
stay elliptical / banana-shaped, C270 lens distortion is the culprit and we
add an intrinsics-calibration pass before going further.

The marker's own centre is used as a TEMPORARY plane origin (0,0); the true
base rotation axis gets recovered from the rings later.

Controls
--------
    Q  quit
    S  save the current rectified frame to /tmp/base_rectified.png

Run
---
    python3 vision/base_register.py
    python3 vision/base_register.py --camera 0
"""
import argparse

import cv2
import numpy as np

ARUCO_DICT   = cv2.aruco.DICT_4X4_50
HEADING_ID   = 10
MARKER_MM    = 50.0          # printed side length of the ID-10 base marker

# Rectified-output geometry. The marker centre maps to the canvas centre;
# the base centre is offset from there (marker sits on the 24-hole ring), so
# the canvas spans a generous area around the marker to keep the disc in view.
PX_PER_MM    = 1.5
OUT_SPAN_MM  = 520           # rectified canvas covers this many mm across
OUT_PX       = int(OUT_SPAN_MM * PX_PER_MM)

CAM_W, CAM_H = 640, 480


def marker_plane_corners() -> np.ndarray:
    """The 4 marker corners in base-plane mm, marker centred at origin.

    Order matches OpenCV ArUco corner order: TL, TR, BR, BL.
    """
    h = MARKER_MM / 2.0
    return np.float32([(-h, -h), (h, -h), (h, h), (-h, h)])


def plane_mm_to_out_px(pts_mm: np.ndarray) -> np.ndarray:
    """Map base-plane mm (origin at marker centre) to rectified-canvas px."""
    c = OUT_PX / 2.0
    out = pts_mm * PX_PER_MM
    out[:, 0] += c
    out[:, 1] += c
    return out.astype(np.float32)


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

    dst_px = plane_mm_to_out_px(marker_plane_corners())

    print("[base_register] running — q quit, s save rectified frame")
    blank = np.zeros((OUT_PX, OUT_PX, 3), np.uint8)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        rect = blank.copy()
        status = "ID 10 not seen"
        if ids is not None and HEADING_ID in ids.flatten():
            idx = int(np.where(ids.flatten() == HEADING_ID)[0][0])
            src_px = corners[idx].reshape(4, 2).astype(np.float32)
            cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], ids[idx:idx + 1])

            H = cv2.getPerspectiveTransform(src_px, dst_px)
            rect = cv2.warpPerspective(frame, H, (OUT_PX, OUT_PX))

            # reference grid on the rectified view: marker box + 50 mm rings
            c = OUT_PX // 2
            cv2.drawMarker(rect, (c, c), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            for r_mm in range(50, 301, 50):
                cv2.circle(rect, (c, c), int(r_mm * PX_PER_MM), (60, 120, 60), 1)
            status = "rectified (marker frame = origin)"

        cv2.putText(frame, status, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(rect, f"{PX_PER_MM:.1f} px/mm  green rings @50mm",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow("camera (raw)", frame)
        cv2.imshow("rectified top-down", rect)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            cv2.imwrite("/tmp/base_rectified.png", rect)
            print("[base_register] saved /tmp/base_rectified.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
