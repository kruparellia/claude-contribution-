#!/usr/bin/env python3
"""
colour_detect.py
----------------
Real-time colour detection from a USB webcam (Logitech C270).

For each frame:
  - Detects blobs of five primary/common colours using HSV masking.
  - Draws a labelled bounding box around every blob above a minimum size.
    Each label shows the colour name and a confidence % — the fraction of
    pixels inside the bounding rect that actually match the colour's HSV range.
  - Appends a right-side panel showing a global "colour presence" bar for
    every colour, i.e. what % of the total frame matches that colour.

Controls
--------
    Q   quit
    D   toggle debug mode — prints the HSV value of the centre pixel to the
        terminal and draws a crosshair there.  Useful when you need to tune
        the HSV ranges for your specific lighting.
    +   increase MIN_BLOB_AREA (reduces noise / small detections)
    -   decrease MIN_BLOB_AREA (picks up smaller objects)

Run
---
    python3 colour_detect.py
    python3 colour_detect.py --camera 1   # if C270 is on /dev/video1
"""

import argparse
import time

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Colour definitions
# ---------------------------------------------------------------------------
# OpenCV HSV scale:  H 0-179  (divide standard 0-360 degrees by 2)
#                    S 0-255
#                    V 0-255
#
# Red wraps the hue wheel, so it needs two separate inRange calls that are
# OR-ed together.  Every other colour fits in one contiguous hue band.
#
# 'display' is the BGR colour used for that colour's box, label, and sidebar bar.
# Tip: if a colour is being missed under your bench lighting, press D to read
# the centre-pixel HSV and compare it against the 'ranges' below.

COLOURS = [
    {
        "name": "RED",
        "ranges": [
            ((0,   110,  80), (10,  255, 255)),   # low-hue side of red
            ((170, 110,  80), (179, 255, 255)),   # high-hue side (wraps 360°)
        ],
        "display": (55, 40, 220),   # BGR
    },
    {
        "name": "GREEN",
        "ranges": [
            ((40, 80, 60), (85, 255, 255)),
        ],
        "display": (45, 190, 45),
    },
    {
        "name": "BLUE",
        "ranges": [
            ((100, 100, 55), (130, 255, 255)),
        ],
        "display": (210, 70, 35),
    },
    {
        "name": "YELLOW",
        "ranges": [
            ((22, 100, 100), (35, 255, 255)),
        ],
        "display": (25, 215, 230),
    },
    {
        "name": "ORANGE",
        "ranges": [
            ((11, 120, 100), (21, 255, 255)),
        ],
        "display": (15, 130, 255),
    },
]

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
MIN_BLOB_AREA   = 1500   # px² — contours smaller than this are ignored
MORPH_KERNEL_SZ = 7      # px — side length of the elliptical morphology kernel
SIDEBAR_W       = 210    # px — width of the right-side stats panel
CAM_W, CAM_H    = 640, 480


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mask(hsv: np.ndarray, colour: dict) -> np.ndarray:
    """
    Build a binary mask for one colour by OR-ing all of its HSV ranges.
    Returns a single-channel uint8 array, same H×W as `hsv`.
    """
    combined = None
    for lo, hi in colour["ranges"]:
        m = cv2.inRange(
            hsv,
            np.array(lo, dtype=np.uint8),
            np.array(hi, dtype=np.uint8),
        )
        combined = m if combined is None else cv2.bitwise_or(combined, m)
    return combined  # type: ignore[return-value]


def _clean_mask(mask: np.ndarray) -> np.ndarray:
    """
    Morphological open (erode then dilate) to remove single-pixel speckle,
    followed by a dilate pass to close small holes inside large blobs.
    """
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SZ, MORPH_KERNEL_SZ)
    )
    mask = cv2.erode(mask,  k, iterations=1)
    mask = cv2.dilate(mask, k, iterations=2)
    return mask


def _blob_confidence(clean_roi: np.ndarray) -> float:
    """
    Per-blob confidence score (0–100).

    Defined as: (pixels in the bounding-rect ROI that pass the colour mask)
                / (total pixels in that ROI)  × 100

    A perfectly saturated coloured object should score ~90–95 %.
    A mixed-colour region that barely triggered the contour threshold will
    score much lower, giving the viewer an intuitive quality signal.
    """
    total = clean_roi.size
    if total == 0:
        return 0.0
    return float(np.count_nonzero(clean_roi)) / total * 100.0


def _draw_label(frame: np.ndarray, text: str, x: int, y: int,
                w: int, h: int, bgr: tuple) -> None:
    """
    Draw a filled-background label above (or below if near top edge) a bbox.
    """
    font       = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.56
    thickness  = 1
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    pad    = 4
    label_y = y - pad - 2          # try above the box
    if label_y - th < 0:           # not enough room — put it below instead
        label_y = y + h + th + pad

    # Clamp horizontally so label never runs off the right edge
    label_x = min(x, frame.shape[1] - tw - 2 * pad - 2)

    cv2.rectangle(
        frame,
        (label_x,          label_y - th - pad),
        (label_x + tw + 2 * pad, label_y + baseline),
        bgr, -1,
    )
    cv2.putText(
        frame, text,
        (label_x + pad, label_y - 1),
        font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
    )


def _draw_sidebar(frame: np.ndarray, entries: list) -> np.ndarray:
    """
    Build a SIDEBAR_W-wide panel and hstack it onto the right of `frame`.

    `entries` is a list of (name, global_pct, bgr_tuple) — one per colour.
    """
    h = frame.shape[0]
    panel = np.full((h, SIDEBAR_W, 3), (28, 28, 28), dtype=np.uint8)

    n     = len(entries)
    row_h = h // n
    bx0   = 10                  # bar left edge
    bx1   = SIDEBAR_W - 10     # bar right edge

    for i, (name, pct, bgr) in enumerate(entries):
        y0 = i * row_h
        y1 = y0 + row_h

        # Colour name
        cv2.putText(
            panel, name, (bx0, y0 + 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.62, (210, 210, 210), 1, cv2.LINE_AA,
        )
        # Percentage value rendered in the colour's own hue
        cv2.putText(
            panel, f"{pct:.1f}%", (bx0, y0 + 47),
            cv2.FONT_HERSHEY_SIMPLEX, 0.56, bgr, 1, cv2.LINE_AA,
        )
        # Background track
        by0, by1 = y0 + 55, y0 + 70
        cv2.rectangle(panel, (bx0, by0), (bx1, by1), (65, 65, 65), -1)
        # Filled bar
        fill_w = int(min(pct, 100.0) / 100.0 * (bx1 - bx0))
        if fill_w > 0:
            cv2.rectangle(panel, (bx0, by0), (bx0 + fill_w, by1), bgr, -1)
        # Row separator
        if i < n - 1:
            cv2.line(panel, (0, y1 - 1), (SIDEBAR_W, y1 - 1), (55, 55, 55), 1)

    # Header label at very top right
    cv2.putText(
        panel, "GLOBAL %", (bx0, 14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 130, 130), 1, cv2.LINE_AA,
    )

    return np.hstack([frame, panel])


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(camera_index: int = 0) -> None:
    global MIN_BLOB_AREA

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS,          30)
    # Keep the internal buffer to 1 frame so we always process the freshest image.
    # Without this, on slower hardware the display lags behind by several frames.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[error] Cannot open camera index {camera_index}.")
        print("        Check the camera is plugged in and try --camera 1.")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[colour_detect] Camera {camera_index} opened at {actual_w}×{actual_h}")
    print("[colour_detect] Controls: Q=quit  D=toggle debug  +/- adjust min blob area")

    debug  = False
    prev_t = time.monotonic()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[warn] Missed frame — retrying.")
            continue

        # ---- FPS counter ----
        now    = time.monotonic()
        fps    = 1.0 / max(now - prev_t, 1e-9)
        prev_t = now

        hsv      = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        total_px = frame.shape[0] * frame.shape[1]

        sidebar_entries = []

        for colour in COLOURS:
            mask  = _make_mask(hsv, colour)
            clean = _clean_mask(mask)
            bgr   = colour["display"]

            # Global presence % for the sidebar
            global_pct = float(np.count_nonzero(clean)) / total_px * 100.0
            sidebar_entries.append((colour["name"], global_pct, bgr))

            # Find blobs and draw per-blob boxes
            contours, _ = cv2.findContours(
                clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                if cv2.contourArea(cnt) < MIN_BLOB_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                conf  = _blob_confidence(clean[y : y + h, x : x + w])
                label = f"{colour['name']}  {conf:.0f}%"

                cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
                _draw_label(frame, label, x, y, w, h, bgr)

        # ---- Debug overlay ----
        if debug:
            cy_px = frame.shape[0] // 2
            cx_px = frame.shape[1] // 2
            hv, sv, vv = int(hsv[cy_px, cx_px, 0]), int(hsv[cy_px, cx_px, 1]), int(hsv[cy_px, cx_px, 2])
            cv2.drawMarker(
                frame, (cx_px, cy_px), (255, 255, 255),
                cv2.MARKER_CROSS, markerSize=24, thickness=1, line_type=cv2.LINE_AA,
            )
            dbg_text = f"H:{hv} S:{sv} V:{vv}"
            cv2.putText(
                frame, dbg_text, (cx_px + 14, cy_px - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA,
            )
            print(f"[debug] centre  H={hv:3d}  S={sv:3d}  V={vv:3d}")

        # ---- HUD ----
        cv2.putText(
            frame, f"{fps:.0f} fps", (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA,
        )
        cv2.putText(
            frame, f"min area: {MIN_BLOB_AREA}px", (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1, cv2.LINE_AA,
        )

        cv2.imshow("Colour Detect", _draw_sidebar(frame, sidebar_entries))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("d"):
            debug = not debug
            print(f"[colour_detect] debug {'ON' if debug else 'OFF'}")
        elif key == ord("+") or key == ord("="):
            MIN_BLOB_AREA = min(MIN_BLOB_AREA + 250, 10000)
            print(f"[colour_detect] min blob area -> {MIN_BLOB_AREA} px²")
        elif key == ord("-"):
            MIN_BLOB_AREA = max(MIN_BLOB_AREA - 250, 250)
            print(f"[colour_detect] min blob area -> {MIN_BLOB_AREA} px²")

    cap.release()
    cv2.destroyAllWindows()
    print("[colour_detect] stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Real-time colour detection (EE6003)")
    parser.add_argument(
        "--camera", type=int, default=0,
        help="V4L2 camera index (default 0 → /dev/video0). Try 1 or 2 if C270 isn't found.",
    )
    args = parser.parse_args()
    main(camera_index=args.camera)
