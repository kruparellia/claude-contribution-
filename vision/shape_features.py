#!/usr/bin/env python3
"""
shape_features.py
-----------------
Extract a fixed-length feature vector describing a coloured blob, from its
contour and the surrounding HSV image.

Features (7 dims, in order):
    0  aspect_ratio  — bbox width / height
    1  solidity      — contour area / convex-hull area      (0..1, lower = more concave)
    2  extent        — contour area / bbox area             (0..1, higher = fills bbox)
    3  circularity   — 4·pi·area / perimeter²               (1.0 = perfect circle)
    4  mean_h        — mean hue inside the mask             (0..179)
    5  mean_s        — mean saturation inside the mask      (0..255)
    6  mean_v        — mean value/brightness inside mask    (0..255)

This is deliberately small and interpretable so the matcher's behaviour is
easy to reason about and to write up in the report.
"""

import numpy as np
import cv2

FEATURE_NAMES = [
    "aspect_ratio",
    "solidity",
    "extent",
    "circularity",
    "mean_h",
    "mean_s",
    "mean_v",
]
N_FEATURES = len(FEATURE_NAMES)


def extract(contour: np.ndarray, hsv: np.ndarray) -> np.ndarray:
    """
    contour: Nx1x2 contour from cv2.findContours
    hsv:     full HSV frame (same dimensions as the image the contour came from)

    Returns a 1-D numpy array of length N_FEATURES, or a zero vector if the
    contour is degenerate.
    """
    out = np.zeros(N_FEATURES, dtype=np.float32)

    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, closed=True))
    if area <= 0 or perimeter <= 0:
        return out

    x, y, w, h = cv2.boundingRect(contour)
    if w <= 0 or h <= 0:
        return out

    # 0  aspect ratio
    out[0] = w / h

    # 1  solidity
    hull = cv2.convexHull(contour)
    hull_area = float(cv2.contourArea(hull))
    out[1] = area / hull_area if hull_area > 0 else 0.0

    # 2  extent
    out[2] = area / (w * h)

    # 3  circularity (Polsby–Popper / isoperimetric)
    out[3] = 4.0 * np.pi * area / (perimeter * perimeter)

    # 4-6  mean H, S, V inside the contour
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    mean_hsv = cv2.mean(hsv, mask=mask)[:3]   # (H, S, V)
    out[4] = mean_hsv[0]
    out[5] = mean_hsv[1]
    out[6] = mean_hsv[2]

    return out
