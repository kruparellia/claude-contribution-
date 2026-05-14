#!/usr/bin/env python3
"""
build_shape_refs.py
-------------------
Compute the shape+colour signature for a target object from a directory of
reference photos.

For each .jpg in --ref-dir:
  1. Build a red HSV mask
  2. Find the largest contour
  3. Extract the 7-feature vector with shape_features.extract
  4. Stack into a (N, 7) matrix

Output .npz contains:
    features  — (N, 7) raw per-image feature vectors
    mean      — (7,)   per-feature mean across references
    std       — (7,)   per-feature stddev across references
    names     — list of feature names (for debugging)

Run:
    python3 vision/build_shape_refs.py
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from shape_features import extract, FEATURE_NAMES, N_FEATURES

RED_RANGES = [
    ((0,   110, 80), (10,  255, 255)),
    ((170, 110, 80), (179, 255, 255)),
]
MORPH_KERNEL_SZ = 7


def red_mask(hsv: np.ndarray) -> np.ndarray:
    m = None
    for lo, hi in RED_RANGES:
        cur = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cur if m is None else cv2.bitwise_or(m, cur)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (MORPH_KERNEL_SZ, MORPH_KERNEL_SZ))
    m = cv2.erode(m, k, iterations=1)
    m = cv2.dilate(m, k, iterations=2)
    return m


def build(ref_dir: Path, out_path: Path) -> None:
    img_paths = sorted(ref_dir.glob("*.jpg"))
    if not img_paths:
        raise SystemExit(f"no .jpg files in {ref_dir}")

    rows = []
    print(" ".join(f"{n:>13}" for n in ["file"] + FEATURE_NAMES))
    print("-" * (14 * (N_FEATURES + 1)))
    for p in img_paths:
        img = cv2.imread(str(p))
        if img is None:
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = red_mask(hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print(f"{p.name:<13}  <no red contour>")
            continue
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < 500:
            print(f"{p.name:<13}  <contour too small>")
            continue
        feats = extract(biggest, hsv)
        rows.append(feats)
        print(f"{p.name:>13} " +
              " ".join(f"{v:>13.3f}" for v in feats))

    if not rows:
        raise SystemExit("no usable references — aborting")

    features = np.vstack(rows).astype(np.float32)
    mean = features.mean(axis=0)
    std  = features.std(axis=0)

    print("-" * (14 * (N_FEATURES + 1)))
    print(f"{'mean':>13} " + " ".join(f"{v:>13.3f}" for v in mean))
    print(f"{'std':>13} " + " ".join(f"{v:>13.3f}" for v in std))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out_path,
             features=features, mean=mean, std=std,
             names=np.array(FEATURE_NAMES))
    print(f"\nsaved -> {out_path}  ({features.shape[0]} references)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build shape+colour signature (EE6003)")
    parser.add_argument("--ref-dir", type=Path,
                        default=Path("vision/refs/red_rock"))
    parser.add_argument("--out", type=Path,
                        default=Path("vision/refs/red_rock_shape.npz"))
    args = parser.parse_args()
    build(args.ref_dir, args.out)
