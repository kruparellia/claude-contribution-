#!/usr/bin/env python3
"""
shape_match.py
--------------
Identity check based on the 7-dim shape+colour signature built by
build_shape_refs.py.

Per feature we compute a z-score:
    z_i = |x_i - mean_i| / std_i

A feature counts as "in range" if z < z_tol (default 2.5).
The blob counts as the target object if at least `min_features` of the 7
features are in range AND total Mahalanobis-like distance is below
`max_total_z`.

This dual gate is important: an object can score 6/7 features in range but
still have one feature wildly wrong (e.g. completely different colour).
The total-z cap catches that.
"""

from pathlib import Path
import numpy as np

from shape_features import extract, N_FEATURES


class ShapeMatcher:
    def __init__(
        self,
        ref_npz_path: Path,
        z_tol: float = 2.5,
        min_features: int = 5,
        max_total_z: float = 12.0,
    ) -> None:
        data = np.load(str(ref_npz_path))
        self.mean = data["mean"].astype(np.float32)
        self.std  = data["std"].astype(np.float32)
        # Floor stddev to avoid div-by-zero when a feature is constant.
        self.std = np.maximum(self.std, 1e-3)
        self.z_tol        = z_tol
        self.min_features = min_features
        self.max_total_z  = max_total_z

    def score(self, contour, hsv) -> tuple[bool, int, float, np.ndarray]:
        """
        Returns (is_match, n_in_range, total_z, per_feature_z).

        n_in_range : how many of the 7 features have |z| < z_tol
        total_z    : sum of per-feature z-scores
        per_feature_z : full vector so the caller can log / display it
        """
        feats = extract(contour, hsv)
        z = np.abs(feats - self.mean) / self.std
        n_in_range = int(np.sum(z < self.z_tol))
        total_z = float(z.sum())
        is_match = (n_in_range >= self.min_features) and (total_z < self.max_total_z)
        return is_match, n_in_range, total_z, z
