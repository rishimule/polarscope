"""Polar-to-cartesian conversion and scan filtering."""
from __future__ import annotations

import numpy as np

RANGE_MIN_M = 0.05
RANGE_MAX_M = 12.0


def polar_to_xy(
    angles_rad: np.ndarray, ranges_m: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Radar convention: 0 rad = up (+y), pi/2 rad = right (+x)."""
    theta = (np.pi / 2.0) - angles_rad
    x = ranges_m * np.cos(theta)
    y = ranges_m * np.sin(theta)
    return x.astype(np.float32), y.astype(np.float32)


def filter_scan(
    angles_rad: np.ndarray,
    ranges_m: np.ndarray,
    qualities: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Drop out-of-range and zero-quality returns."""
    mask = (
        (ranges_m >= RANGE_MIN_M)
        & (ranges_m <= RANGE_MAX_M)
        & (qualities > 0)
    )
    return angles_rad[mask], ranges_m[mask], qualities[mask]
