"""Depth sampling and validity checks. No ROS imports."""

import numpy as np

MIN_DEPTH = 0.5  # m
MAX_DEPTH = 10.0  # m


def depth_in_metres(depth_image: np.ndarray) -> np.ndarray:
    """Depth images are either float metres (32FC1) or integer millimetres (16UC1)."""
    if np.issubdtype(depth_image.dtype, np.integer):
        return depth_image.astype(np.float32) / 1000.0
    return depth_image


def is_valid_depth(z: float, min_depth: float = MIN_DEPTH, max_depth: float = MAX_DEPTH) -> bool:
    """Band-pass: finite and within ``[min_depth, max_depth]``."""
    return bool(np.isfinite(z)) and min_depth <= z <= max_depth


def sample_depth(
    depth_image: np.ndarray,
    u: float,
    v: float,
    half_window: int = 5,
    min_depth: float = MIN_DEPTH,
    max_depth: float = MAX_DEPTH,
) -> float | None:
    """Median of the valid depth pixels in a window centred on ``(u, v)``.

    The window is ``2 * half_window + 1`` pixels wide and is clipped at the
    image border. Returns ``None`` if the centre is outside the image or no
    pixel in the window passes the band-pass.
    """
    height, width = depth_image.shape[:2]
    col, row = int(round(u)), int(round(v))
    if not (0 <= col < width and 0 <= row < height):
        return None
    window = depth_in_metres(
        depth_image[
            max(row - half_window, 0) : row + half_window + 1,
            max(col - half_window, 0) : col + half_window + 1,
        ]
    )
    valid = window[np.isfinite(window) & (window >= min_depth) & (window <= max_depth)]
    if valid.size == 0:
        return None
    return float(np.median(valid))
