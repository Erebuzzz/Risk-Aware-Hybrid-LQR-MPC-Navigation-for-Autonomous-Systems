"""
Path indexing helpers for stable tracking on figure-eights and open polylines.

Closed loop: forward-biased nearest projection reduces branch flips at self-crossings.
Open polyline: clamped indices — no false wrap from last to first waypoint.
"""

from __future__ import annotations

import numpy as np


def nearest_idx_closed(
    ref_x: np.ndarray,
    ref_y: np.ndarray,
    current_idx: int,
    px: float,
    py: float,
    n: int,
    i_back: int = 12,
    i_fwd: int = 100,
    tie_eps: float = 0.025,
) -> tuple[int, float]:
    """
    Nearest index on a closed polyline using a forward-biased window.

    Search steps in [current_idx - i_back, current_idx + i_fwd] on the circle.
    Among candidates within tie_eps of the minimum distance, prefer the smallest
    forward arc (idx - current_idx) mod n (continuity at crossings).
    """
    n = int(n)
    current_idx = int(current_idx) % n
    candidates: list[tuple[float, int]] = []
    for step in range(-i_back, i_fwd + 1):
        idx = (current_idx + step) % n
        d = float(np.hypot(ref_x[idx] - px, ref_y[idx] - py))
        candidates.append((d, idx))
    d_min = min(c[0] for c in candidates)
    close = [(d, idx) for d, idx in candidates if d <= d_min + tie_eps]
    close.sort(key=lambda t: (t[0], (t[1] - current_idx) % n))
    best_d, best_idx = close[0]
    return best_idx, best_d


def nearest_idx_open(
    wp_x: np.ndarray,
    wp_y: np.ndarray,
    wp_idx: int,
    px: float,
    py: float,
    n_wp: int,
    i_back: int = 12,
    i_fwd: int = 100,
    tie_eps: float = 0.025,
) -> tuple[int, float]:
    """
    Nearest index on an open polyline; indices stay in [0, n_wp - 1].
    Tie-break: minimum |idx - wp_idx| for continuity along the path.
    """
    n_wp = int(n_wp)
    if n_wp <= 0:
        return 0, 0.0
    wp_idx = int(np.clip(wp_idx, 0, n_wp - 1))
    lo = max(0, wp_idx - i_back)
    hi = min(n_wp - 1, wp_idx + i_fwd)
    candidates = [
        (float(np.hypot(wp_x[i] - px, wp_y[i] - py)), i) for i in range(lo, hi + 1)
    ]
    d_min = min(c[0] for c in candidates)
    close = [(d, idx) for d, idx in candidates if d <= d_min + tie_eps]
    close.sort(key=lambda t: (t[0], abs(t[1] - wp_idx)))
    return close[0][1], close[0][0]


def lookahead_idx_open(
    wp_x: np.ndarray,
    wp_y: np.ndarray,
    wp_idx: int,
    lookahead: float,
    n_wp: int,
) -> int:
    """First index at or beyond `lookahead` meters along the polyline forward from wp_idx."""
    n_wp = int(n_wp)
    if n_wp <= 1:
        return max(0, n_wp - 1)
    wp_idx = int(np.clip(wp_idx, 0, n_wp - 1))
    arc = 0.0
    j = wp_idx
    while j < n_wp - 1:
        seg = float(np.hypot(wp_x[j + 1] - wp_x[j], wp_y[j + 1] - wp_y[j]))
        arc += seg
        if arc >= lookahead:
            return j + 1
        j += 1
    return n_wp - 1
