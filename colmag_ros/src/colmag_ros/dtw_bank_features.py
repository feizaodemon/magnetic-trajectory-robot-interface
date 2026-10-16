"""Pure feature preprocessing and DTW distance for curated template banks.

This module preserves the current ``centroid + max_abs + resample64``
semantics. Historical first-point/range/resample96 paths intentionally remain
separate.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence, Tuple

from colmag_ros.param_utils import finite_float

Point = Tuple[float, float]


def resample_by_arclength(points: Sequence[Point], n: int = 64) -> list[Point]:
    """Resample a polyline to ``n`` points evenly spaced along its arc length."""
    if len(points) < 2:
        return list(points)

    dists = [0.0]
    for a, b in zip(points, points[1:]):
        dists.append(dists[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = dists[-1]
    if total <= 1e-12:
        return [points[0]] * n

    out: list[Point] = []
    j = 0
    for k in range(n):
        target = total * k / (n - 1)
        while j < len(dists) - 2 and dists[j + 1] < target:
            j += 1
        d0, d1 = dists[j], dists[j + 1]
        x0, y0 = points[j]
        x1, y1 = points[j + 1]
        alpha = 0.0 if d1 <= d0 else (target - d0) / (d1 - d0)
        out.append((x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)))
    return out


def normalize_points(points: Sequence[Any], resample_length: int = 64) -> list[Point]:
    """Center on the centroid, scale by the max abs coordinate, then resample by arc length."""
    clean: list[Point] = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        x = finite_float(point[0])
        y = finite_float(point[1])
        clean.append((x, y))

    if len(clean) < 2:
        return clean

    cx = sum(x for x, _ in clean) / len(clean)
    cy = sum(y for _, y in clean) / len(clean)
    centered = [(x - cx, y - cy) for x, y in clean]
    max_abs = max(max(abs(x), abs(y)) for x, y in centered)
    scale = max(max_abs, 1e-9)
    normalized = [(x / scale, y / scale) for x, y in centered]
    return resample_by_arclength(normalized, n=resample_length)


def dtw_distance(a: Sequence[Point], b: Sequence[Point], window: Optional[int] = None) -> float:
    """Return the DTW distance between two point sequences, divided by ``len(a) + len(b)``.

    ``window`` limits the warping path to a Sakoe-Chiba band of that many
    steps (widened to ``abs(len(a) - len(b))`` so a path always exists). It
    is faster but can only raise the distance. ``None``, the runtime default,
    searches the full matrix.
    """
    n, m = len(a), len(b)
    if n == 0 or m == 0:
        return float("inf")

    inf = float("inf")
    band = max(n, m) if window is None else max(int(window), abs(n - m))
    prev = [inf] * (m + 1)
    curr = [inf] * (m + 1)
    prev[0] = 0.0
    for i in range(1, n + 1):
        lo, hi = max(1, i - band), min(m, i + band)
        # Cells just outside the band may still hold values from two rows ago.
        curr[lo - 1] = inf
        if hi < m:
            curr[hi + 1] = inf
        ax, ay = a[i - 1]
        for j in range(lo, hi + 1):
            bx, by = b[j - 1]
            cost = math.hypot(ax - bx, ay - by)
            curr[j] = cost + min(prev[j], curr[j - 1], prev[j - 1])
        prev, curr = curr, prev
    return prev[m] / (n + m)
