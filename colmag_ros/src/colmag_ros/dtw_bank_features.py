"""Pure feature preprocessing and DTW distance for curated template banks.

This module preserves the current ``centroid + max_abs + resample64``
semantics. Historical first-point/range/resample96 paths intentionally remain
separate.
"""

import math


def finite_float(value, default=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def resample_by_arclength(points, n=64):
    if len(points) < 2:
        return list(points)

    dists = [0.0]
    for a, b in zip(points, points[1:]):
        dists.append(dists[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = dists[-1]
    if total <= 1e-12:
        return [points[0]] * n

    out = []
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


def normalize_points(points, resample_length=64):
    clean = []
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


def dtw_distance(a, b):
    n, m = len(a), len(b)
    if n == 0 or m == 0:
        return float("inf")

    inf = float("inf")
    prev = [inf] * (m + 1)
    curr = [inf] * (m + 1)
    prev[0] = 0.0
    for i in range(1, n + 1):
        curr[0] = inf
        ax, ay = a[i - 1]
        for j in range(1, m + 1):
            bx, by = b[j - 1]
            cost = math.hypot(ax - bx, ay - by)
            curr[j] = cost + min(prev[j], curr[j - 1], prev[j - 1])
        prev, curr = curr, prev
    return prev[m] / (n + m)
