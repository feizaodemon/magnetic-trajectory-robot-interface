"""The optional Sakoe-Chiba window: off by default, never below the full search."""

import math
import random

from colmag_ros import dtw_bank_features as features
from colmag_ros import dtw_template_bank_tools as bank_tools


def _reference_dtw(a, b):
    """The full DTW loop, as it was before the window was added."""
    n, m = len(a), len(b)
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


def _stroke(rng, n):
    return features.normalize_points([(rng.random(), rng.random()) for _ in range(8)], n)


def test_default_and_wide_window_match_full_search_exactly():
    rng = random.Random(7)
    for n, m in ((64, 64), (40, 64), (64, 17), (2, 2)):
        a, b = _stroke(rng, n), _stroke(rng, m)
        expected = _reference_dtw(a, b)
        assert features.dtw_distance(a, b) == expected
        assert features.dtw_distance(a, b, window=max(n, m)) == expected


def test_narrow_window_never_undercuts_full_search():
    rng = random.Random(3)
    for width in (0, 1, 4, 16):
        a, b = _stroke(rng, 64), _stroke(rng, 48)
        windowed = features.dtw_distance(a, b, window=width)
        assert math.isfinite(windowed)
        assert windowed >= _reference_dtw(a, b)


def test_window_reaches_bank_ranking():
    rng = random.Random(5)
    query = _stroke(rng, 64)
    bank = {"templates": [
        {"id": "t1", "label": "1", "points": _stroke(rng, 64)},
        {"id": "t2", "label": "2", "points": _stroke(rng, 64)},
    ]}
    full = bank_tools.rank_labels_for_query(query, bank)
    narrow = bank_tools.rank_labels_for_query(query, bank, window=1)
    assert narrow["best_distance"] >= full["best_distance"]
    assert bank_tools.rank_labels_for_query(query, bank, window=64) == full
