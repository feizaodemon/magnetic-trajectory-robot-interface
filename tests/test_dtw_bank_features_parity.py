"""Golden parity tests for the curated DTW bank feature semantics."""

import math
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
TRACKED_BANK = (
    REPO / "colmag_ros" / "config" / "dtw_banks"
    / "m104c1_8symbol_mouse_seeded_yflip.json"
)

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from colmag_ros import dtw_bank_features as features  # noqa: E402
import dtw_template_bank_tools as bank_tools  # noqa: E402
import trajectory_symbol_top3_recognizer_node as recognizer  # noqa: E402


def _legacy_finite_float(value, default=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def _legacy_resample_by_arclength(points, n=64):
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


def _legacy_normalize_points(points, resample_length=64):
    clean = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        clean.append((_legacy_finite_float(point[0]), _legacy_finite_float(point[1])))
    if len(clean) < 2:
        return clean

    cx = sum(x for x, _ in clean) / len(clean)
    cy = sum(y for _, y in clean) / len(clean)
    centered = [(x - cx, y - cy) for x, y in clean]
    max_abs = max(max(abs(x), abs(y)) for x, y in centered)
    scale = max(max_abs, 1e-9)
    normalized = [(x / scale, y / scale) for x, y in centered]
    return _legacy_resample_by_arclength(normalized, n=resample_length)


def _legacy_dtw_distance(a, b):
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


def _legacy_rank(query, bank, top_k=3):
    scored = []
    for template in bank["templates"]:
        distance = _legacy_dtw_distance(query, template["points"])
        scored.append((distance, str(template.get("id", "")), str(template["label"])))
    scored.sort(key=lambda item: (item[0], item[1]))

    best_by_label = {}
    for distance, template_id, label in scored:
        if label not in best_by_label:
            best_by_label[label] = (distance, template_id)
    ranked = sorted(best_by_label.items(), key=lambda item: (item[1][0], item[0]))
    top = [
        (label, distance, template_id)
        for label, (distance, template_id) in ranked[:top_k]
    ]
    margin = ranked[1][1][0] - ranked[0][1][0]
    return top, margin


def test_centroid_max_abs_resample64_matches_frozen_reference_exactly():
    strokes = (
        [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [1.0, 3.0]],
        [[1.25, -0.5], [1.25, -0.5], [1.25, -0.5]],
        [["bad", 1.0], [float("nan"), 2.0], [3.0, float("inf")]],
    )
    for stroke in strokes:
        expected = _legacy_normalize_points(stroke, 64)
        actual = features.normalize_points(stroke, 64)
        assert actual == expected
        assert len(actual) == 64


def test_dtw_distance_matches_frozen_reference_exactly():
    first = _legacy_normalize_points([(0, 0), (1, 1), (2, 0)], 64)
    second = _legacy_normalize_points([(0, 0), (0.75, 1.2), (2, 0.1)], 64)
    assert features.dtw_distance(first, second) == _legacy_dtw_distance(first, second)
    assert math.isinf(features.dtw_distance([], second))


def test_runtime_compatibility_names_delegate_without_behavior_change():
    points = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)]
    assert recognizer.finite_float("nan", 7.0) == features.finite_float("nan", 7.0)
    assert recognizer.resample_by_arclength(points, 9) == features.resample_by_arclength(points, 9)
    assert recognizer.normalize_points(points, 64) == features.normalize_points(points, 64)
    normalized = features.normalize_points(points, 64)
    assert recognizer.dtw_distance(normalized, normalized) == features.dtw_distance(normalized, normalized)


def test_curated_bank_top3_distance_and_gate_match_frozen_reference():
    bank = bank_tools.load_template_bank(TRACKED_BANK)
    state = {
        "status": "ready",
        "path": str(TRACKED_BANK),
        "bank_name": TRACKED_BANK.stem,
        "bank": bank,
        "labels": list(bank["labels"]),
        "template_count": len(bank["templates"]),
    }

    for expected_label in ("1", "2", "3", "A", "X"):
        raw_query = next(
            template["points"] for template in bank["templates"]
            if template["label"] == expected_label
        )
        reference_query = _legacy_normalize_points(raw_query, 64)
        expected_top, expected_margin = _legacy_rank(reference_query, bank)

        candidates, gate, metadata = bank_tools.recognize_with_template_bank(
            raw_query,
            state,
            top_k=3,
            max_distance=0.12,
            min_margin=0.01,
            min_confidence=0.30,
        )

        assert expected_top[0][0] == expected_label
        assert [candidate["label"] for candidate in candidates] == [item[0] for item in expected_top]
        assert [candidate["distance"] for candidate in candidates] == [item[1] for item in expected_top]
        expected_confidences = [
            max(0.0, min(1.0, 1.0 - item[1] / 0.12))
            for item in expected_top
        ]
        assert [candidate["confidence"] for candidate in candidates] == expected_confidences
        assert candidates[0]["margin"] == expected_margin
        assert metadata["nearest_template"] == expected_top[0][2]

        expected_confidence = expected_confidences[0]
        expected_accepted = (
            expected_top[0][1] <= 0.12
            and expected_margin >= 0.01
            and expected_confidence >= 0.30
        )
        expected_reason = ""
        if expected_top[0][1] > 0.12:
            expected_reason = "dtw_distance_too_large"
        elif expected_margin < 0.01:
            expected_reason = "dtw_margin_too_small"
        elif expected_confidence < 0.30:
            expected_reason = "dtw_confidence_too_low"
        assert gate["top1_confidence"] == expected_confidence
        assert gate["best_distance"] == expected_top[0][1]
        assert gate["margin"] == expected_margin
        assert gate["accepted"] is expected_accepted
        assert gate["uncertain"] is (not expected_accepted)
        assert gate["uncertainty_reason"] == expected_reason
