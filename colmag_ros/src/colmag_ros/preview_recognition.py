"""Preview-only symbol recognition for the trajectory dashboard.

Production routes recognize symbols in the DTW template-bank recognizer node.
These helpers run the built-in DTW templates in-process for the dashboard
preview. They never publish anything and never touch a robot, task, or
confirm path.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from colmag_ros.dashboard_candidate_display import backend_display_name
from colmag_ros.dashboard_points import clean_stroke_points
from colmag_ros.dtw_bank_features import Point, dtw_distance, normalize_points

DEFAULT_LABELS = ("1", "2", "3", "V", "O", "X", "A", "C")

RECOGNIZER_BACKENDS = ("dtw",)


def _line(points: Sequence[Tuple[float, float]]) -> List[Point]:
    return [(float(x), float(y)) for x, y in points]


BUILTIN_TEMPLATE_POINTS: Dict[str, List[Point]] = {
    "A": _line([(-0.8, -0.8), (-0.45, 0.1), (0.0, 0.9), (0.45, 0.1), (0.8, -0.8), (0.35, -0.05), (-0.35, -0.05)]),
    "2": _line([(-0.65, 0.65), (-0.15, 0.9), (0.55, 0.65), (0.35, 0.15), (-0.45, -0.45), (-0.65, -0.8), (0.65, -0.8)]),
    "C": _line([(0.65, 0.65), (0.15, 0.9), (-0.55, 0.65), (-0.8, 0.0), (-0.55, -0.65), (0.15, -0.9), (0.65, -0.65)]),
    "X": _line([(-0.75, 0.75), (0.75, -0.75), (0.0, 0.0), (0.75, 0.75), (-0.75, -0.75)]),
    "1": _line([(-0.25, 0.55), (0.0, 0.85), (0.0, -0.8)]),
    "3": _line([(-0.55, 0.75), (0.45, 0.75), (0.0, 0.05), (0.5, -0.1), (0.35, -0.75), (-0.55, -0.7)]),
    "V": _line([(-0.75, 0.75), (0.0, -0.85), (0.75, 0.75)]),
    "O": _line([(0.0, 0.9), (0.65, 0.65), (0.9, 0.0), (0.65, -0.65), (0.0, -0.9), (-0.65, -0.65), (-0.9, 0.0), (-0.65, 0.65), (0.0, 0.9)]),
}


def build_builtin_templates(labels: Sequence[str] = DEFAULT_LABELS,
                            resample_length: int = 64) -> list[dict[str, Any]]:
    """Return normalized built-in DTW templates for the known ``labels``."""
    templates: list[dict[str, Any]] = []
    for label in labels:
        if label not in BUILTIN_TEMPLATE_POINTS:
            continue
        templates.append(
            {
                "label": label,
                "points": normalize_points(BUILTIN_TEMPLATE_POINTS[label], resample_length),
                "template_id": "builtin_%s" % label,
            }
        )
    return templates


def confidence_from_distances(distance: float, worst_distance: float) -> float:
    """Map a DTW distance to [0, 1] relative to the worst template distance."""
    if not math.isfinite(distance):
        return 0.0
    if not math.isfinite(worst_distance) or worst_distance <= 1e-9:
        return 1.0
    return max(0.0, min(1.0, 1.0 - distance / worst_distance))


def recognize_top3(points: Sequence[Any], labels: Sequence[str] = DEFAULT_LABELS,
                   resample_length: int = 64) -> list[dict[str, Any]]:
    """Rank the built-in templates by DTW distance and return the best three."""
    normalized = normalize_points(points, resample_length)
    templates = build_builtin_templates(labels, resample_length)
    distances = sorted(
        ((dtw_distance(normalized, template["points"]), template) for template in templates),
        key=lambda item: item[0],
    )

    top = distances[:3]
    worst_distance = distances[-1][0] if distances else float("inf")
    candidates: list[dict[str, Any]] = []
    for rank, (distance, template) in enumerate(top, start=1):
        candidates.append(
            {
                "rank": rank,
                "label": template["label"],
                "confidence": confidence_from_distances(distance, worst_distance),
                "distance": distance,
            }
        )
    return candidates


def recognize_trajectory_candidates(points: Any, min_points: int = 8,
                                    labels: Optional[Sequence[str]] = None,
                                    ) -> tuple[str, list[dict[str, Any]]]:
    """Preview-only top-3 recognition over a live stroke point sequence.

    Runs ``recognize_top3`` against the built-in DTW templates. Returns
    ``(status, candidates)`` where status is one of ``collecting`` / ``ready``
    / ``none`` and candidates is a list of ``{rank, label, confidence,
    distance}``.
    """
    pts = clean_stroke_points(points)
    if len(pts) < int(min_points):
        return "collecting", []
    kwargs: dict[str, Any] = {}
    if labels:
        kwargs["labels"] = tuple(labels)
    candidates = recognize_top3(pts, **kwargs)
    if not candidates:
        return "none", []
    return "ready", candidates


def recognize_stroke(points: Any, backend: Any = "dtw", min_points: int = 8,
                     labels: Optional[Sequence[str]] = None,
                     ) -> tuple[str, list[dict[str, Any]], str]:
    """Preview recognition with the built-in DTW templates.

    Returns ``(status, candidates, backend_label)``. Production routes use the
    external DTW template-bank node instead.
    """
    pts = clean_stroke_points(points)
    backend = str(backend or "dtw").strip().lower()

    if len(pts) < int(min_points):
        return "collecting", [], backend_display_name(backend)

    status, cands = recognize_trajectory_candidates(pts, min_points, labels)
    status = "no_text" if status == "none" else status
    return status, cands, backend_display_name("dtw")
