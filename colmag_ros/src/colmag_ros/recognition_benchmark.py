"""Leave-one-out robustness benchmark for the recognizers in ``gesture_recognizers``.

No ROS imports live here. Each template in the bank is held out once, the
recognizer is fitted on the rest, and the held-out stroke is classified clean
and under each perturbation level. Perturbations stand in for input that the
tracked mouse-drawn bank does not contain, such as board noise, a tilted
drawing, or a stroke that ends early. They are synthetic: a result here is not
evidence of accuracy on the real magnetic board.

Perturbations are seeded per (template, family, level, repeat), so every run
and every recognizer sees exactly the same queries.
"""

from __future__ import annotations

import math
import random
import time
from typing import Callable, Dict, List, Sequence, Tuple

from colmag_ros.gesture_recognizers import Point2D, Template, TemplateRecognizer

Perturbation = Callable[[List[Point2D], float, random.Random], List[Point2D]]


def add_noise(points: List[Point2D], sigma: float, rng: random.Random) -> List[Point2D]:
    """Gaussian jitter; ``sigma`` is relative to the stroke's larger bounding-box side."""
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    size = max(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
    return [(x + rng.gauss(0.0, sigma * size), y + rng.gauss(0.0, sigma * size)) for x, y in points]


def rotate(points: List[Point2D], degrees: float, rng: random.Random) -> List[Point2D]:
    """Rotate about the centroid by ``degrees``, clockwise or counter-clockwise at random."""
    angle = math.radians(degrees) * rng.choice((-1.0, 1.0))
    cx = sum(x for x, _ in points) / len(points)
    cy = sum(y for _, y in points) / len(points)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return [
        (cx + (x - cx) * cos_a - (y - cy) * sin_a, cy + (x - cx) * sin_a + (y - cy) * cos_a)
        for x, y in points
    ]


def truncate(points: List[Point2D], fraction: float, _rng: random.Random) -> List[Point2D]:
    """Drop the last ``fraction`` of the points, as if the pen lifted early."""
    keep = max(2, int(round(len(points) * (1.0 - fraction))))
    return points[:keep]


def stretch(points: List[Point2D], factor: float, rng: random.Random) -> List[Point2D]:
    """Scale one axis, chosen at random, by ``factor`` about the centroid."""
    cx = sum(x for x, _ in points) / len(points)
    cy = sum(y for _, y in points) / len(points)
    if rng.random() < 0.5:
        return [(cx + (x - cx) * factor, y) for x, y in points]
    return [(x, cy + (y - cy) * factor) for x, y in points]


PERTURBATIONS: Dict[str, Tuple[Perturbation, Tuple[float, ...], str]] = {
    "noise": (add_noise, (0.02, 0.05, 0.08, 0.12), "Gaussian noise, sigma / stroke size"),
    "rotation": (rotate, (10.0, 20.0, 30.0, 45.0), "Rotation, degrees"),
    "truncation": (truncate, (0.1, 0.2, 0.3, 0.4), "Truncated stroke, fraction removed"),
    "stretch": (stretch, (1.25, 1.5, 2.0, 2.5), "One-axis stretch, factor"),
}


def conditions(families: Sequence[str]) -> List[Tuple[str, float]]:
    """The clean condition followed by every (family, level) pair."""
    out = [("clean", 0.0)]
    for family in families:
        out.extend((family, level) for level in PERTURBATIONS[family][1])
    return out


def query_seed(index: int, family: str, level: float, repeat: int) -> str:
    return f"{index}:{family}:{level}:{repeat}"


def perturbed_query(points: List[Point2D], family: str, level: float,
                    rng: random.Random) -> List[Point2D]:
    if family == "clean":
        return list(points)
    return PERTURBATIONS[family][0](list(points), level, rng)


def evaluate_leave_one_out(recognizer: TemplateRecognizer, templates: Sequence[Template],
                           families: Sequence[str], repeats: int = 1) -> List[Dict[str, object]]:
    """Return one row per condition with top-1/top-3 accuracy and mean query time.

    Clean queries are deterministic, so they are scored once whatever
    ``repeats`` is.
    """
    plan = conditions(families)
    counts = {condition: [0, 0, 0] for condition in plan}  # queries, top-1, top-3
    elapsed = {condition: 0.0 for condition in plan}

    for index, (label, points) in enumerate(templates):
        recognizer.fit([template for i, template in enumerate(templates) if i != index])
        points = list(points)
        for family, level in plan:
            for repeat in range(1 if family == "clean" else repeats):
                rng = random.Random(query_seed(index, family, level, repeat))
                query = perturbed_query(points, family, level, rng)
                started = time.perf_counter()
                ranking = recognizer.rank(query)
                elapsed[(family, level)] += time.perf_counter() - started
                top = [ranked_label for ranked_label, _ in ranking[:3]]
                tally = counts[(family, level)]
                tally[0] += 1
                tally[1] += int(bool(top) and top[0] == label)
                tally[2] += int(label in top)

    rows = []
    for family, level in plan:
        queries, top1, top3 = counts[(family, level)]
        rows.append({
            "recognizer": recognizer.name,
            "family": family,
            "level": level,
            "queries": queries,
            "top1": top1 / queries if queries else 0.0,
            "top3": top3 / queries if queries else 0.0,
            "ms_per_query": 1000.0 * elapsed[(family, level)] / queries if queries else 0.0,
        })
    return rows
