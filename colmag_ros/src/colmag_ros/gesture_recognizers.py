"""Alternative single-stroke recognizers for offline comparison with the DTW bank.

No ROS imports live here, and nothing here is wired into the runtime
recognizer node. Every recognizer is a nearest-template classifier with the
same interface, so the benchmark in ``colmag_ros.recognition_benchmark`` can
swap them freely:

``DTWRecognizer``
    The runtime method: centroid/max-abs normalization, 64-point arc-length
    resampling, and ``dtw_distance`` from ``colmag_ros.dtw_bank_features``,
    optionally with the same Sakoe-Chiba ``window`` the runtime can enable.
``DerivativeDTWRecognizer``
    DTW on the local slope of the normalized stroke (Keogh and Pazzani, 2001),
    which aligns shape changes instead of absolute positions.
``ProtractorRecognizer``
    The closed-form member of the $1 family (Li, 2010). Orientation-sensitive:
    the optimal rotation is searched only within ``max_rotation_deg`` so ``V``
    and an upside-down ``A`` stay distinct.
``PDollarRecognizer``
    The $P point-cloud matcher (Vatavu, Anthony and Wobbrock, 2012). It ignores
    stroke direction and point order.
``FeatureKNNRecognizer``
    A fixed-length vector of hand-crafted geometric features, standardized on
    the templates and compared with Euclidean distance.

All of them are pure Python so they run in the same Python 3.8 environment as
the ROS nodes, without NumPy.
"""

from __future__ import annotations

import functools
import math
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from colmag_ros.dtw_bank_features import (
    dtw_distance,
    normalize_points,
    resample_by_arclength,
)

Point2D = Tuple[float, float]
Template = Tuple[str, Sequence[Point2D]]
Ranking = List[Tuple[str, float]]


def as_points(points: Sequence[Any]) -> List[Point2D]:
    """Return the finite ``(x, y)`` pairs from a list of point-like items."""
    clean = []
    for point in points:
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            clean.append((x, y))
    return clean


def path_length(points: Sequence[Point2D]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


class TemplateRecognizer:
    """Nearest-template classifier; subclasses define the representation and distance.

    ``rank`` returns every label once, ordered by the distance of its closest
    template, the same rule the runtime DTW bank uses.
    """

    name = "template"

    def __init__(self) -> None:
        self._templates: List[Tuple[str, Any]] = []

    def preprocess(self, points: Sequence[Point2D]) -> Any:
        raise NotImplementedError

    def distance(self, query: Any, template: Any) -> float:
        raise NotImplementedError

    def fit(self, templates: Sequence[Template]) -> "TemplateRecognizer":
        self._templates = [
            (str(label), self.preprocess(as_points(points)))
            for label, points in templates
        ]
        return self

    def rank(self, points: Sequence[Point2D]) -> Ranking:
        query = self.preprocess(as_points(points))
        best: Dict[str, float] = {}
        for label, template in self._templates:
            distance = self.distance(query, template)
            if distance < best.get(label, math.inf):
                best[label] = distance
        return sorted(best.items(), key=lambda item: (item[1], item[0]))


class DTWRecognizer(TemplateRecognizer):
    name = "DTW"

    def __init__(self, resample_points: int = 64, window: Optional[int] = None) -> None:
        super().__init__()
        self.resample_points = resample_points
        self.window = window
        if window is not None:
            self.name = "%s (window %d)" % (type(self).name, window)

    def preprocess(self, points: Sequence[Point2D]) -> List[Point2D]:
        return normalize_points(points, resample_length=self.resample_points)

    def distance(self, query: List[Point2D], template: List[Point2D]) -> float:
        return dtw_distance(query, template, window=self.window)


def derivative_sequence(points: Sequence[Point2D]) -> List[Point2D]:
    """Keogh-Pazzani derivative estimate, averaging the backward and central slopes."""
    if len(points) < 3:
        return [(0.0, 0.0)] * len(points)
    out = []
    for prev, cur, nxt in zip(points, points[1:], points[2:]):
        out.append((
            ((cur[0] - prev[0]) + (nxt[0] - prev[0]) / 2.0) / 2.0,
            ((cur[1] - prev[1]) + (nxt[1] - prev[1]) / 2.0) / 2.0,
        ))
    return [out[0]] + out + [out[-1]]


class DerivativeDTWRecognizer(DTWRecognizer):
    name = "Derivative DTW"

    def preprocess(self, points: Sequence[Point2D]) -> List[Point2D]:
        resampled = super().preprocess(points)
        # Scale slopes back to roughly unit size so distances stay comparable.
        return [
            (dx * (self.resample_points - 1), dy * (self.resample_points - 1))
            for dx, dy in derivative_sequence(resampled)
        ]


def centered(points: Sequence[Point2D]) -> List[Point2D]:
    if not points:
        return []
    cx = sum(x for x, _ in points) / len(points)
    cy = sum(y for _, y in points) / len(points)
    return [(x - cx, y - cy) for x, y in points]


class ProtractorRecognizer(TemplateRecognizer):
    name = "Protractor ($1)"

    def __init__(self, resample_points: int = 16, max_rotation_deg: float = 45.0) -> None:
        super().__init__()
        self.resample_points = resample_points
        self.max_rotation = math.radians(max_rotation_deg)

    def preprocess(self, points: Sequence[Point2D]) -> List[float]:
        if len(points) < 2:
            return [0.0] * (2 * self.resample_points)
        resampled = resample_by_arclength(list(points), n=self.resample_points)
        vector = [value for point in centered(resampled) for value in point]
        norm = math.sqrt(sum(value * value for value in vector)) or 1.0
        return [value / norm for value in vector]

    def distance(self, query: List[float], template: List[float]) -> float:
        # Closed-form optimal rotation of the template onto the query (Li, 2010).
        a = b = 0.0
        for i in range(0, len(query), 2):
            a += template[i] * query[i] + template[i + 1] * query[i + 1]
            b += template[i] * query[i + 1] - template[i + 1] * query[i]
        angle = math.atan2(b, a)
        angle = max(-self.max_rotation, min(self.max_rotation, angle))
        similarity = a * math.cos(angle) + b * math.sin(angle)
        return math.acos(max(-1.0, min(1.0, similarity)))


class PDollarRecognizer(TemplateRecognizer):
    name = "$P point cloud"

    def __init__(self, resample_points: int = 16, epsilon: float = 0.5) -> None:
        super().__init__()
        self.resample_points = resample_points
        self.step = max(1, int(math.floor(resample_points ** (1.0 - epsilon))))

    def preprocess(self, points: Sequence[Point2D]) -> List[Point2D]:
        if len(points) < 2:
            return [(0.0, 0.0)] * self.resample_points
        resampled = resample_by_arclength(list(points), n=self.resample_points)
        xs = [x for x, _ in resampled]
        ys = [y for _, y in resampled]
        size = max(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
        return [(x / size, y / size) for x, y in centered(resampled)]

    def _cloud_distance(self, a: List[Point2D], b: List[Point2D], start: int) -> float:
        n = len(a)
        matched = [False] * n
        total = 0.0
        i = start
        for k in range(n):
            ax, ay = a[i]
            best_j, best_d = -1, math.inf
            for j in range(n):
                if not matched[j]:
                    d = math.hypot(ax - b[j][0], ay - b[j][1])
                    if d < best_d:
                        best_j, best_d = j, d
            matched[best_j] = True
            total += (1.0 - k / n) * best_d
            i = (i + 1) % n
        return total

    def distance(self, query: List[Point2D], template: List[Point2D]) -> float:
        best = math.inf
        for start in range(0, len(query), self.step):
            best = min(
                best,
                self._cloud_distance(query, template, start),
                self._cloud_distance(template, query, start),
            )
        return best


DIRECTION_BINS = 8
GRID_SIZE = 3


def stroke_features(points: Sequence[Point2D]) -> List[float]:
    """Translation- and scale-invariant geometric features of one stroke.

    Features: a length-weighted 8-bin direction histogram, total and absolute
    turning angle, start-to-end closure, bounding-box aspect, start and end
    position in the bounding box, and 3x3 grid occupancy.
    """
    resampled = resample_by_arclength(list(points), n=32) if len(points) >= 2 else list(points)
    if len(resampled) < 3:
        return [0.0] * (DIRECTION_BINS + 8 + GRID_SIZE * GRID_SIZE)

    xs = [x for x, _ in resampled]
    ys = [y for _, y in resampled]
    min_x, min_y = min(xs), min(ys)
    width, height = max(xs) - min_x, max(ys) - min_y
    size = max(width, height) or 1.0
    length = path_length(resampled) or 1.0

    histogram = [0.0] * DIRECTION_BINS
    headings = []
    for a, b in zip(resampled, resampled[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        step = math.hypot(dx, dy)
        if step <= 1e-12:
            continue
        heading = math.atan2(dy, dx)
        headings.append(heading)
        sector = int(((heading + math.pi) / (2 * math.pi)) * DIRECTION_BINS) % DIRECTION_BINS
        histogram[sector] += step / length

    turning = absolute_turning = 0.0
    for h0, h1 in zip(headings, headings[1:]):
        delta = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
        turning += delta
        absolute_turning += abs(delta)

    def relative(point: Point2D) -> Tuple[float, float]:
        return ((point[0] - min_x) / size, (point[1] - min_y) / size)

    start, end = relative(resampled[0]), relative(resampled[-1])
    closure = math.hypot(resampled[-1][0] - resampled[0][0], resampled[-1][1] - resampled[0][1]) / length

    grid = [0.0] * (GRID_SIZE * GRID_SIZE)
    for point in resampled:
        gx, gy = relative(point)
        col = min(GRID_SIZE - 1, int(gx * GRID_SIZE))
        row = min(GRID_SIZE - 1, int(gy * GRID_SIZE))
        grid[row * GRID_SIZE + col] += 1.0 / len(resampled)

    return histogram + [
        turning / (2 * math.pi),
        absolute_turning / (2 * math.pi),
        closure,
        math.log((width + 1e-3 * size) / (height + 1e-3 * size)),
        start[0], start[1],
        end[0], end[1],
    ] + grid


class FeatureKNNRecognizer(TemplateRecognizer):
    name = "Feature k-NN"

    def __init__(self) -> None:
        super().__init__()
        self._mean: List[float] = []
        self._scale: List[float] = []

    def fit(self, templates: Sequence[Template]) -> "FeatureKNNRecognizer":
        raw = [stroke_features(as_points(points)) for _, points in templates]
        count = len(raw) or 1
        dims = len(raw[0]) if raw else 0
        self._mean = [sum(row[d] for row in raw) / count for d in range(dims)]
        self._scale = [
            math.sqrt(sum((row[d] - self._mean[d]) ** 2 for row in raw) / count) or 1.0
            for d in range(dims)
        ]
        self._templates = [
            (str(label), self._standardize(row)) for (label, _), row in zip(templates, raw)
        ]
        return self

    def _standardize(self, row: Sequence[float]) -> List[float]:
        return [(value - mean) / scale for value, mean, scale in zip(row, self._mean, self._scale)]

    def preprocess(self, points: Sequence[Point2D]) -> List[float]:
        return self._standardize(stroke_features(points))

    def distance(self, query: List[float], template: List[float]) -> float:
        return math.sqrt(sum((q - t) ** 2 for q, t in zip(query, template)))


RECOGNIZERS: Dict[str, Callable[..., TemplateRecognizer]] = {
    "dtw": DTWRecognizer,
    # The optional Sakoe-Chiba window the runtime recognizer can enable.
    "dtw_window4": functools.partial(DTWRecognizer, window=4),
    "ddtw": DerivativeDTWRecognizer,
    "protractor": ProtractorRecognizer,
    "pdollar": PDollarRecognizer,
    "features": FeatureKNNRecognizer,
}


def make_recognizer(key: str, **kwargs: Any) -> TemplateRecognizer:
    """Build a recognizer by its ``RECOGNIZERS`` key; raises ``KeyError`` for unknown keys."""
    return RECOGNIZERS[key](**kwargs)
