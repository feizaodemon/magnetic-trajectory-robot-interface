"""Package-level stroke-aware helpers for classifier and recognizer paths.

The helpers accept the old flat ``points`` representation and the newer
``strokes`` representation. They intentionally have no ROS dependency.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple


Point2D = Tuple[float, float]
Stroke = List[Point2D]


def coerce_point(point: Any) -> Optional[Point2D]:
    """Return a finite ``(x, y)`` from a pair or an ``x``/``y`` dict, or ``None``."""
    if point is None:
        return None
    x: Any
    y: Any
    try:
        if isinstance(point, dict):
            x = point.get("x")
            y = point.get("y")
            if x is None and isinstance(point.get("position"), dict):
                x = point["position"].get("x")
                y = point["position"].get("y")
            if x is None and isinstance(point.get("point"), dict):
                x = point["point"].get("x")
                y = point["point"].get("y")
        else:
            x = point[0]
            y = point[1]
        fx = float(x)
        fy = float(y)
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    if not (math.isfinite(fx) and math.isfinite(fy)):
        return None
    return fx, fy


def _normalize_point_list(points: Sequence[Any]) -> List[Stroke]:
    strokes: List[Stroke] = []
    current: Stroke = []
    for point in points or []:
        if point is None:
            if current:
                strokes.append(current)
                current = []
            continue
        coerced = coerce_point(point)
        if coerced is None:
            continue
        current.append(coerced)
    if current:
        strokes.append(current)
    return strokes


def normalize_strokes(strokes: Any) -> List[Stroke]:
    """Return the non-empty strokes of a list of point lists, dropping invalid points."""
    if not isinstance(strokes, list):
        return []
    out: List[Stroke] = []
    for stroke in strokes:
        if isinstance(stroke, list):
            clean = [pt for pt in (coerce_point(point) for point in stroke) if pt is not None]
            if clean:
                out.append(clean)
    return out


def normalize_payload_strokes(payload: Any) -> List[Stroke]:
    """Return valid strokes from a payload, flat points, or point list.

    Supported inputs:
    - ``{"points": [{"x": ..., "y": ...}, ...]}``
    - ``{"strokes": [[{"x": ..., "y": ...}], ...]}``
    - ``[{"x": ..., "y": ...}, None, {"x": ..., "y": ...}]``
    """
    if isinstance(payload, dict):
        strokes = normalize_strokes(payload.get("strokes"))
        if strokes:
            return strokes
        points = payload.get("points", [])
        return _normalize_point_list(points if isinstance(points, list) else [])
    if isinstance(payload, list):
        return _normalize_point_list(payload)
    return []


def flatten_strokes(strokes: Sequence[Sequence[Point2D]]) -> List[Point2D]:
    """Concatenate strokes into one flat point list."""
    return [point for stroke in strokes for point in stroke]


def stroke_metadata(strokes: Sequence[Sequence[Point2D]]) -> Dict[str, int]:
    """Return the non-empty stroke count and the total point count."""
    return {
        "stroke_count": len([stroke for stroke in strokes if stroke]),
        "point_count": len(flatten_strokes(strokes)),
    }
