"""ROS-free point and stroke helpers for the dashboard.

This module owns small parsing/cleanup routines shared by dashboard input,
preview recognition, and tests. It does not import ROS, Tk, recognizers, or any
robot execution path.
"""


def coerce_xy_point(point):
    """Return ``(x, y)`` floats from supported point shapes, or ``None``."""
    if isinstance(point, (list, tuple)) and len(point) >= 2:
        try:
            return float(point[0]), float(point[1])
        except (TypeError, ValueError):
            return None

    if not isinstance(point, dict):
        return None

    if "x" in point and "y" in point:
        try:
            return float(point["x"]), float(point["y"])
        except (TypeError, ValueError):
            return None

    for key in ("position", "point"):
        nested = point.get(key)
        if isinstance(nested, dict) and "x" in nested and "y" in nested:
            try:
                return float(nested["x"]), float(nested["y"])
            except (TypeError, ValueError):
                return None
    return None


def extract_xy_points(payload):
    """Extract a list of ``(x, y)`` floats from dashboard/capture payloads."""
    if not isinstance(payload, dict):
        return []

    points = None
    for key in ("trajectory", "points", "path", "points_2d"):
        if key in payload and isinstance(payload[key], list):
            points = payload[key]
            break

    if not points:
        return []

    out = []
    for point in points:
        xy = coerce_xy_point(point)
        if xy is not None:
            out.append(xy)
    return out


def extract_single_point(payload):
    """Extract one ``(x, y)`` float pair from a point payload."""
    if not isinstance(payload, dict):
        return None
    if "x" in payload and "y" in payload:
        try:
            return float(payload["x"]), float(payload["y"])
        except (TypeError, ValueError):
            return None
    for key in ("point", "position"):
        nested = payload.get(key)
        if isinstance(nested, dict) and "x" in nested and "y" in nested:
            try:
                return float(nested["x"]), float(nested["y"])
            except (TypeError, ValueError):
                return None
    return None


def clean_stroke_points(trail):
    """Writing points only: drop ``None`` stroke breaks, oldest to newest."""
    return [point for point in trail if point is not None]


def build_ocr_stroke_points(trail):
    """Clean stroke points for OCR/preview recognition."""
    return clean_stroke_points(trail)


def flatten_strokes(strokes):
    """Flatten ordered strokes into one ordered point list."""
    points = []
    for stroke in strokes:
        points.extend(stroke)
    return points
