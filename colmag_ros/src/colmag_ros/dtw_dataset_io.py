"""Pure JSON/CSV helpers used by the DTW template-bank tooling."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple
from urllib.parse import quote

from colmag_ros.trajectory_strokes import flatten_strokes


Point2D = Tuple[float, float]


def normalize_point(point: Any) -> Optional[Point2D]:
    """Return a finite ``(x, y)`` from a pair or an ``x``/``y`` (or ``u``/``v``) dict, or ``None``."""
    if point is None:
        return None
    try:
        if isinstance(point, dict):
            x = point.get("x", point.get("u"))
            y = point.get("y", point.get("v"))
            if x is None and isinstance(point.get("position"), dict):
                x = point["position"].get("x", point["position"].get("u"))
                y = point["position"].get("y", point["position"].get("v"))
        else:
            x = point[0]
            y = point[1]
        fx = float(x)
        fy = float(y)
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    if not math.isfinite(fx) or not math.isfinite(fy):
        return None
    return fx, fy


def normalize_points(points: Any) -> List[Point2D]:
    """Return the valid points of a point list or of a payload dict's point list."""
    if isinstance(points, dict):
        points = extract_points_payload(points)
    result: List[Point2D] = []
    for point in points or []:
        normalized = normalize_point(point)
        if normalized is not None:
            result.append(normalized)
    return result


def extract_points_payload(payload: Dict[str, Any]) -> Any:
    """Return the first list under ``points``/``trajectory``/``path``/``points_2d``, else ``[]``."""
    for key in ("points", "trajectory", "path", "points_2d"):
        value = payload.get(key)
        if isinstance(value, list):
            return value
    return []


def normalize_strokes(payload: Any) -> List[List[Point2D]]:
    """Return strokes from ``strokes``, or split a flat point list at invalid points."""
    if isinstance(payload, dict) and isinstance(payload.get("strokes"), list):
        result: List[List[Point2D]] = []
        for stroke in payload["strokes"]:
            stroke_points = normalize_points(stroke)
            if stroke_points:
                result.append(stroke_points)
        return result
    points = extract_points_payload(payload) if isinstance(payload, dict) else payload
    result = []
    current: List[Point2D] = []
    for point in points or []:
        normalized = normalize_point(point)
        if normalized is None:
            if current:
                result.append(current)
                current = []
        else:
            current.append(normalized)
    if current:
        result.append(current)
    return result


def parse_json_text(text: Any) -> Dict[str, Any]:
    """Parse JSON text (or copy a dict); return ``{}`` unless it is a JSON object."""
    if isinstance(text, dict):
        return dict(text)
    try:
        data = json.loads(str(text or ""))
    except ValueError:
        return {}
    return data if isinstance(data, dict) else {}


def load_standalone_seed(path: Any) -> Dict[str, Any]:
    """Load a standalone seed JSON file as a labeled sample record."""
    data = parse_json_text(Path(path).read_text())
    strokes = normalize_strokes(data)
    return {
        "source": "standalone_seed",
        "file": str(path),
        "intended_label": str(data.get("label", "")),
        "points": flatten_strokes(strokes),
        "strokes": strokes,
        "stroke_count": len(strokes),
        "raw": data,
    }


def load_runtime_sample(path: Any) -> Dict[str, Any]:
    """Load a recorded runtime sample, preferring its raw symbol capture."""
    data = parse_json_text(Path(path).read_text())
    capture = data.get("raw_symbol_capture")
    if not isinstance(capture, dict):
        capture = parse_json_text(data.get("raw_symbol_capture_json", ""))
    strokes = normalize_strokes(capture if capture else data)
    return {
        "source": "runtime_sample",
        "file": str(path),
        "intended_label": str(data.get("intended_label", "")),
        "points": flatten_strokes(strokes),
        "raw": data,
    }


def resolve_repo_path(path: Any, repo: Any) -> Path:
    """Return ``path`` unchanged when absolute, else relative to ``repo``."""
    value = Path(path)
    return value if value.is_absolute() else Path(repo) / value


def seed_files_by_label(seeds_root: Any, labels: Sequence[str]) -> Dict[str, List[Path]]:
    """Return ``{label: sorted seed JSON paths}`` from ``<seeds_root>/<quoted label>/``."""
    root = Path(seeds_root)
    per_label: Dict[str, List[Path]] = {}
    for label in labels:
        label_dir = root / quote(label, safe="-_.~")
        per_label[label] = sorted(label_dir.glob("*.json")) if label_dir.is_dir() else []
    return per_label


def ensure_output_dir(path: Any, allowed_prefixes: Sequence[Any]) -> Path:
    """Create and return ``path``; raise ``ValueError`` unless it is under an allowed prefix."""
    resolved = Path(path).resolve()
    allowed = [Path(prefix).resolve() for prefix in allowed_prefixes]
    if not any(resolved == prefix or prefix in resolved.parents for prefix in allowed):
        raise ValueError("output path must stay under ignored repo-local outputs")
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved


def write_csv(path: Any, rows: Sequence[Dict[str, Any]]) -> Path:
    """Write ``rows`` as CSV with sorted union-of-keys columns."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in rows for key in row})
    with out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return out


def write_json(path: Any, data: Any) -> Path:
    """Write ``data`` as indented, key-sorted JSON."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    return out
