"""Controlled-capture dataset files for magnetic_ui_state_node.

ROS-free: point normalization, the optional PNG rendering and the on-disk
dataset layout (``normalized_points/``, ``rendered_png/``, ``samples.jsonl``,
``metadata.csv``, ``README.md``). The node writes a dataset only when
``controlled_capture_save_dataset`` is true.
"""

import csv
import json
from pathlib import Path

from colmag_ros.param_utils import finite_float

DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE = 64
DEFAULT_CONTROLLED_CAPTURE_MARGIN = 4


def normalize_capture_points(points, image_size=DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE, margin=DEFAULT_CONTROLLED_CAPTURE_MARGIN):
    clean = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        x = finite_float(point[0])
        y = finite_float(point[1])
        clean.append((x, y))

    if not clean:
        return []

    xs = [x for x, _ in clean]
    ys = [y for _, y in clean]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = x_max - x_min
    y_span = y_max - y_min
    max_span = max(x_span, y_span)
    image_size = max(2, int(image_size))
    margin = max(0, int(margin))
    drawable = max(1.0, float(image_size - 1 - 2 * margin))
    bounded_extent = drawable / float(max(1, image_size - 1))

    if max_span <= 1e-12:
        return [[0.0, 0.0] for _x, _y in clean]

    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    scale = bounded_extent / max_span
    normalized = []
    for x, y in clean:
        normalized.append([
            max(-1.0, min(1.0, (x - center_x) * scale * 2.0)),
            max(-1.0, min(1.0, (y - center_y) * scale * 2.0)),
        ])
    return normalized


def _render_controlled_capture_png(points, output_path, image_size=DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE):
    from colmag_ros.trajectory_rasterizer import rasterize_trajectory, write_png_grayscale

    image, _metadata = rasterize_trajectory(
        points,
        image_size=int(image_size),
        margin=DEFAULT_CONTROLLED_CAPTURE_MARGIN,
        line_thickness=3,
        invert=True,
    )
    write_png_grayscale(output_path, image)


def write_controlled_capture_dataset(payload, dataset_dir, render_png=True,
                                     image_size=DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE):
    """Write one controlled-capture sample under ``dataset_dir``; return ``payload``.

    Writes ``normalized_points/<id>.json``, optionally ``rendered_png/<id>.png``,
    appends to ``samples.jsonl`` and ``metadata.csv``, and creates ``README.md``
    once. Adds ``normalized_points_path`` and ``image_path`` (or ``image_error``)
    to ``payload`` in place.
    """
    dataset_root = Path(dataset_dir)
    rendered_dir = dataset_root / "rendered_png"
    normalized_dir = dataset_root / "normalized_points"
    dataset_root.mkdir(parents=True, exist_ok=True)
    normalized_dir.mkdir(parents=True, exist_ok=True)

    sample_id = payload.get("sample_id", "sample")
    normalized_path = normalized_dir / ("%s.json" % sample_id)
    normalized_path.write_text(
        json.dumps(
            {
                "sample_id": sample_id,
                "normalized_points": payload.get("normalized_points", []),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    payload["normalized_points_path"] = str(normalized_path)

    if render_png:
        image_path = rendered_dir / ("%s.png" % sample_id)
        try:
            rendered_dir.mkdir(parents=True, exist_ok=True)
            _render_controlled_capture_png(
                payload.get("points", []),
                image_path,
                image_size=image_size,
            )
            payload["image_path"] = str(image_path)
        except Exception as exc:
            payload["image_error"] = str(exc)

    samples_path = dataset_root / "samples.jsonl"
    with samples_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")

    metadata_path = dataset_root / "metadata.csv"
    metadata_exists = metadata_path.exists()
    with metadata_path.open("a", encoding="utf-8", newline="") as handle:
        fieldnames = [
            "sample_id",
            "dataset_label",
            "duration_sec",
            "point_count",
            "idle_end",
            "timeout_end",
            "image_path",
            "normalized_points_path",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        if not metadata_exists:
            writer.writeheader()
        writer.writerow(
            {
                "sample_id": sample_id,
                "dataset_label": payload.get("dataset_label", ""),
                "duration_sec": payload.get("duration_sec", 0.0),
                "point_count": len(payload.get("points", [])),
                "idle_end": payload.get("idle_end", False),
                "timeout_end": payload.get("timeout_end", False),
                "image_path": payload.get("image_path", ""),
                "normalized_points_path": payload.get("normalized_points_path", ""),
            }
        )

    readme_path = dataset_root / "README.md"
    if not readme_path.exists():
        readme_path.write_text(
            "# M33-E Controlled Capture Dataset\n\n"
            "Generated only when `controlled_capture_save_dataset:=true`.\n"
            "This dataset is offline evidence; it does not imply recognition improvement.\n",
            encoding="utf-8",
        )

    return payload
