#!/usr/bin/env python3
"""Rasterize 2D COLMAG trajectories into small grayscale images.

This utility is ROS-independent. It prepares an image representation that can
be reused by OCR, template matching, geometric recognizers, or debug views.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


DEFAULT_IMAGE_SIZE = 64
DEFAULT_MARGIN = 4
DEFAULT_LINE_THICKNESS = 2
SMALL_SPAN = 1e-12

Point = Tuple[float, float]
Pixel = Tuple[int, int]
Image = List[List[int]]


@dataclass(frozen=True)
class RasterizerConfig:
    image_size: int = DEFAULT_IMAGE_SIZE
    margin: int = DEFAULT_MARGIN
    line_thickness: int = DEFAULT_LINE_THICKNESS
    invert: bool = False


def rasterize_trajectory(
    points: Iterable[Sequence[float]],
    *,
    image_size: int = DEFAULT_IMAGE_SIZE,
    margin: int = DEFAULT_MARGIN,
    line_thickness: int = DEFAULT_LINE_THICKNESS,
    invert: bool = False,
) -> tuple[Image, dict]:
    """Convert finite ``[x, y]`` points into a square uint8 grayscale image."""
    config = RasterizerConfig(
        image_size=image_size,
        margin=margin,
        line_thickness=line_thickness,
        invert=invert,
    )
    _validate_config(config)

    finite_points = _coerce_finite_points(points)
    background = 255 if invert else 0
    image = [[background for _ in range(image_size)] for _ in range(image_size)]

    pixel_points, metadata = _normalize_to_pixels(finite_points, config)
    stroke = 0 if invert else 255
    if len(pixel_points) == 1:
        _draw_pixel(image, pixel_points[0], stroke, line_thickness)
    else:
        for start, end in zip(pixel_points, pixel_points[1:]):
            for pixel in _bresenham_line(start, end):
                _draw_pixel(image, pixel, stroke, line_thickness)

    metadata.update(
        {
            "image_size": image_size,
            "margin": margin,
            "line_thickness": line_thickness,
            "invert": invert,
            "pixel_value_note": (
                "0 background and 255 trajectory when invert=false; "
                "inverted when invert=true."
            ),
        }
    )
    return image, metadata


def write_png_grayscale(path: str | Path, image: Image) -> None:
    """Write an 8-bit grayscale PNG using only the Python standard library."""
    output_path = Path(path)
    height = len(image)
    width = len(image[0]) if image else 0
    if width <= 0 or height <= 0:
        raise ValueError("image must be non-empty")

    rows = []
    for row in image:
        if len(row) != width:
            raise ValueError("image rows must have equal length")
        rows.append(bytes([0]) + bytes(_clamp_u8(value) for value in row))

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        body = chunk_type + data
        return (
            struct.pack(">I", len(data))
            + body
            + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)
        )

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(b"".join(rows)))
        + chunk(b"IEND", b"")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(png)


def _validate_config(config: RasterizerConfig) -> None:
    if config.image_size < 2:
        raise ValueError("image_size must be at least 2")
    if config.margin < 0:
        raise ValueError("margin must be non-negative")
    if config.margin * 2 >= config.image_size:
        raise ValueError("margin leaves no drawable image area")
    if config.line_thickness < 1:
        raise ValueError("line_thickness must be at least 1")


def _coerce_finite_points(points: Iterable[Sequence[float]]) -> list[Point]:
    finite_points: list[Point] = []
    for point in points:
        if len(point) < 2:
            continue
        try:
            x = float(point[0])
            y = float(point[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            finite_points.append((x, y))
    return finite_points


def _normalize_to_pixels(points: list[Point], config: RasterizerConfig) -> tuple[list[Pixel], dict]:
    if not points:
        return [], {
            "point_count": 0,
            "bounds": None,
            "scale": 1.0,
            "degenerate": True,
            "pixel_points": [],
            "orientation": "x increases right; y increases upward in source and is flipped to image rows.",
        }

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = x_max - x_min
    y_span = y_max - y_min
    max_span = max(x_span, y_span)
    drawable_span = max(1, config.image_size - 1 - 2 * config.margin)

    # 保持 aspect ratio 可以避免把 A/C/S/X 等轨迹压扁成别的形状；
    # margin 让笔画不贴边，后续 OCR/template matching 更容易调试。
    degenerate = max_span <= SMALL_SPAN
    scale = drawable_span / max_span if not degenerate else 1.0
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    image_center = (config.image_size - 1) / 2.0

    pixels: list[Pixel] = []
    for x, y in points:
        if degenerate:
            column = int(round(image_center))
            row = int(round(image_center))
        else:
            column = int(round(image_center + (x - center_x) * scale))
            row = int(round(image_center - (y - center_y) * scale))
        pixels.append(
            (
                _clip_int(row, 0, config.image_size - 1),
                _clip_int(column, 0, config.image_size - 1),
            )
        )

    return pixels, {
        "point_count": len(points),
        "bounds": {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "x_span": x_span,
            "y_span": y_span,
            "max_span": max_span,
        },
        "scale": scale,
        "degenerate": degenerate,
        "pixel_points": pixels,
        "orientation": "x increases right; y increases upward in source and is flipped to image rows.",
    }


def _draw_pixel(image: Image, pixel: Pixel, value: int, line_thickness: int) -> None:
    row, column = pixel
    radius = max(0, (line_thickness - 1) // 2)
    image_size = len(image)
    for out_row in range(row - radius, row + radius + 1):
        if out_row < 0 or out_row >= image_size:
            continue
        for out_column in range(column - radius, column + radius + 1):
            if 0 <= out_column < image_size:
                image[out_row][out_column] = value


def _bresenham_line(start: Pixel, end: Pixel) -> list[Pixel]:
    row0, col0 = start
    row1, col1 = end
    d_col = abs(col1 - col0)
    d_row = -abs(row1 - row0)
    step_col = 1 if col0 < col1 else -1
    step_row = 1 if row0 < row1 else -1
    error = d_col + d_row

    pixels: list[Pixel] = []
    while True:
        pixels.append((row0, col0))
        if row0 == row1 and col0 == col1:
            break
        double_error = 2 * error
        if double_error >= d_row:
            error += d_row
            col0 += step_col
        if double_error <= d_col:
            error += d_col
            row0 += step_row
    return pixels


def _clip_int(value: int, lower: int, upper: int) -> int:
    return min(max(value, lower), upper)


def _clamp_u8(value: int | float) -> int:
    return int(min(max(value, 0), 255))
