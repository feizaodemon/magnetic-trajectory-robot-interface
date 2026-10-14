"""Tests for 64x64 trajectory rasterization."""

from colmag_ros import trajectory_rasterizer


def test_rasterizer_preserves_aspect_ratio_and_flips_y():
    image, metadata = trajectory_rasterizer.rasterize_trajectory(
        [(0.0, 0.0), (2.0, 1.0)],
        image_size=64,
        margin=4,
        line_thickness=1,
    )

    first, second = metadata["pixel_points"]
    assert len(image) == 64
    assert len(image[0]) == 64
    assert metadata["bounds"]["x_span"] == 2.0
    assert metadata["bounds"]["y_span"] == 1.0
    assert metadata["scale"] == 27.5
    assert first[1] == 4
    assert second[1] == 59
    assert first[0] > second[0]
    assert any(value == 255 for row in image for value in row)


def test_rasterizer_handles_degenerate_points_at_center():
    image, metadata = trajectory_rasterizer.rasterize_trajectory(
        [(1.0, 1.0), (1.0, 1.0)],
        image_size=64,
        margin=4,
        line_thickness=2,
    )

    assert metadata["degenerate"] is True
    assert metadata["pixel_points"] == [(32, 32), (32, 32)]
    assert image[32][32] == 255


def test_write_png_grayscale(tmp_path):
    image, _metadata = trajectory_rasterizer.rasterize_trajectory([(0.0, 0.0), (1.0, 0.0)])
    output_path = tmp_path / "debug.png"

    trajectory_rasterizer.write_png_grayscale(output_path, image)

    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
