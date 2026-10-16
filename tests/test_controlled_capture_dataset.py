"""The controlled-capture dataset files written by magnetic_ui_state_node.

Pins the file tree, file contents and CSV columns so moving the writer out of
the node changes nothing on disk. No ROS master or node is started.
"""

import csv
import json
from pathlib import Path

import magnetic_ui_state_node as state_node
from colmag_ros import controlled_capture_dataset
from test_magnetic_ui_state import arm_controlled_capture, captures, sample

METADATA_COLUMNS = [
    "sample_id",
    "dataset_label",
    "duration_sec",
    "point_count",
    "idle_end",
    "timeout_end",
    "image_path",
    "normalized_points_path",
]
README = (
    "# M33-E Controlled Capture Dataset\n\n"
    "Generated only when `controlled_capture_save_dataset:=true`.\n"
    "This dataset is offline evidence; it does not imply recognition improvement.\n"
)


def _capture_twice(tmp_path, **options):
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
        controlled_capture_min_duration_sec=0.6,
        controlled_capture_idle_sec=0.5,
        controlled_capture_min_ink_samples=4,
        controlled_capture_save_dataset=True,
        controlled_capture_dataset_dir=str(tmp_path / "dataset"),
        controlled_capture_dataset_label="two",
        **options,
    )
    payloads = []
    for start in (100.0, 200.0):
        arm_controlled_capture(machine, start=start)
        machine.process_sample(sample(start + 1.1, x=0.0, y=0.0))
        machine.process_sample(sample(start + 1.2, x=0.1, y=0.05))
        machine.process_sample(sample(start + 1.8, x=0.1, y=0.05))
        [payload] = captures(machine.process_sample(sample(start + 2.4, x=0.1, y=0.05)))
        payloads.append(payload)
    return payloads


def _tree(root):
    return sorted(str(path.relative_to(root)) for path in root.rglob("*"))


def test_dataset_files_and_contents(tmp_path):
    first, second = _capture_twice(tmp_path)
    root = tmp_path / "dataset"
    ids = [first["sample_id"], second["sample_id"]]
    assert ids == ["m33e_controlled_000001", "m33e_controlled_000002"]

    assert _tree(root) == sorted(
        ["README.md", "metadata.csv", "samples.jsonl", "normalized_points", "rendered_png"]
        + ["normalized_points/%s.json" % i for i in ids]
        + ["rendered_png/%s.png" % i for i in ids]
    )
    assert (root / "README.md").read_text(encoding="utf-8") == README

    for payload in (first, second):
        sample_id = payload["sample_id"]
        normalized_path = root / "normalized_points" / ("%s.json" % sample_id)
        image_path = root / "rendered_png" / ("%s.png" % sample_id)
        assert payload["normalized_points_path"] == str(normalized_path)
        assert payload["image_path"] == str(image_path)
        assert "image_error" not in payload
        assert normalized_path.read_text(encoding="utf-8") == json.dumps(
            {"sample_id": sample_id, "normalized_points": payload["normalized_points"]},
            indent=2, sort_keys=True) + "\n"
        assert image_path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"

    lines = (root / "samples.jsonl").read_text(encoding="utf-8").splitlines()
    assert lines == [json.dumps(p, sort_keys=True) for p in (first, second)]

    with (root / "metadata.csv").open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        assert reader.fieldnames == METADATA_COLUMNS
        rows = list(reader)
    assert rows == [
        {
            "sample_id": p["sample_id"],
            "dataset_label": "two",
            "duration_sec": str(p["duration_sec"]),
            "point_count": str(len(p["points"])),
            "idle_end": str(p["idle_end"]),
            "timeout_end": str(p["timeout_end"]),
            "image_path": p["image_path"],
            "normalized_points_path": p["normalized_points_path"],
        }
        for p in (first, second)
    ]


def test_rendered_png_is_the_renderer_output_for_the_raw_points(tmp_path):
    first, _second = _capture_twice(tmp_path)
    expected = tmp_path / "expected.png"
    controlled_capture_dataset._render_controlled_capture_png(first["points"], expected, image_size=64)
    written = tmp_path / "dataset" / "rendered_png" / ("%s.png" % first["sample_id"])
    assert written.read_bytes() == expected.read_bytes()


def test_png_rendering_can_be_turned_off(tmp_path):
    first, second = _capture_twice(tmp_path, controlled_capture_render_png=False)
    root = tmp_path / "dataset"
    assert "rendered_png" not in _tree(root)
    for payload in (first, second):
        assert "image_path" not in payload
    with (root / "metadata.csv").open(encoding="utf-8", newline="") as handle:
        assert [row["image_path"] for row in csv.DictReader(handle)] == ["", ""]


def test_writer_module_is_ros_free_and_shared_with_the_node():
    source = (Path(controlled_capture_dataset.__file__)).read_text(encoding="utf-8")
    for forbidden in ("rospy", "std_msgs", "Publisher(", "/colmag/"):
        assert forbidden not in source
    assert state_node.normalize_capture_points is controlled_capture_dataset.normalize_capture_points
