from pathlib import Path

from colmag_ros import dashboard_sample_lifecycle as lifecycle


def test_sample_cleanup_and_metadata_preserve_published_contract():
    raw = [(0.0, 0.0), (0.0001, 0.0), (0.01, 0.0), (0.02, 0.0)]

    cleaned = lifecycle.cleanup_drawing_sample_points(
        raw,
        min_point_delta=0.005,
        max_points=2,
        enabled=True,
    )
    metadata = lifecycle.sample_cleanup_metadata(
        raw,
        cleaned,
        min_point_delta=0.005,
        max_points=2,
        enabled=True,
    )

    assert cleaned == [(0.0, 0.0), (0.02, 0.0)]
    assert metadata == {
        "sample_cleanup_enabled": True,
        "sample_min_point_delta": 0.005,
        "sample_max_points": 2,
        "raw_point_count": 4,
        "published_point_count": 2,
    }


def test_drawing_sample_state_preserves_freeze_and_pen_up_order():
    state = lifecycle.update_drawing_zone_sample_state(
        {},
        (0.0, 0.0),
        inside_drawing_zone=True,
        input_active=True,
    )
    state = lifecycle.update_drawing_zone_sample_state(
        state,
        (0.1, 0.0),
        inside_drawing_zone=False,
        input_active=False,
    )

    assert state == {
        "active_points": [(0.0, 0.0), None],
        "frozen_points": [],
        "was_inside": False,
        "phase": "PEN_UP_PENDING",
    }

    frozen = lifecycle.update_drawing_zone_sample_state(
        state,
        (0.2, 0.0),
        inside_drawing_zone=False,
        input_active=True,
        inside_control_zone=True,
    )
    assert frozen["phase"] == "SAMPLE_FROZEN"
    assert frozen["frozen_points"] == [(0.0, 0.0), None]


def test_capture_payload_remains_simulation_safe_and_schema_compatible():
    payload = lifecycle.build_dashboard_drawing_zone_capture_payload(
        [(0.0, 0.0), (1.0, 1.0)],
        "sample-1",
        7,
        12.5,
        "/colmag/trajectory_2d",
        drawing_zone={"x1": 1, "y1": 2, "x2": 3, "y2": 4},
    )

    assert payload["capture_mode"] == "dashboard_drawing_zone"
    assert payload["controls_excluded_from_sample"] is True
    assert payload["controls_real_robot"] is False
    assert payload["test_only"] is False
    assert payload["points"] == [[0.0, 0.0], [1.0, 1.0]]
    assert payload["drawing_zone_canvas_rect"] == {
        "x1": 1.0,
        "y1": 2.0,
        "x2": 3.0,
        "y2": 4.0,
    }


def test_sample_module_has_no_ros_tk_dispatch_or_robot_dependencies():
    source = Path(lifecycle.__file__).read_text()
    for forbidden in (
        "import rospy",
        "import tkinter",
        "/colmag/task_command",
        "FollowJointTrajectory",
        "franka_control",
    ):
        assert forbidden not in source
