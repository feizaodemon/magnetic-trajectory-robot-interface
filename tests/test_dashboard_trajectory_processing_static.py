from pathlib import Path

from colmag_ros import dashboard_trajectory_processing as processing


def test_stroke_gate_modes_preserve_pass_through_and_bounds():
    assert processing.stroke_gate_pass("off", None, None)
    assert processing.stroke_gate_pass(
        "z_range", 0.2, None, z_min=None, z_max=None
    )
    assert processing.stroke_gate_pass(
        "z_range", 0.2, None, z_min=0.1, z_max=0.3
    )
    assert not processing.stroke_gate_pass(
        "z_range", 0.4, None, z_min=0.1, z_max=0.3
    )
    assert processing.stroke_gate_pass(
        "speed", None, None, min_speed=0.1, max_speed=1.0
    )
    assert not processing.stroke_gate_pass(
        "speed", None, 2.0, min_speed=0.1, max_speed=1.0
    )


def test_trajectory_bounds_preserve_fixed_symmetric_and_auto_contracts():
    fixed = (-1.0, 1.0, -2.0, 2.0)
    assert processing.compute_trajectory_bounds([], fixed) == fixed
    assert processing.compute_trajectory_bounds(
        [(5.0, 6.0)],
        fixed,
        scale_mode="fixed_symmetric",
        projection_extent=0.06,
    ) == (-0.06, 0.06, -0.06, 0.06)

    auto = processing.compute_trajectory_bounds(
        [(1.0, 2.0), (3.0, 4.0)],
        fixed,
        scale_mode="auto",
        padding_ratio=0.0,
        min_span=0.01,
    )
    assert auto == (1.0, 3.0, 2.0, 4.0)


def test_split_segments_drops_single_point_runs_without_reordering():
    trail = [(0, 0), None, (1, 1), (2, 2), None, (3, 3), (4, 4)]
    assert processing.split_trail_segments(trail) == [
        [(1, 1), (2, 2)],
        [(3, 3), (4, 4)],
    ]


def test_trajectory_processing_module_is_side_effect_free():
    source = Path(processing.__file__).read_text()
    for forbidden in (
        "import rospy",
        "import tkinter",
        "Publisher(",
        "Subscriber(",
        "FollowJointTrajectory",
        "/colmag/task_command",
    ):
        assert forbidden not in source
