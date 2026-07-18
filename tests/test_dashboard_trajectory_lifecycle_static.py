from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
    MagneticTrajectoryDashboardNode,
)


class _Variable:
    def __init__(self, name, events):
        self.name = name
        self.events = events
        self.value = None

    def set(self, value):
        self.value = value
        self.events.append(("set", self.name, value))


class _Root:
    def __init__(self, events):
        self.events = events

    def after(self, delay, callback):
        self.events.append(("after", delay))
        callback()


def _node():
    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    events = []
    node.input_z_min = None
    node.input_z_max = None
    node.input_valid_gate_enabled = True
    node._input_active_streak = 0
    node.input_reactivate_samples = 1
    node._prev_sample = None
    node.stroke_gate_mode = "off"
    node.gate_z_min = None
    node.gate_z_max = None
    node.gate_min_speed = 0.0
    node.gate_max_speed = 0.0
    node.trajectory_sample_count = 0
    node.drawing_sample_state = {}
    node.drawing_zone_trail = []
    node.trail = []
    node.gate_invalid_breaks_stroke = True
    node.root = _Root(events)
    node.traj_valid_var = _Variable("valid", events)
    node.traj_xyz_var = _Variable("xyz", events)
    node.traj_count_var = _Variable("count", events)
    node.traj_input_gate_var = _Variable("input", events)
    node.traj_filter_var = _Variable("filter", events)
    node.trajectory_recording_enabled = True
    node.candidate_preview_enabled = True
    node.auto_recognize = True
    node._trajectory_point_in_drawing_zone = lambda point: True
    node._trajectory_point_in_control_zone = lambda point: False
    node._start_new_drawing_after_recognized_if_needed = (
        lambda inside, writing: events.append(("start_new", inside, writing))
    )
    node._sync_drawing_sample_buffers = lambda: events.append(("sync",))
    node._schedule_ui_refresh = lambda: events.append(("refresh",))
    node._feed_recorder = (
        lambda point, z, timestamp, valid, writing: events.append(
            ("record", point, z, timestamp, valid, writing)
        )
    )
    node._schedule_auto_recognize = lambda: events.append(("recognize",))
    return node, events


def test_ingest_writing_sample_preserves_state_text_and_effect_order():
    node, events = _node()

    node._ingest_trajectory_sample(
        {
            "timestamp": 10.0,
            "sample_index": 7,
            "z": 0.2,
            "valid": True,
            "filter_mode": "raw",
        },
        (0.1, 0.2),
    )

    assert node.latest_input_active is True
    assert node.latest_writing is True
    assert node.latest_inside_drawing_zone is True
    assert node.latest_inside_control_zone is False
    assert node.trail == [(0.1, 0.2)]
    assert node.drawing_zone_trail == [(0.1, 0.2)]
    assert node.traj_valid_var.value == "Valid: true | input active: yes | writing: yes"
    assert node.traj_xyz_var.value == "x/y/z: 0.1000 / 0.2000 / 0.2000"
    assert node.traj_count_var.value == "Samples: 1 (idx 7)"
    assert node.traj_filter_var.value == "Filter: raw"

    names = [event[0] for event in events]
    assert names.index("start_new") < names.index("sync")
    assert names.index("sync") < names.index("refresh")
    assert names.index("refresh") < names.index("record")
    assert names.index("record") < names.index("recognize")


def test_ingest_invalid_sample_preserves_stroke_break_and_no_recognition():
    node, events = _node()
    node.trail = [(0.0, 0.0)]
    node.drawing_zone_trail = [(0.0, 0.0)]
    node.drawing_sample_state = {
        "active_points": [(0.0, 0.0)],
        "frozen_points": [],
        "was_inside": True,
        "phase": "DRAWING_ACTIVE",
    }

    node._ingest_trajectory_sample(
        {"timestamp": 11.0, "valid": False},
        (0.1, 0.1),
    )

    assert node.latest_input_active is False
    assert node.latest_input_inactive_reason == "valid=false"
    assert node.latest_writing is False
    assert node.trail == [(0.0, 0.0), None]
    assert node.drawing_zone_trail == [(0.0, 0.0), None]
    assert "recognize" not in [event[0] for event in events]
    assert events[-1] == ("record", (0.1, 0.1), None, 11.0, False, False)
