import importlib.util
from pathlib import Path


def load_state_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "colmag_ros"
        / "scripts"
        / "magnetic_ui_state_node.py"
    )
    spec = importlib.util.spec_from_file_location("magnetic_ui_state_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


state_node = load_state_module()


def sample(timestamp, x=0.048, y=0.030, valid=True, sequence_id=1):
    return {
        "timestamp": timestamp,
        "sequence_id": sequence_id,
        "sample_index": int(timestamp * 10),
        "x": x,
        "y": y,
        "valid": valid,
        "source": "test",
    }


def last_ui(events):
    ui_events = [payload for kind, payload in events if kind == "ui"]
    assert ui_events
    return ui_events[-1]


def captures(events):
    return [payload for kind, payload in events if kind == "capture"]


def arm_controlled_capture(machine, start=10.0):
    machine.process_sample(sample(start))
    events = machine.process_sample(sample(start + machine.dwell_sec))
    assert last_ui(events)["state"] == "CONTROLLED_CAPTURE_READY"


def test_b_zone_dwell_enters_digit_mode_only_after_threshold():
    machine = state_node.MagneticUIStateMachine(dwell_sec=2.0)

    assert last_ui(machine.process_sample(sample(10.0)))["state"] == "HOVER_MENU"
    early = last_ui(machine.process_sample(sample(11.5)))
    assert early["state"] == "HOVER_MENU"
    assert early["dwell_progress"] < 1.0

    armed = last_ui(machine.process_sample(sample(12.0)))
    assert armed["state"] == "DIGIT_MODE_ARMED"
    assert armed["dwell_progress"] == 1.0


def test_leaving_b_zone_before_dwell_cancels_hover():
    machine = state_node.MagneticUIStateMachine(dwell_sec=2.0)

    machine.process_sample(sample(20.0))
    cancelled = last_ui(machine.process_sample(sample(20.5, x=0.0, y=0.0)))

    assert cancelled["state"] == "IDLE"
    assert "cancelled" in cancelled["message"]


def test_capture_publishes_only_when_enough_points_exist():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        capture_window_sec=0.4,
        min_capture_points=4,
    )

    machine.process_sample(sample(30.0))
    machine.process_sample(sample(31.0))
    machine.process_sample(sample(31.1, x=-0.02, y=0.02))
    machine.process_sample(sample(31.2, x=-0.01, y=0.01))
    machine.process_sample(sample(31.3, x=0.00, y=0.00))
    events = machine.process_sample(sample(31.6, x=0.02, y=-0.02))

    captures = [payload for kind, payload in events if kind == "capture"]
    assert len(captures) == 1
    assert captures[0]["sample_id"] == "m28_symbol_000001"
    assert len(captures[0]["points"]) >= 4
    assert last_ui(events)["state"] == "WAIT_CONFIRM"


def test_capture_with_too_few_points_is_cancelled():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        capture_window_sec=0.2,
        min_capture_points=6,
    )

    machine.process_sample(sample(40.0))
    machine.process_sample(sample(41.0))
    machine.process_sample(sample(41.1, x=-0.02, y=0.02))
    events = machine.process_sample(sample(41.4, x=-0.01, y=0.01))

    assert [payload for kind, payload in events if kind == "capture"] == []
    assert last_ui(events)["state"] == "CANCELLED"


def test_a_zone_locks_motion_and_x_zone_cancels():
    machine = state_node.MagneticUIStateMachine()

    locked = last_ui(machine.process_sample(sample(50.0, x=0.024, y=0.030)))
    assert locked["state"] == "MOTION_LOCKED"

    machine = state_node.MagneticUIStateMachine()
    cancelled = last_ui(machine.process_sample(sample(51.0, x=0.024, y=0.000)))
    assert cancelled["state"] == "CANCELLED"


def test_legacy_control_zones_can_be_disabled_without_capture_actions():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=0.1,
        capture_window_sec=0.1,
        min_capture_points=1,
        enable_legacy_control_zones=False,
    )

    events = []
    for index, (x, y) in enumerate(
        ((0.048, 0.030), (0.024, 0.030), (0.024, 0.000))
    ):
        events.extend(machine.process_sample(sample(60.0 + index, x=x, y=y)))

    assert captures(events) == []
    assert machine.state == "IDLE"
    assert machine.last_active_zone == ""
    assert all(last_ui([event])["active_zone"] == "" for event in events)


def test_controlled_capture_disabled_preserves_default_fixed_window_behavior():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        capture_window_sec=0.4,
        min_capture_points=4,
        controlled_capture_enabled=False,
    )

    machine.process_sample(sample(60.0))
    armed = machine.process_sample(sample(61.0))
    assert last_ui(armed)["state"] == "DIGIT_MODE_ARMED"

    machine.process_sample(sample(61.1, x=-0.02, y=0.02))
    machine.process_sample(sample(61.2, x=-0.01, y=0.01))
    machine.process_sample(sample(61.3, x=0.00, y=0.00))
    events = machine.process_sample(sample(61.6, x=0.02, y=-0.02))

    assert len(captures(events)) == 1
    assert captures(events)[0]["sample_id"].startswith("m28_symbol_")
    assert "capture_mode" not in captures(events)[0]


def test_compute_point_velocity_uses_xy_distance_over_dt():
    speed = state_node.compute_point_velocity((0.0, 0.0), (3.0, 4.0), 2.0)

    assert speed == 2.5


def test_idle_gate_does_not_finish_before_minimum_duration():
    decision = state_node.should_finish_controlled_capture(
        0.5,
        10,
        1.0,
        min_duration_sec=0.6,
        min_ink_samples=8,
        idle_sec=0.5,
        timeout_sec=4.0,
    )

    assert decision["finish"] is False
    assert decision["idle_end"] is False


def test_idle_gate_does_not_finish_before_minimum_ink_samples():
    decision = state_node.should_finish_controlled_capture(
        1.0,
        7,
        1.0,
        min_duration_sec=0.6,
        min_ink_samples=8,
        idle_sec=0.5,
        timeout_sec=4.0,
    )

    assert decision["finish"] is False
    assert decision["min_ink_samples_passed"] is False


def test_idle_gate_finishes_after_low_velocity_for_idle_sec():
    decision = state_node.should_finish_controlled_capture(
        1.2,
        8,
        0.6,
        min_duration_sec=0.6,
        min_ink_samples=8,
        idle_sec=0.5,
        timeout_sec=4.0,
    )

    assert decision["finish"] is True
    assert decision["idle_end"] is True
    assert decision["timeout_end"] is False


def test_timeout_finishes_controlled_capture():
    decision = state_node.should_finish_controlled_capture(
        4.0,
        8,
        0.0,
        min_duration_sec=0.6,
        min_ink_samples=8,
        idle_sec=0.5,
        timeout_sec=4.0,
    )

    assert decision["finish"] is True
    assert decision["timeout_end"] is True


def test_controlled_capture_publishes_compatible_payload_after_idle():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
        controlled_capture_min_duration_sec=0.6,
        controlled_capture_idle_velocity_threshold=0.04,
        controlled_capture_idle_sec=0.5,
        controlled_capture_timeout_sec=4.0,
        controlled_capture_min_ink_samples=4,
    )

    arm_controlled_capture(machine, start=70.0)
    machine.process_sample(sample(71.1, x=0.0, y=0.0))
    machine.process_sample(sample(71.2, x=0.1, y=0.0))
    machine.process_sample(sample(71.8, x=0.1, y=0.0))
    events = machine.process_sample(sample(72.4, x=0.1, y=0.0))

    payloads = captures(events)
    assert len(payloads) == 1
    payload = payloads[0]
    assert payload["capture_mode"] == "controlled_capture"
    assert payload["sample_id"].startswith("m33e_controlled_")
    assert payload["sequence_id"] == 1
    assert len(payload["points"]) >= 4
    assert payload["source_topic"] == "/colmag/trajectory_2d"
    assert payload["idle_end"] is True
    assert payload["timeout_end"] is False
    assert last_ui(events)["state"] == "CONFIRM_PENDING"


def test_controlled_capture_completion_preserves_terminal_event_order():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
        controlled_capture_min_duration_sec=0.6,
        controlled_capture_idle_sec=0.5,
        controlled_capture_min_ink_samples=4,
    )

    arm_controlled_capture(machine, start=75.0)
    machine.process_sample(sample(76.1, x=0.0, y=0.0))
    machine.process_sample(sample(76.2, x=0.1, y=0.0))
    machine.process_sample(sample(76.8, x=0.1, y=0.0))
    events = machine.process_sample(sample(77.4, x=0.1, y=0.0))

    assert [kind for kind, _payload in events] == ["capture", "ui", "ui"]
    assert events[1][1]["state"] == "CONTROLLED_CAPTURE_DONE"
    assert events[2][1]["state"] == "CONFIRM_PENDING"


def test_controlled_capture_timeout_publishes_when_min_samples_passed():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
        controlled_capture_timeout_sec=0.5,
        controlled_capture_min_ink_samples=3,
    )

    arm_controlled_capture(machine, start=80.0)
    machine.process_sample(sample(81.1, x=0.0, y=0.0))
    machine.process_sample(sample(81.2, x=0.1, y=0.0))
    events = machine.process_sample(sample(81.7, x=0.2, y=0.0))

    payloads = captures(events)
    assert len(payloads) == 1
    assert payloads[0]["timeout_end"] is True


def test_x_cancel_clears_controlled_capture_state():
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
    )

    arm_controlled_capture(machine, start=90.0)
    machine.process_sample(sample(91.1, x=0.0, y=0.0))
    assert machine.state == "CONTROLLED_CAPTURING"
    events = machine.process_sample(sample(91.2, x=0.024, y=0.000))

    assert last_ui(events)["state"] == "CANCELLED"
    assert machine.capture_points == []
    assert machine.capture_started_at is None


def test_normalized_points_are_bounded_and_non_empty():
    normalized = state_node.normalize_capture_points([(0.0, 0.0), (2.0, 1.0), (4.0, 0.0)])

    assert normalized
    for x, y in normalized:
        assert -1.0 <= x <= 1.0
        assert -1.0 <= y <= 1.0


def test_dataset_writing_disabled_by_default(tmp_path):
    machine = state_node.MagneticUIStateMachine(
        dwell_sec=1.0,
        controlled_capture_enabled=True,
        controlled_capture_min_duration_sec=0.6,
        controlled_capture_idle_sec=0.5,
        controlled_capture_min_ink_samples=4,
        controlled_capture_dataset_dir=str(tmp_path / "dataset"),
    )

    arm_controlled_capture(machine, start=100.0)
    machine.process_sample(sample(101.1, x=0.0, y=0.0))
    machine.process_sample(sample(101.2, x=0.1, y=0.0))
    machine.process_sample(sample(101.8, x=0.1, y=0.0))
    events = machine.process_sample(sample(102.4, x=0.1, y=0.0))

    assert len(captures(events)) == 1
    assert not (tmp_path / "dataset").exists()


def test_controlled_capture_payload_does_not_create_command_topics():
    decision = {
        "idle_end": True,
        "timeout_end": False,
        "min_ink_samples_passed": True,
    }
    payload = state_node.build_controlled_capture_payload(
        timestamp=2.0,
        sequence_id=7,
        sample_id="sample",
        points=[(0.0, 0.0), (1.0, 1.0)],
        started_at=1.0,
        finish_decision=decision,
    )

    text = str(payload)
    assert "/colmag/confirmed_label" not in text
    assert "/colmag/task_command" not in text
    assert "/colmag/robot_command" not in text
