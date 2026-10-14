from types import SimpleNamespace

import pytest

import task_dispatcher_node as dispatcher


def confirmed(label, confidence=0.9):
    return {"label": label, "confirmed": True, "confidence": confidence}


def test_task_dispatcher_maps_core_magnetic_labels():
    assert dispatcher.build_task_command(confirmed("A"))["task"] == "HOME_OR_READY"
    assert dispatcher.build_task_command(confirmed("2"))["task"] == "HEXAGON_TRAJECTORY"
    assert dispatcher.build_task_command(confirmed("C"))["task"] == "SOFT_DESCEND_PREVIEW"
    assert dispatcher.build_task_command(confirmed("X"))["task"] == "STOP_OR_CANCEL"


def test_task_dispatcher_maps_current_direction_labels():
    assert dispatcher.build_task_command(confirmed("1"))["task"] == "MOVE_LEFT"
    assert dispatcher.build_task_command(confirmed("3"))["task"] == "MOVE_RIGHT"
    assert dispatcher.build_task_command(confirmed("V"))["task"] == "HOVER_APPROACH"
    assert dispatcher.build_task_command(confirmed("O"))["task"] == "ORBIT_SMALL"


def test_parse_mapping_accepts_json_object_or_dict():
    assert dispatcher.parse_mapping('{"a": "HOME_OR_READY", "1": "MOVE_LEFT"}') == {
        "A": "HOME_OR_READY",
        "1": "MOVE_LEFT",
    }
    assert dispatcher.parse_mapping({"x": "STOP_OR_CANCEL"}) == {"X": "STOP_OR_CANCEL"}


@pytest.mark.parametrize("bad", ['{"A": "HOME_OR_READY"', "A=HOME", '["A"]', "42", 42, None])
def test_parse_mapping_rejects_invalid_value_instead_of_using_default(bad):
    with pytest.raises(ValueError, match="label_to_task_json"):
        dispatcher.parse_mapping(bad)


def test_mapping_name_selects_c4_mapping():
    assert dispatcher.label_to_task_mapping_for_name("m104c4_8symbol_gazebo") == (
        dispatcher.C4_LABEL_TO_TASK
    )


@pytest.mark.parametrize("bad", ["m104c4_8symbol_gazebo ", "m104c4_8symbol", "", None])
def test_unknown_mapping_name_is_rejected_instead_of_using_c4(bad):
    with pytest.raises(ValueError, match="dispatcher_mapping_name"):
        dispatcher.label_to_task_mapping_for_name(bad)
    with pytest.raises(ValueError, match="dispatcher_mapping_name"):
        dispatcher.build_task_command(confirmed("A"), dispatcher_mapping_name=bad)


def test_explicit_empty_mapping_dispatches_nothing():
    command = dispatcher.build_task_command(confirmed("A"), label_to_task={})
    assert command["task"] == "NO_OP"
    assert command["accepted"] is False
    assert command["reason"] == "unknown_label"


def test_unknown_or_unconfirmed_label_maps_to_no_op():
    unknown = dispatcher.build_task_command(confirmed("Z"))
    assert unknown["task"] == "NO_OP"
    assert unknown["accepted"] is False
    assert unknown["reason"] == "unknown_label"

    rejected = dispatcher.build_task_command({"label": "A", "confirmed": False, "confidence": 0.9})
    assert rejected["task"] == "NO_OP"
    assert rejected["reason"] == "confirmed_label_false"


def test_low_confidence_label_maps_to_no_op():
    command = dispatcher.build_task_command(confirmed("A", confidence=0.1), min_confidence=0.25)

    assert command["task"] == "NO_OP"
    assert command["accepted"] is False
    assert command["reason"] == "confidence_below_threshold"
    assert command["controls_real_robot"] is False


def test_task_command_adds_compatible_ros_time_provenance_metadata():
    command = dispatcher.build_task_command(
        confirmed("1"),
        command_id="dispatcher-test-1",
        issued_at=123.5,
        source_id="test-dispatcher",
        schema_version=1,
        target_adapter_session_id="adapter-session-new",
    )
    assert command["command_id"] == "dispatcher-test-1"
    assert command["issued_at"] == 123.5
    assert command["timestamp"] == 123.5
    assert command["source_id"] == "test-dispatcher"
    assert command["schema_version"] == 1
    assert command["target_adapter_session_id"] == "adapter-session-new"
    assert command["upstream_controls_real_robot"] is False
    assert command["controls_real_robot"] is False


def test_task_command_ids_are_unique_by_default():
    first = dispatcher.build_task_command(confirmed("1"))
    second = dispatcher.build_task_command(confirmed("1"))
    assert first["command_id"] != second["command_id"]


def test_legacy_dry_run_command_has_explicit_empty_session_binding():
    command = dispatcher.build_task_command(confirmed("1"), target_adapter_session_id="")
    assert command["target_adapter_session_id"] == ""


def test_dispatcher_tracks_latest_latched_adapter_session_for_next_confirmation():
    node = dispatcher.TaskDispatcherNode.__new__(dispatcher.TaskDispatcherNode)
    node.adapter_session_id = ""
    node._handle_adapter_session(SimpleNamespace(data=" session-new \n"))
    command = dispatcher.build_task_command(
        confirmed("1"), target_adapter_session_id=node.adapter_session_id
    )
    assert command["target_adapter_session_id"] == "session-new"
