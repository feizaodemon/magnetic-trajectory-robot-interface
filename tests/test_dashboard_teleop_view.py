"""TELEOP view model: adapter state and input as card text and pad keys."""

from colmag_ros.dashboard_teleop_view import (
    NO_STATE_VIEW,
    direction_label,
    gripper_view,
    input_direction,
    pad_keys,
    teleop_backend_text,
    teleop_state_view,
)


def test_input_direction_needs_an_engaged_nonzero_vector():
    assert input_direction({"engaged": True, "x": 0.0, "y": -1.0, "z": 0.0}) == (0.0, -1.0, 0.0)
    assert input_direction({"engaged": False, "x": 1.0}) is None
    assert input_direction({"engaged": True, "x": 0.0, "y": 0.0, "z": 0.0}) is None
    assert input_direction({"engaged": True, "x": "fast"}) is None
    assert input_direction(None) is None


def test_direction_labels_and_pad_keys_follow_the_keyboard_mapping():
    assert direction_label((1.0, 0.0, -1.0)) == "+X -Z"
    assert direction_label(None) == ""
    keys = {letter: label for letter, _vector, label in pad_keys()}
    assert keys == {"W": "+X", "S": "-X", "A": "+Y", "D": "-Y", "Q": "+Z", "E": "-Z",
                    "O": "Open", "C": "Close"}


def test_gripper_view_text_and_lit_key():
    assert gripper_view(None) == ("", None)
    assert gripper_view({"state": "CLOSING", "width_m": 0.0712}) == (
        "Gripper: Closing · 71 mm", "close")
    assert gripper_view({"state": "HOLDING", "width_m": 0.048}) == (
        "Gripper: Holding an object · 48 mm", None)
    assert gripper_view({"state": "REJECTED", "reason": "teleop_not_owner", "width_m": None}) == (
        "Gripper: Command rejected · teleop not owner", None)


def test_state_view_texts_and_tones():
    assert teleop_state_view(None) == NO_STATE_VIEW
    assert teleop_state_view({"state": "COMMAND_PUBLISHED"}, (0.0, 1.0, 0.0)) == (
        "Moving +Y", "Bounded Cartesian target sent", "success")
    assert teleop_state_view({"state": "STOPPING", "reason": "stale_cartesian_input"}) == (
        "Stopping", "Key released, braking to rest", "neutral")
    assert teleop_state_view(
        {"state": "WAITING_FOR_INPUT", "input_mode": "normalized_cartesian"}) == (
        "Waiting for input", "Hold a direction key in the teleop terminal", "neutral")
    assert teleop_state_view({"state": "PAUSED", "reason": "stale_cartesian_input"}) == (
        "Idle", "Hold a direction key in the teleop terminal", "neutral")
    assert teleop_state_view({"state": "PAUSED", "reason": "stale_joint_state"}) == (
        "Paused", "Stale joint state", "warning")
    assert teleop_state_view({"state": "REJECTED", "reason": "workspace_limit"}) == (
        "Target rejected", "Workspace limit", "danger")
    assert teleop_state_view({"state": "NEW_STATE", "reason": ""}) == ("New state", "", "neutral")


def test_backend_text_reports_the_open_gate():
    assert teleop_backend_text({"hardware_active": True}) == "Hardware command gate open"
    assert teleop_backend_text({"simulation_commands_enabled": True}) == (
        "Simulation commands · hardware inactive")
    assert teleop_backend_text({}) == "Commands disabled"
    assert teleop_backend_text(None) == ""
