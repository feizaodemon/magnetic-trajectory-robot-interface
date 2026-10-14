"""Focused static/unit contracts for selectable keyboard teleop backends."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO / "colmag_ros" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from colmag_ros import cartesian_keyboard_input, keyboard_teleop
from launch_xml import arg_defaults, arg_values, param_values


TELEOP_LAUNCH = REPO / "colmag_ros" / "launch" / "colmag_keyboard_gazebo_teleop.launch"
REAL_TELEOP_LAUNCH = REPO / "colmag_ros" / "launch" / "colmag_keyboard_fr3_teleop.launch"
FINAL_GAZEBO_LAUNCH = REPO / "colmag_ros" / "launch" / "final_gazebo_robot_demo.launch"
HARDWARE_STACK_LAUNCH = REPO / "colmag_ros" / "launch" / "colmag_fr3_hardware_stack.launch"
ADAPTER_LAUNCH = (
    REPO
    / "colmag_ros"
    / "launch"
    / "colmag_fr3_task_trajectory_adapter.launch"
)
ADAPTER_SOURCE = REPO / "colmag_ros" / "src" / "colmag_fr3_task_trajectory_adapter.cpp"
TELEOP_SCRIPT = REPO / "colmag_ros" / "scripts" / "keyboard_teleop_node.py"
CARTESIAN_SCRIPT = (
    REPO / "colmag_ros" / "scripts" / "keyboard_cartesian_teleop_node.py"
)
CARTESIAN_ADAPTER = (
    REPO / "colmag_ros" / "src" / "colmag_fr3_cartesian_teleop_adapter.cpp"
)


def test_command_mapping_reuses_the_canonical_c4_contract():
    expected = {
        "left": ("1", "MOVE_LEFT"),
        "hex": ("2", "HEXAGON_TRAJECTORY"),
        "right": ("3", "MOVE_RIGHT"),
        "hover": ("V", "HOVER_APPROACH"),
        "orbit": ("O", "ORBIT_SMALL"),
        "stop": ("X", "STOP_OR_CANCEL"),
        "home": ("A", "HOME_OR_READY"),
        "down": ("C", "SOFT_DESCEND_PREVIEW"),
    }
    for command, (label, task) in expected.items():
        assert keyboard_teleop.normalize_command(command) == label
        assert keyboard_teleop.task_for_command(command) == task


def test_confirmed_payload_is_backend_independent():
    payload = keyboard_teleop.build_confirmed_label_payload(
        "left", sequence_id=7, timestamp=123.5
    )
    assert payload["label"] == "1"
    assert payload["confirmed"] is True
    assert payload["confidence"] == 1.0
    assert payload["sequence_id"] == 7
    assert payload["confirmed_by"] == "terminal_keyboard_teleop"
    assert "gazebo_only" not in payload
    assert "controls_real_robot" not in payload


def test_unknown_commands_do_not_create_confirmed_payloads():
    assert keyboard_teleop.normalize_command("pick") is None
    assert keyboard_teleop.task_for_command("pick") is None
    try:
        keyboard_teleop.build_confirmed_label_payload("pick", sequence_id=1)
    except ValueError as exc:
        assert "unknown keyboard teleop command" in str(exc)
    else:
        raise AssertionError("unknown command unexpectedly produced a payload")


def test_continuous_keyboard_mapping_is_normalized_cartesian_not_task_labels():
    expected = {
        "w": (1.0, 0.0, 0.0),
        "s": (-1.0, 0.0, 0.0),
        "a": (0.0, 1.0, 0.0),
        "d": (0.0, -1.0, 0.0),
        "q": (0.0, 0.0, 1.0),
        "e": (0.0, 0.0, -1.0),
    }
    for key, direction in expected.items():
        assert cartesian_keyboard_input.direction_for_key(key) == direction
        payload = cartesian_keyboard_input.build_cartesian_input_payload(
            key, sequence_id=4, timestamp=12.0
        )
        assert (payload["x"], payload["y"], payload["z"]) == direction
        assert payload["engaged"] is True
        assert "task" not in payload
        assert "label" not in payload
    assert cartesian_keyboard_input.build_cartesian_input_payload(
        " ", sequence_id=5
    )["engaged"] is False
    release = cartesian_keyboard_input.build_release_payload(sequence_id=6, timestamp=13.0)
    assert (release["x"], release["y"], release["z"]) == (0.0, 0.0, 0.0)
    assert release["engaged"] is True


def test_gripper_keys_build_discrete_open_close_commands():
    assert cartesian_keyboard_input.gripper_command_for_key("o") == "open"
    assert cartesian_keyboard_input.gripper_command_for_key("C") == "close"
    assert cartesian_keyboard_input.gripper_command_for_key("\x1b[C") is None
    assert cartesian_keyboard_input.gripper_command_for_key("w") is None
    payload = cartesian_keyboard_input.build_gripper_payload(
        "close", sequence_id=2, timestamp=5.0)
    assert payload == {"timestamp": 5.0, "sequence_id": 2, "command": "close",
                       "input_mode": "keyboard_gripper"}
    with pytest.raises(ValueError):
        cartesian_keyboard_input.build_gripper_payload("grip", sequence_id=3)


def test_key_hold_tracker_bridges_the_first_repeat_delay_and_detects_release():
    tracker = cartesian_keyboard_input.KeyHoldTracker(
        first_repeat_window_sec=0.6, repeat_window_sec=0.12
    )
    hold, release = cartesian_keyboard_input.HOLD, cartesian_keyboard_input.RELEASE
    assert tracker.poll(0.0) is None
    assert tracker.press("w", 0.0) is True
    # The terminal's first repeat arrives about 0.5 s after the press.
    assert tracker.poll(0.5) == hold
    assert tracker.press("w", 0.5) is False
    assert tracker.poll(0.55) == hold
    assert tracker.press("w", 0.54) is False
    # Repeats stop: released one repeat window after the last one.
    assert tracker.poll(0.65) == hold
    assert tracker.poll(0.67) == release
    assert tracker.poll(0.7) is None
    # A different key starts a new hold at once; clear() drops it silently.
    assert tracker.press("w", 1.0) is True
    assert tracker.press("a", 1.1) is True
    assert tracker.key == "a"
    tracker.clear()
    assert tracker.poll(1.2) is None


def test_launch_wraps_the_dual_mode_existing_gazebo_backend():
    root = ET.parse(TELEOP_LAUNCH).getroot()
    include = root.find("include")
    assert include is not None
    assert include.attrib["file"].endswith("/final_gazebo_robot_demo.launch")
    include_args = arg_values(include)
    assert include_args["start_dtw_frontend"] == "true"
    assert include_args["start_cartesian_teleop"] == "true"
    assert include_args["start_keyboard_teleop"] == "false"
    assert include_args["task_command_topic"] == "$(arg task_command_topic)"
    assert include_args["keyboard_confirmed_topic"] == "$(arg confirmed_topic)"
    # The only extra node is the Gazebo-only, TELEOP-gated gripper bridge.
    assert [node.attrib["type"] for node in root.findall("node")] == [
        "gazebo_gripper_bridge_node.py"
    ]


def test_existing_final_route_keeps_its_frontend_enabled_by_default():
    root = ET.parse(FINAL_GAZEBO_LAUNCH).getroot()
    args = arg_defaults(root)
    assert args["start_dtw_frontend"] == "true"
    frontend = next(
        include
        for include in root.findall("include")
        if include.attrib.get("file", "").endswith("/dtw_mouse_demo_frontend.launch")
    )
    assert frontend.attrib["if"] == "$(arg start_dtw_frontend)"
    frontend_args = arg_values(frontend)
    assert frontend_args["required_control_mode"] == "TASK"
    manual_tasks = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "keyboard_manual_task_dispatcher"
    )
    params = param_values(manual_tasks)
    assert params["confirmed_label_topic"] == "$(arg keyboard_confirmed_topic)"
    assert params["task_command_topic"] == "$(arg task_command_topic)"
    assert params["gazebo_only"] == "true"
    assert params["required_control_mode"] == "TASK"

    continuous = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "colmag_gazebo_cartesian_teleop_adapter"
    )
    continuous_params = param_values(continuous)
    assert continuous_params["input_mode"] == "normalized_cartesian"
    assert continuous_params["normalized_input_topic"] == "$(arg cartesian_input_topic)"
    assert continuous_params["joint_state_topic"] == "/joint_states"
    assert continuous_params["command_topic"] == "/position_joint_trajectory_controller/command"
    assert continuous_params["simulation_commands_enabled"] == "true"
    assert continuous_params["send_commands"] == "false"
    assert "task_command_topic" not in continuous_params


def test_real_launch_requires_explicit_hardware_opt_in_and_uses_isolated_topics():
    root = ET.parse(REAL_TELEOP_LAUNCH).getroot()
    args = arg_defaults(root)
    assert args["robot_ip"] is None
    assert args["allow_real_robot"] == "false"
    assert args["send_goals"] == "false"
    assert args["hardware_profile"] == "dry-run"
    assert args["confirmed_topic"] == "/colmag/teleop/confirmed_label"
    assert args["task_command_topic"] == "/colmag/teleop/task_command"

    node = next(
        candidate for candidate in root.findall("node")
        if candidate.attrib.get("type") == "task_dispatcher_node.py"
    )
    params = param_values(node)
    assert node.attrib["type"] == "task_dispatcher_node.py"
    assert params["confirmed_label_topic"] == "$(arg confirmed_topic)"
    assert params["task_command_topic"] == "$(arg task_command_topic)"
    assert params["gazebo_only"] == "false"
    assert params["required_control_mode"] == "TELEOP"

    include = root.find("include")
    include_args = arg_values(include)
    assert include.attrib["file"].endswith("/colmag_fr3_hardware_stack.launch")
    assert include_args["command_topic"] == "$(arg task_command_topic)"
    assert include_args["hardware_execution_enabled"] == "$(arg allow_real_robot)"
    assert include_args["send_goals"] == "$(arg send_goals)"
    assert include_args["hardware_profile"] == "$(arg hardware_profile)"
    assert "gazebo" not in include.attrib["file"].lower()


def test_hardware_stack_routes_the_selected_command_topic_to_strategy_a():
    root = ET.parse(HARDWARE_STACK_LAUNCH).getroot()
    args = arg_defaults(root)
    assert args["command_topic"] == "/colmag/task_command"
    adapter_include = next(
        include
        for include in root.findall("include")
        if include.attrib.get("file", "").endswith(
            "/colmag_fr3_task_trajectory_adapter.launch"
        )
    )
    include_args = arg_values(adapter_include)
    assert include_args["command_topic"] == "$(arg command_topic)"


def test_real_backend_contract_is_concrete_and_fails_closed():
    launch = ADAPTER_LAUNCH.read_text(encoding="utf-8")
    source = ADAPTER_SOURCE.read_text(encoding="utf-8")
    for task in ("MOVE_LEFT", "HEXAGON_TRAJECTORY", "MOVE_RIGHT", "STOP_OR_CANCEL"):
        assert task in launch
    for unsupported in (
        "HOVER_APPROACH",
        "ORBIT_SMALL",
        "HOME_OR_READY",
        "SOFT_DESCEND_PREVIEW",
    ):
        assert unsupported not in launch
    for contract in (
        "FollowJointTrajectoryAction",
        "joint_state_topic",
        "waitForServer",
        "action_server_unavailable",
        "validateRuntimeParameters",
        "unsupported_task_no_real_goal",
        "cancelGoal",
    ):
        assert contract in source
    assert "cancelAllGoals" not in source


def test_keyboard_nodes_are_installed_and_continuous_source_has_no_task_route():
    cmake = (REPO / "colmag_ros" / "CMakeLists.txt").read_text(encoding="utf-8")
    script = TELEOP_SCRIPT.read_text(encoding="utf-8")
    cartesian_script = CARTESIAN_SCRIPT.read_text(encoding="utf-8")
    assert "scripts/keyboard_teleop_node.py" in cmake
    assert "scripts/keyboard_cartesian_teleop_node.py" in cmake
    assert "/colmag/teleop/confirmed_label" in script
    assert "task_command" not in script
    assert "/colmag/teleop/cartesian_input" in cartesian_script
    assert "task_dispatcher" not in cartesian_script
    assert "task_command" not in cartesian_script


def test_gazebo_continuous_route_uses_the_same_cartesian_core_as_real_board():
    source = CARTESIAN_ADAPTER.read_text()
    assert "Fr3CartesianTeleopCore" in source
    assert "stepNormalizedCartesian" in source
    assert "core_->step(latest_sample_" in source
    assert "task_dispatcher_node" not in source
    core_sources = sorted(
        path.name
        for path in (REPO / "colmag_ros").rglob("*cartesian_teleop_core*")
        if path.is_file()
    )
    assert core_sources == ["fr3_cartesian_teleop_core.cpp", "fr3_cartesian_teleop_core.h"]


def test_package_docs_explain_both_backends_and_hardware_boundary():
    docs = (REPO / "colmag_ros" / "README.md").read_text(encoding="utf-8")
    assert "roslaunch colmag_ros colmag_keyboard_gazebo_teleop.launch" in docs
    assert "rosrun colmag_ros keyboard_cartesian_teleop_node.py" in docs
    assert "rosrun colmag_ros keyboard_teleop_node.py" in docs
    assert "/colmag/teleop/confirmed_label" in docs
    assert "/colmag/task_command" in docs
    assert "TASK and TELEOP control modes" in docs
    assert "never changes a hardware gate" in docs
    assert "colmag_keyboard_fr3_teleop.launch" in docs
    assert "colmag_real_board_fr3_cartesian_teleop.launch" in docs
    assert "allow_real_robot=true" in docs
    assert "calibration_valid=true" in docs
    assert "predefined-task diagnostic" in docs
