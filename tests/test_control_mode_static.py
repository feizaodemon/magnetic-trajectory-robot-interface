"""Static and ROS-free contracts for the TASK/TELEOP mode seam."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO / "colmag_ros" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from colmag_ros import control_mode
from dashboard_harness import DashboardHarness
from launch_xml import arg_defaults, arg_values, param_values


SCRIPTS = REPO / "colmag_ros" / "scripts"
LAUNCH = REPO / "colmag_ros" / "launch"
DISPATCHER = SCRIPTS / "task_dispatcher_node.py"
MODE_NODE = SCRIPTS / "control_mode_node.py"
FINAL_GAZEBO = LAUNCH / "final_gazebo_robot_demo.launch"
REAL_PROFILE = LAUNCH / "colmag_real_board_fr3_cartesian_teleop.launch"
MOUSE_FRONTEND = LAUNCH / "dtw_mouse_demo_frontend.launch"
TASK_ADAPTER = REPO / "colmag_ros" / "src" / "colmag_fr3_task_trajectory_adapter.cpp"
GAZEBO_BRIDGE = (
    REPO
    / "colmag_gazebo_stub"
    / "scripts"
    / "fr3_gazebo_visible_task_bridge_node.py"
)


def test_mode_contract_has_exactly_two_values_and_fails_closed_to_task():
    assert control_mode.SUPPORTED_MODES == {"TASK", "TELEOP"}
    assert control_mode.startup_mode() == "TASK"
    assert control_mode.startup_mode("invalid") == "TASK"
    assert control_mode.normalize_mode(" teleop ") == "TELEOP"
    assert control_mode.normalize_mode("robot") is None


def test_mode_admission_is_observed_and_mutually_exclusive():
    assert control_mode.mode_allows("TASK", "TASK", True)
    assert not control_mode.mode_allows("TASK", "TELEOP", True)
    assert not control_mode.mode_allows("TASK", "TASK", False)
    assert control_mode.mode_allows("TELEOP", "TELEOP", True)
    assert not control_mode.mode_allows("TELEOP", "TASK", True)
    assert control_mode.mode_allows("", None, False)


def test_mode_node_is_latched_and_rejects_invalid_requests():
    text = MODE_NODE.read_text()
    assert "latch=True" in text
    assert "control_mode.startup_mode" in text
    assert "control_mode.normalize_mode(message.data)" in text
    assert "Rejected invalid control mode request" in text
    assert "allow_real_robot" not in text
    assert "send_commands" not in text


def test_dashboard_contains_only_small_mode_request_and_status_wiring(monkeypatch):
    disabled = DashboardHarness(monkeypatch)
    assert "/colmag/control_mode/request" not in disabled.publisher_topics()
    assert "/colmag/control_mode" not in disabled.rospy.subscribers

    harness = DashboardHarness(monkeypatch, control_mode_enabled=True)
    node = harness.node
    request_topic = "/colmag/control_mode/request"

    def shown():
        assert node.control_mode_var in harness.shown_variables()
        assert node.control_mode_button_var in harness.shown_variables()
        return node.control_mode_var.get(), node.control_mode_button_var.get()

    def click():
        node.control_mode_button.options["command"]()

    # The mode request publisher is the only one this seam adds.
    assert harness.publisher_topics() == [request_topic, "/colmag/symbol_capture"]
    assert shown() == ("Mode: TASK", "Switch to TELEOP")
    click()  # Before any mode is observed, the safe request is TASK.
    harness.send("/colmag/control_mode", control_mode.TASK)
    click()
    assert shown() == ("Mode: TASK", "Switch to TELEOP")
    harness.send("/colmag/control_mode", control_mode.TELEOP)
    assert shown() == ("Mode: TELEOP", "Switch to TASK")
    click()
    harness.send("/colmag/control_mode", "bogus")
    assert shown() == ("Mode: INVALID — blocked", "Request TASK")
    click()

    assert harness.messages(request_topic) == [
        control_mode.TASK, control_mode.TELEOP, control_mode.TASK, control_mode.TASK]


def test_final_gazebo_profile_owns_one_mode_source_and_both_capabilities():
    root = ET.parse(FINAL_GAZEBO).getroot()
    assert arg_defaults(root)["initial_control_mode"] == "TASK"
    mode_nodes = [node for node in root.findall("node") if node.attrib.get("type") == "control_mode_node.py"]
    assert len(mode_nodes) == 1

    frontend = next(
        include for include in root.findall("include")
        if include.attrib["file"].endswith("/dtw_mouse_demo_frontend.launch")
    )
    frontend_args = arg_values(frontend)
    assert frontend_args["required_control_mode"] == "TASK"
    assert frontend_args["control_mode_enabled"] == "true"
    assert frontend_args["latch_task_command"] == "false"

    manual_tasks = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "keyboard_manual_task_dispatcher"
    )
    assert param_values(manual_tasks)["required_control_mode"] == "TASK"
    assert param_values(manual_tasks)["latch_task_command"] == "false"

    continuous = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "colmag_gazebo_cartesian_teleop_adapter"
    )
    continuous_params = param_values(continuous)
    assert continuous_params["input_mode"] == "normalized_cartesian"
    assert continuous_params["control_mode_topic"] == "$(arg control_mode_topic)"
    assert continuous_params["simulation_commands_enabled"] == "true"

    bridge = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "fr3_gazebo_visible_task_bridge_node"
    )
    assert param_values(bridge)["required_control_mode"] == "TASK"


def test_real_profile_owns_one_mode_source_and_both_capabilities():
    root = ET.parse(REAL_PROFILE).getroot()
    assert arg_defaults(root)["initial_mode"] == "TASK"
    mode_nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "control_mode_node.py"
    ]
    assert len(mode_nodes) == 1

    frontend = next(
        include for include in root.findall("include")
        if include.attrib["file"].endswith("/dtw_real_board_demo_frontend.launch")
    )
    frontend_args = arg_values(frontend)
    assert frontend_args["required_control_mode"] == "TASK"
    assert frontend_args["control_mode_enabled"] == "true"
    assert frontend_args["latch_task_command"] == "false"

    discrete = next(
        include for include in root.findall("include")
        if include.attrib["file"].endswith(
            "/colmag_fr3_task_trajectory_adapter.launch"
        )
    )
    discrete_args = arg_values(discrete)
    assert discrete_args["required_control_mode"] == "TASK"
    assert discrete_args["control_mode_topic"] == "$(arg control_mode_topic)"

    continuous = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "colmag_fr3_cartesian_teleop_adapter"
    )
    continuous_params = param_values(continuous)
    assert continuous_params["input_mode"] == "magnetic_board"
    assert continuous_params["simulation_commands_enabled"] == "false"


def test_discrete_owners_cancel_only_their_active_goal_when_task_loses_mode():
    task_source = TASK_ADAPTER.read_text()
    bridge_source = GAZEBO_BRIDGE.read_text()
    assert "relinquishOwnedGoalForMode" in task_source
    assert 'cancelOwnedGoal(transition, "control_mode_transition")' in task_source
    assert "cancelAllGoals" not in task_source
    assert "_handle_control_mode" in bridge_source
    assert "self.client.cancel_goal()" in bridge_source
    assert "cancel_all_goals" not in bridge_source


def test_task_dispatcher_keeps_legacy_compatibility_but_gates_managed_routes():
    text = DISPATCHER.read_text()
    assert 'rospy.get_param("~required_control_mode", "")' in text
    assert "control_mode.mode_allows" in text
    assert "task command admission blocked" in text
    root = ET.parse(MOUSE_FRONTEND).getroot()
    dispatcher = next(
        node for node in root.findall("group/node")
        if node.attrib.get("type") == "task_dispatcher_node.py"
    )
    params = param_values(dispatcher)
    assert params["required_control_mode"] == "$(arg required_control_mode)"
    assert params["control_mode_topic"] == "$(arg control_mode_topic)"
