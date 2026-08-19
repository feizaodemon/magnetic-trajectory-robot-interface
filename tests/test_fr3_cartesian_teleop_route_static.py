"""Focused source/launch safety contracts for continuous Real-FR3 TELEOP."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[1]
COLMAG = REPO / "colmag_ros"
LAUNCH = COLMAG / "launch" / "colmag_real_board_fr3_cartesian_teleop.launch"
CONFIG = COLMAG / "config" / "colmag_fr3_cartesian_teleop.yaml"
CORE_HEADER = COLMAG / "include" / "colmag_ros" / "fr3_cartesian_teleop_core.h"
CORE_SOURCE = COLMAG / "src" / "fr3_cartesian_teleop_core.cpp"
ADAPTER = COLMAG / "src" / "colmag_fr3_cartesian_teleop_adapter.cpp"
TASK_ADAPTER = COLMAG / "src" / "colmag_fr3_task_trajectory_adapter.cpp"
CMAKE = COLMAG / "CMakeLists.txt"


def _args(root):
    return {item.attrib["name"]: item.attrib.get("default") for item in root.findall("arg")}


def _params(node):
    return {item.attrib["name"]: item.attrib.get("value") for item in node.findall("param")}


def test_launch_defaults_open_no_hardware_or_serial_path():
    root = ET.parse(LAUNCH).getroot()
    args = _args(root)
    assert args["initial_mode"] == "TASK"
    assert args["board_source"] == "none"
    assert args["start_fr3_hardware"] == "false"
    assert args["allow_real_robot"] == "false"
    assert args["send_goals"] == "false"
    assert args["send_commands"] == "false"
    assert args["hardware_profile"] == "dry-run"
    assert args["calibration_valid"] == "false"


def test_real_profile_contains_board_acquisition_task_and_continuous_paths():
    text = LAUNCH.read_text()
    for required in (
        "dtw_real_board_demo_frontend.launch",
        "/colmag/trajectory_2d",
        "colmag_fr3_hardware_bringup.launch",
        "m104r1c_fr3_task_trajectory_adapter_static_entrypoint.launch",
        "colmag_fr3_cartesian_teleop_adapter",
        "required_control_mode\" value=\"TASK",
        "/$(arg controller_name)/command",
    ):
        assert required in text


def test_route_has_exactly_one_continuous_controller_command_producer():
    root = ET.parse(LAUNCH).getroot()
    adapters = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "colmag_fr3_cartesian_teleop_adapter"
    ]
    assert len(adapters) == 1
    params = _params(adapters[0])
    assert params["command_topic"] == "$(arg command_topic)"
    assert params["input_mode"] == "magnetic_board"
    assert params["simulation_commands_enabled"] == "false"
    assert params["allow_real_robot"] == "$(arg allow_real_robot)"
    assert params["send_commands"] == "$(arg send_commands)"
    assert params["hardware_bringup_enabled"] == "$(arg start_fr3_hardware)"
    assert params["hardware_profile"] == "$(arg hardware_profile)"
    assert params["calibration_valid"] == "$(arg calibration_valid)"
    assert "task_command_topic" not in params


def test_adapter_fails_closed_on_mode_freshness_clutch_and_hardware_gates():
    text = ADAPTER.read_text()
    for contract in (
        'control_mode_ != kTeleopMode',
        '"new_input_required"',
        '"new_joint_state_required"',
        '"stale_magnetic_input"',
        '"stale_joint_state"',
        '"interaction_not_engaged"',
        "hardware_bringup_enabled_ && allow_real_robot_ && send_commands_",
        'hardware_profile_ == kActiveHardwareProfile',
        'if (!commandGateOpen())',
        'if (!result.accepted)',
    ):
        assert contract in text
    assert text.index('if (!commandGateOpen())') < text.index('command_publisher_.publish(command)')
    assert "joint_after_mode_change_ = false" in text
    assert "joint_after_mode_change_ = true" in text


def test_core_contains_bounded_mapping_s_curve_fk_jacobian_and_dls():
    header = CORE_HEADER.read_text()
    source = CORE_SOURCE.read_text()
    for contract in (
        "workspace_min",
        "workspace_max",
        "max_linear_speed_m_s",
        "max_joint_step_rad",
        "NormalizedCartesianInput",
        "joint_lower",
        "joint_upper",
    ):
        assert contract in header
    for contract in (
        "forwardKinematics",
        "numericalJacobian",
        "solveIk",
        "dls_damping",
        "sCurveStep",
        "ik_residual_exceeded",
        "non_finite_joint_target",
        "stepNormalizedCartesian",
        "stepPreparedTarget",
    ):
        assert contract in source


def test_config_is_focused_and_marks_calibration_as_external_gate():
    config = yaml.safe_load(CONFIG.read_text())
    assert config["workspace_min"] == [0.30, -0.30, 0.121]
    assert config["workspace_max"] == [0.60, 0.30, 0.680]
    assert config["input_freshness_timeout_sec"] > 0.0
    assert config["joint_state_freshness_timeout_sec"] > 0.0
    assert config["dls_damping"] > 0.0
    assert len(config["joint_lower"]) == 7
    assert len(config["joint_upper"]) == 7
    assert "calibration_valid" in CONFIG.read_text()


def test_build_metadata_registers_core_and_adapter_without_new_package_dependency():
    text = CMAKE.read_text()
    assert "add_library(colmag_fr3_cartesian_teleop_core" in text
    assert "add_executable(colmag_fr3_cartesian_teleop_adapter" in text
    assert "colmag_fr3_cartesian_teleop_core" in text
    assert "colmag_fr3_cartesian_teleop_adapter" in text


def test_task_owner_is_mode_gated_and_uses_owned_action_cancellation():
    text = TASK_ADAPTER.read_text()
    for contract in (
        "required_control_mode_",
        "taskModeAllowsAdmission",
        "relinquishOwnedGoalForMode",
        'cancelOwnedGoal(transition, "control_mode_transition")',
        '"task_mode_not_selected"',
    ):
        assert contract in text
    assert "cancelAllGoals" not in text
