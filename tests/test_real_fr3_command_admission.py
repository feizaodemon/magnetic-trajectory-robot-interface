import xml.etree.ElementTree as ET
from pathlib import Path

from cpp_harness import compile_and_run
from launch_xml import arg_defaults


REPO = Path(__file__).resolve().parents[1]
IMPLEMENTATION = REPO / "colmag_ros" / "src" / "real_fr3_command_admission.cpp"
INCLUDE_DIR = REPO / "colmag_ros" / "include"
HEADER = INCLUDE_DIR / "colmag_ros" / "real_fr3_command_admission.h"
STRATEGY_A = REPO / "colmag_ros" / "src" / "colmag_fr3_task_trajectory_adapter.cpp"
CMAKE = REPO / "colmag_ros" / "CMakeLists.txt"
MOUSE_DRY_RUN = REPO / "colmag_ros" / "launch" / "colmag_mouse_fr3_adapter_dry_run.launch"
BOARD_DRY_RUN = REPO / "colmag_ros" / "launch" / "colmag_real_board_fr3_adapter_dry_run.launch"
ADAPTER_ENTRYPOINT = (
    REPO
    / "colmag_ros"
    / "launch"
    / "colmag_fr3_task_trajectory_adapter.launch"
)


def test_real_fr3_command_admission_interface_behavior(tmp_path):
    compile_and_run(tmp_path, "real_fr3_command_admission_test", "real_fr3_command_admission.cpp")


def test_strategy_a_delegates_only_common_admission_policy():
    strategy_a = STRATEGY_A.read_text()
    assert '#include "colmag_ros/real_fr3_command_admission.h"' in strategy_a
    assert strategy_a.count("evaluateRealFr3CommandAdmission(") == 1
    assert "admission.allowed" in strategy_a
    assert "RealFr3AdmissionInput" in strategy_a
    assert "RealFr3AdmissionPolicy" in strategy_a
    assert "if (require_confirmed_ &&" not in strategy_a
    assert "if (require_accepted_ &&" not in strategy_a
    assert "sendMotionGoal" in strategy_a
    assert "waitForServer" in strategy_a
    assert "send_goals_false" in strategy_a


def test_policy_is_ros_free_and_installed_as_one_canonical_library():
    policy_text = HEADER.read_text() + IMPLEMENTATION.read_text()
    assert "ros::" not in policy_text
    assert "NodeHandle" not in policy_text
    assert "Publisher" not in policy_text
    assert "SimpleActionClient" not in policy_text

    cmake_text = CMAKE.read_text()
    assert "add_library(colmag_real_fr3_command_admission" in cmake_text
    assert cmake_text.count("src/real_fr3_command_admission.cpp") == 1
    assert "LIBRARIES colmag_real_fr3_command_admission" in cmake_text
    assert "install(DIRECTORY include/${PROJECT_NAME}/" in cmake_text


def test_official_fr3_dry_run_defaults_remain_unchanged():
    mouse = arg_defaults(ET.parse(MOUSE_DRY_RUN).getroot())
    board = arg_defaults(ET.parse(BOARD_DRY_RUN).getroot())
    entrypoint = arg_defaults(ET.parse(ADAPTER_ENTRYPOINT).getroot())

    expected_joint_names = "$(arg arm_id)_joint1, $(arg arm_id)_joint2, $(arg arm_id)_joint3, $(arg arm_id)_joint4, $(arg arm_id)_joint5, $(arg arm_id)_joint6, $(arg arm_id)_joint7"
    for route in (mouse, board):
        assert route["follow_joint_trajectory_action"] == "/position_joint_trajectory_controller/follow_joint_trajectory"
        assert route["joint_state_topic"] == "/franka_state_controller/joint_states"
        assert route["joint_names"] == expected_joint_names

    assert entrypoint["send_goals"] == "false"
    assert entrypoint["joint_nudge_delta_rad"] == "$(arg move_left_joint_delta_rad)"
    assert entrypoint["max_abs_joint_delta_rad"] == "0.12"
    assert entrypoint["goal_duration_sec"] == "3.0"
    assert entrypoint["joint_state_max_age_sec"] == "0.5"
    assert entrypoint["action_server_wait_sec"] == "0.5"
