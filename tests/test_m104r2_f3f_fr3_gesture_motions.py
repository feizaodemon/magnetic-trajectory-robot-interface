import subprocess
import sys
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
INCLUDE_DIR = REPO / "colmag_ros" / "include"
BUILD_DIR = REPO / "outputs" / "agent_tmp" / "m104r2_f3f"
SCRIPTS = REPO / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fr3_lab_operator as operator
import fr3_lab_runtime as runtime


def test_production_joint_excursion_returns_to_full_start_vector():
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    executable = BUILD_DIR / "fr3_joint_excursion_trajectory_test"
    compile_result = subprocess.run(
        [
            "g++",
            "-std=c++14",
            "-Wall",
            "-Wextra",
            "-pedantic",
            "-I",
            str(INCLUDE_DIR),
            str(REPO / "tests/cpp/fr3_joint_excursion_trajectory_test.cpp"),
            str(REPO / "colmag_ros/src/fr3_joint_excursion_trajectory.cpp"),
            str(REPO / "colmag_ros/src/fr3_trajectory_safety.cpp"),
            "-o",
            str(executable),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr
    run_result = subprocess.run(
        [str(executable)], cwd=REPO, text=True, capture_output=True, check=False
    )
    assert run_result.returncode == 0, run_result.stderr


def test_strategy_a_wires_three_semantic_motions_to_one_send_path():
    adapter = (REPO / "colmag_ros/src/colmag_fr3_task_trajectory_adapter.cpp").read_text()
    cmake = (REPO / "colmag_ros/CMakeLists.txt").read_text()
    assert '#include "colmag_ros/fr3_joint_excursion_trajectory.h"' in adapter
    assert adapter.count("client->sendGoal(") == 1
    assert 'task == "MOVE_LEFT"' in adapter
    assert 'task == "MOVE_RIGHT"' in adapter
    assert "generateJointExcursionTrajectory" in adapter
    assert "colmag_fr3_joint_excursion_trajectory" in cmake


def test_real_fr3_defaults_use_visible_bounded_amplitudes_and_semantic_allowlist():
    config = (
        REPO / "colmag_ros/config/colmag_fr3_task_trajectory_adapter.yaml"
    ).read_text()
    stack = (REPO / "colmag_ros/launch/colmag_fr3_hardware_stack.launch").read_text()
    for value in (
        "joint_nudge_delta_rad: 0.10",
        "max_abs_joint_delta_rad: 0.12",
        "hexagon_primary_offset_rad: 0.08",
        "hexagon_secondary_offset_rad: 0.10",
        "execution_timeout_sec: 15.0",
        "- MOVE_LEFT",
        "- HEXAGON_TRAJECTORY",
        "- MOVE_RIGHT",
        "- STOP_OR_CANCEL",
    ):
        assert value in config
    assert 'name="target_joint_name" default="$(arg arm_id)_joint1"' in stack
    entrypoint = (
        REPO / "colmag_ros/launch/m104r1c_fr3_task_trajectory_adapter_static_entrypoint.launch"
    ).read_text()
    assert 'name="hexagon_primary_joint_name" default="$(arg arm_id)_joint1"' in entrypoint
    assert 'name="hexagon_secondary_joint_name" default="$(arg arm_id)_joint6"' in entrypoint


def test_docker_image_build_installs_the_visible_motion_contract():
    dockerfile = (REPO / "docker/Dockerfile.noetic-fr3").read_text()
    compose = (REPO / "docker/compose.fr3.yaml").read_text()
    assert "COPY colmag_ros src/colmag_ros" in dockerfile
    assert "catkin_make install" in dockerfile
    assert "COPY --from=builder /opt/colmag_ws/install /opt/colmag_ws/install" in dockerfile
    assert 'org.opencontainers.image.version="portfolio-fr3-noetic"' in dockerfile
    assert "grep -F 'joint_nudge_delta_rad: 0.10'" in dockerfile
    assert "grep -F 'max_abs_joint_delta_rad: 0.12'" in dockerfile
    assert "grep -F 'hexagon_primary_offset_rad: 0.08'" in dockerfile
    assert "grep -F 'hexagon_secondary_offset_rad: 0.10'" in dockerfile
    assert "grep -F 'kVisibleMotionHardLimitRad = 0.12'" in dockerfile
    assert compose.count("*fr3-common") == 3
    assert "magnetic-trajectory-interface:fr3-noetic" in compose


@pytest.mark.parametrize(
    "choice,label,task",
    [
        ("1", "1", "MOVE_LEFT"),
        ("2", "2", "HEXAGON_TRAJECTORY"),
        ("3", "3", "MOVE_RIGHT"),
    ],
)
def test_gate_c_action_selection_maps_to_typed_semantic(choice, label, task):
    lab = operator.Fr3LabOperator(
        REPO, input_func=lambda _prompt: choice, interactive=lambda: True, environ={}
    )
    action = operator.gate_c_action_plan(lab._select_action())
    assert action.expected_label == label
    assert action.expected_task == task


@pytest.mark.parametrize("input_source", list(operator.InputSource))
@pytest.mark.parametrize(
    "action,task",
    [
        (operator.GateCAction.LEFT, "MOVE_LEFT"),
        (operator.GateCAction.HEXAGON, "HEXAGON_TRAJECTORY"),
        (operator.GateCAction.RIGHT, "MOVE_RIGHT"),
    ],
)
def test_input_source_plan_does_not_change_selected_semantic_action(
    input_source, action, task
):
    source_plan = operator.gate_c_input_plan(input_source)
    action_plan = operator.gate_c_action_plan(action)
    assert source_plan.input_source is input_source
    assert action_plan.expected_task == task


@pytest.mark.parametrize("choice", ["", "4", "left"])
def test_gate_c_action_selection_has_no_default_or_free_form_value(choice):
    lab = operator.Fr3LabOperator(
        REPO, input_func=lambda _prompt: choice, interactive=lambda: True, environ={}
    )
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        lab._select_action()
    assert blocked.value.reason == "action_selection_invalid"


@pytest.mark.parametrize("task", ["MOVE_LEFT", "HEXAGON_TRAJECTORY", "MOVE_RIGHT"])
def test_gate_c_outcome_requires_the_selected_semantic_task(task):
    assert runtime.gate_c_outcome_reason(
        "FINAL_STATE_VERIFIED", 1, 1, task, task
    ) == "pass"
    assert runtime.gate_c_outcome_reason(
        "FINAL_STATE_VERIFIED", 1, 1, "STOP_OR_CANCEL", task
    ) == "unexpected_motion_primitive"


def test_public_cli_and_operator_do_not_expose_or_publish_action_overrides():
    assert operator.main(["gate-c", "--action", "1"]) == runtime.EXIT_USAGE
    source = (REPO / "scripts/fr3_lab_operator.py").read_text()
    assert "rostopic pub" not in source
    assert "FollowJointTrajectoryGoal" not in source
