import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fr3_lab_runtime as runtime
import task_dispatcher_node as dispatcher
from cpp_harness import compile_and_run


def test_production_hexagon_geometry_is_closed_and_bounded(tmp_path):
    compile_and_run(
        tmp_path,
        "fr3_hexagon_trajectory_test",
        "fr3_hexagon_trajectory.cpp",
        "fr3_trajectory_safety.cpp",
        "fr3_hardware_safety.cpp",
    )


def test_strategy_a_wires_hexagon_into_the_existing_single_send_owner():
    adapter = (REPO / "colmag_ros/src/colmag_fr3_task_trajectory_adapter.cpp").read_text()
    config = (REPO / "colmag_ros/config/colmag_fr3_task_trajectory_adapter.yaml").read_text()
    entrypoint = (
        REPO / "colmag_ros/launch/colmag_fr3_task_trajectory_adapter.launch"
    ).read_text()
    cmake = (REPO / "colmag_ros/CMakeLists.txt").read_text()
    assert 'admission.normalized_task != "HEXAGON_TRAJECTORY"' in adapter
    assert "generateHexagonTrajectory" in adapter
    assert adapter.count("client->sendGoal(") == 1
    assert adapter.count("one_shot_gate_->recordSendAttempt()") == 1
    assert "cancelAllGoals" not in adapter
    assert "colmag_fr3_hexagon_trajectory" in cmake
    assert "- HEXAGON_TRAJECTORY" in config
    assert 'name="hexagon_primary_joint_name" default="$(arg arm_id)_joint1"' in entrypoint
    assert 'name="hexagon_secondary_joint_name" default="$(arg arm_id)_joint6"' in entrypoint
    assert "hexagon_primary_offset_rad: 0.08" in config
    assert "hexagon_secondary_offset_rad: 0.10" in config


def test_gate_c_route_reuses_one_timeout_and_stays_one_shot():
    mouse = REPO / "colmag_ros/launch/colmag_mouse_fr3_hardware_demo.launch"
    stack = REPO / "colmag_ros/launch/colmag_fr3_hardware_stack.launch"
    ET.parse(mouse)
    ET.parse(stack)
    mouse_text = mouse.read_text()
    stack_text = stack.read_text()
    assert 'name="execution_timeout_sec" default="15.0"' in mouse_text
    assert 'name="execution_timeout_sec" value="$(arg execution_timeout_sec)"' in mouse_text
    assert 'name="one_shot_hardware_mode" default="true"' in mouse_text
    assert 'name="max_motion_goals_per_process" default="1"' in mouse_text
    assert 'name="allowed_tasks" default="MOVE_LEFT, HEXAGON_TRAJECTORY, MOVE_RIGHT, STOP_OR_CANCEL"' in mouse_text
    assert stack_text.count('name="execution_timeout_sec"') == 2


def test_existing_label_2_semantic_mapping_is_reused_without_changing_label_3():
    confirmed = {"confirmed": True, "confidence": 1.0}
    assert dispatcher.build_task_command(dict(confirmed, label="2"))["task"] == \
        "HEXAGON_TRAJECTORY"
    assert dispatcher.build_task_command(dict(confirmed, label="3"))["task"] == "MOVE_RIGHT"
    adapter = (REPO / "colmag_ros/src/colmag_fr3_task_trajectory_adapter.cpp").read_text()
    assert 'readString(payload, "task"' in adapter
    assert 'readString(payload, "label"' not in adapter


def test_gate_c_plan_supports_selected_hexagon_without_motion_overrides():
    plan = runtime.gate_c_plan(
        "colmag_mouse_fr3_hardware_demo.launch", "HEXAGON_TRAJECTORY", "2"
    )
    assert plan.primitive == "HEXAGON_TRAJECTORY"
    assert plan.expected_label == "2"
    assert plan.max_goals == 1
    command = plan.launch_command("secret-ip")
    assert "joint_nudge_delta_rad" not in " ".join(command)
    assert all("primitive" not in argument and "hexagon" not in argument for argument in command)


def test_gate_c_operator_uses_exact_label_2_instructions_and_no_direct_publish():
    operator = (REPO / "scripts/fr3_lab_operator.py").read_text()
    # fr3_lab_runtime.py re-exports the scripts/fr3_lab package; check both.
    runtime_source = "\n".join(
        path.read_text()
        for path in [REPO / "scripts/fr3_lab_runtime.py", *sorted((REPO / "scripts/fr3_lab").glob("*.py"))]
    )
    assert "action_plan.expected_label" in operator
    assert "action_plan.expected_task" in operator
    for forbidden in (
        "rostopic pub",
        "FollowJointTrajectoryGoal",
        "automatic_home",
        "automatic_recovery",
    ):
        assert forbidden not in operator + runtime_source


@pytest.mark.parametrize(
    "terminal,goals,attempts,task,reason",
    [
        ("FINAL_STATE_VERIFIED", 1, 1, "HEXAGON_TRAJECTORY", "pass"),
        (
            "FINAL_STATE_VERIFIED",
            1,
            1,
            "JOINT_NUDGE_POSITIVE",
            "unexpected_motion_primitive",
        ),
        ("FINAL_STATE_VERIFIED", 2, 1, "HEXAGON_TRAJECTORY", "multiple_goals_observed"),
        ("GOAL_ABORTED", 1, 1, "HEXAGON_TRAJECTORY", "goal_aborted"),
    ],
)
def test_gate_c_requires_hexagon_exactly_one_goal_and_final_verification(
    terminal, goals, attempts, task, reason
):
    assert runtime.gate_c_outcome_reason(
        terminal, goals, attempts, task, "HEXAGON_TRAJECTORY"
    ) == reason


@pytest.mark.parametrize(
    "config",
    [
        {"hexagon_primary_offset_rad": 0.0, "hexagon_secondary_offset_rad": 0.01},
        {"hexagon_primary_offset_rad": 0.08, "hexagon_secondary_offset_rad": 0.120001},
        {"hexagon_primary_offset_rad": float("nan"), "hexagon_secondary_offset_rad": 0.01},
    ],
)
def test_operator_rejects_hexagon_offsets_outside_hard_limit(config):
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        runtime.operator_hexagon_offsets(config)
    assert blocked.value.reason == "hexagon_offset_invalid"
