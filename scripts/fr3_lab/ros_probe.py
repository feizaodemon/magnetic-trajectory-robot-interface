"""ROS graph, controller, topic and host probes.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

import csv
import io
import json
import math
import platform
import re
import shutil
import socket
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

from fr3_lab.constants import (
    ACTION_NAME,
    ACTIVE_GOAL_STATUS_CODES,
    ADAPTER_SESSION_TOPIC,
    ADAPTER_STATE_TOPIC,
    CONTROLLER_NAME,
    EXIT_PREREQUISITE,
    EXIT_RUNTIME_EVIDENCE,
    JOINT_STATE_TOPIC,
    TASK_COMMAND_TOPIC,
)
from fr3_lab.textparse import (
    _int_value,
    parse_named_sequence,
    parse_sequence,
    _read_text,
    _string_tuple,
    _unquote,
)
from fr3_lab.process import (
    CommandRunner,
)
from fr3_lab.model import (
    GatePlan,
    JointSample,
    LiveContract,
    OperatorBlocked,
    RepositoryState,
)


def repository_state(runner: CommandRunner, repo: Path) -> RepositoryState:
    revision = runner.run(["git", "-C", str(repo), "rev-parse", "HEAD"], timeout=5.0)
    status = runner.run(["git", "-C", str(repo), "status", "--porcelain"], timeout=5.0)
    staged = runner.run(["git", "-C", str(repo), "diff", "--cached", "--name-only"], timeout=5.0)
    if revision.returncode != 0 or status.returncode != 0 or staged.returncode != 0:
        raise OperatorBlocked("repository_unavailable", EXIT_PREREQUISITE)
    return RepositoryState(
        revision=revision.stdout.strip(),
        clean=not status.stdout.strip(),
        staged_empty=not staged.stdout.strip(),
    )


def is_native_linux() -> bool:
    if platform.system() != "Linux":
        return False
    text = "{} {}".format(platform.release(), _read_text(Path("/proc/version"))).lower()
    return "microsoft" not in text and "wsl" not in text


def ros_noetic_available(runner: CommandRunner) -> bool:
    if any(shutil.which(tool) is None for tool in ("roslaunch", "rosnode", "rostopic", "rosparam", "rosservice", "rosversion", "rospack")):
        return False
    result = runner.run(["rosversion", "-d"], timeout=3.0)
    return result.returncode == 0 and result.stdout.strip() == "noetic"


def franka_stack_available(runner: CommandRunner, repo: Path) -> bool:
    launch = repo / "colmag_ros/launch/colmag_fr3_hardware_stack.launch"
    if not launch.is_file():
        return False
    for package in (
        "franka_control",
        "franka_description",
        "controller_manager",
        "joint_trajectory_controller",
        "colmag_ros",
    ):
        if runner.run(["rospack", "find", package], timeout=3.0).returncode != 0:
            return False
    return True


def detect_control_processes(runner: CommandRunner) -> Tuple[str, ...]:
    result = runner.run(["ps", "-eo", "args="], timeout=3.0)
    if result.returncode != 0:
        return ("process_scan_failed",)
    markers = (
        "franka_control_node",
        "colmag_fr3_task_trajectory_adapter",
        "task_dispatcher_node.py",
        "colmag_fr3_hardware_stack.launch",
        "colmag_mouse_fr3_hardware_demo.launch",
        "colmag_real_board_fr3_hardware_demo.launch",
    )
    return tuple(line.strip() for line in result.stdout.splitlines() if any(marker in line for marker in markers))


def probe_franka_port(robot_ip: str) -> bool:
    try:
        connection = socket.create_connection((robot_ip, 1337), timeout=2.0)
    except OSError:
        return False
    connection.close()
    return True


def ros_master_available(runner: CommandRunner) -> bool:
    return runner.run(["rosnode", "list"], timeout=2.0).returncode == 0


def ros_nodes(runner: CommandRunner) -> Tuple[str, ...]:
    result = runner.run(["rosnode", "list"], timeout=2.0)
    if result.returncode != 0:
        return ()
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip())


def task_publishers(runner: CommandRunner) -> Tuple[str, ...]:
    result = runner.run(["rostopic", "info", TASK_COMMAND_TOPIC], timeout=2.0)
    if result.returncode != 0:
        return ()
    return parse_ros_topic_publishers(result.stdout)


def parse_ros_topic_publishers(text: str) -> Tuple[str, ...]:
    publishers: List[str] = []
    in_publishers = False
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("Publishers:"):
            in_publishers = True
            inline = line.split(":", 1)[1].strip()
            if inline and inline != "None":
                publishers.append(inline)
            continue
        if line.startswith("Subscribers:"):
            break
        if in_publishers and line.startswith("*"):
            publishers.append(line[1:].split("(", 1)[0].strip())
    return tuple(publishers)


def read_controllers(runner: CommandRunner) -> Dict[str, str]:
    result = runner.run(["rosservice", "call", "/controller_manager/list_controllers", "{}"], timeout=3.0)
    if result.returncode != 0:
        return {}
    return parse_controller_states(result.stdout)


def parse_controller_states(text: str) -> Dict[str, str]:
    controllers: Dict[str, str] = {}
    current = ""
    for raw in text.splitlines():
        line = raw.strip().lstrip("-").strip()
        if line.startswith("name:"):
            current = _unquote(line.split(":", 1)[1].strip())
        elif line.startswith("state:") and current:
            controllers[current] = _unquote(line.split(":", 1)[1].strip())
            current = ""
    return controllers


def read_controller_joints(runner: CommandRunner) -> Tuple[str, ...]:
    result = runner.run(["rosparam", "get", "/{}/joints".format(CONTROLLER_NAME)], timeout=3.0)
    if result.returncode != 0:
        raise OperatorBlocked("controller_not_running", EXIT_RUNTIME_EVIDENCE)
    joints = parse_sequence(result.stdout)
    if not joints:
        raise OperatorBlocked("joint_name_mismatch", EXIT_RUNTIME_EVIDENCE)
    return tuple(str(value) for value in joints)


def read_joint_sample(runner: CommandRunner) -> JointSample:
    result = runner.run(["rostopic", "echo", "-n", "1", JOINT_STATE_TOPIC], timeout=5.0)
    if result.returncode != 0:
        raise OperatorBlocked("joint_name_mismatch", EXIT_RUNTIME_EVIDENCE)
    names = tuple(str(value) for value in parse_named_sequence(result.stdout, "name"))
    positions = tuple(float(value) for value in parse_named_sequence(result.stdout, "position"))
    if not names or len(names) != len(positions) or not all(math.isfinite(value) for value in positions):
        raise OperatorBlocked("joint_name_mismatch", EXIT_RUNTIME_EVIDENCE)
    return JointSample(names, positions)


def read_adapter_state(runner: CommandRunner, timeout: float = 5.0) -> Mapping[str, object]:
    payload = read_string_topic(runner, ADAPTER_STATE_TOPIC, timeout)
    if not payload:
        return {}
    try:
        value = json.loads(payload)
    except (TypeError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def read_adapter_session_present(runner: CommandRunner) -> bool:
    return bool(read_string_topic(runner, ADAPTER_SESSION_TOPIC, 3.0).strip())


def read_string_topic(runner: CommandRunner, topic: str, timeout: float) -> str:
    result = runner.run(["rostopic", "echo", "-p", "-n", "1", topic], timeout=timeout)
    if result.returncode != 0:
        return ""
    rows = list(csv.DictReader(io.StringIO(result.stdout)))
    if not rows:
        return ""
    row = rows[-1]
    for key, value in row.items():
        if key and key.endswith("data"):
            return value or ""
    return ""


def collect_live_contract(runner: CommandRunner) -> LiveContract:
    state = read_adapter_state(runner)
    sample = read_joint_sample(runner)
    return LiveContract(
        controller_joints=read_controller_joints(runner),
        live_joints=sample.names,
        adapter_joints=_string_tuple(state.get("expected_joint_names")),
        adapter_state=state,
        adapter_session_present=read_adapter_session_present(runner),
        ros_version=_command_value(runner, ["rosversion", "-d"]),
        franka_control_version=_command_value(runner, ["rosversion", "franka_control"]),
        libfranka_version=_libfranka_version(runner),
    )


def validate_live_contract(live: LiveContract, plan: GatePlan) -> None:
    if not live.controller_joints or not (
        live.controller_joints == live.live_joints == live.adapter_joints
    ):
        raise OperatorBlocked("joint_name_mismatch", EXIT_RUNTIME_EVIDENCE)
    state = live.adapter_state
    expected = {
        "send_goals": plan.send_goals,
        "hardware_execution_enabled": plan.hardware_execution_enabled,
        "hardware_profile": plan.hardware_profile,
        "one_shot_hardware_mode": True,
    }
    if any(state.get(key) != value for key, value in expected.items()):
        raise OperatorBlocked("gate_mapping_mismatch", EXIT_RUNTIME_EVIDENCE)
    if _int_value(state.get("motion_goal_send_attempts"), -1) != 0:
        raise OperatorBlocked("active_goal_present", EXIT_RUNTIME_EVIDENCE)
    if not live.adapter_session_present:
        raise OperatorBlocked("adapter_session_missing", EXIT_RUNTIME_EVIDENCE)


def validate_graph_contract(runner: CommandRunner, expect_task_publisher: bool) -> None:
    topics = _command_value_tuple(runner, ["rostopic", "list"])
    required = {ACTION_NAME + suffix for suffix in ("/goal", "/status", "/result")}
    if not required.issubset(set(topics)):
        raise OperatorBlocked("action_endpoint_missing", EXIT_RUNTIME_EVIDENCE)
    publishers = task_publishers(runner)
    if expect_task_publisher:
        if publishers != ("/task_dispatcher_node",):
            raise OperatorBlocked("task_publisher_present", EXIT_RUNTIME_EVIDENCE)
    elif publishers:
        raise OperatorBlocked("task_publisher_present", EXIT_RUNTIME_EVIDENCE)
    controllers = read_controllers(runner)
    if any("colmag_fr3_preset_primitive_controller" in name and state == "running"
           for name, state in controllers.items()):
        raise OperatorBlocked("strategy_b_running", EXIT_RUNTIME_EVIDENCE)


def active_goal_present(runner: CommandRunner) -> bool:
    result = runner.run(["rostopic", "echo", "-n", "1", ACTION_NAME + "/status"], timeout=2.0)
    if result.returncode != 0:
        return False
    return any(int(code) in ACTIVE_GOAL_STATUS_CODES for code in re.findall(r"\bstatus:\s*(\d+)\b", result.stdout))


def observe_one_message(runner: CommandRunner, topic: str, seconds: float) -> bool:
    result = runner.run(["rostopic", "echo", "-n", "1", topic], timeout=seconds)
    return result.returncode == 0 and bool(result.stdout.strip())


def _command_value(runner: CommandRunner, args: Sequence[str]) -> str:
    result = runner.run(args, timeout=3.0)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _command_value_tuple(runner: CommandRunner, args: Sequence[str]) -> Tuple[str, ...]:
    value = _command_value(runner, args)
    return tuple(line.strip() for line in value.splitlines() if line.strip() and value != "unknown")


def _libfranka_version(runner: CommandRunner) -> str:
    for package in ("libfranka", "franka"):
        result = runner.run(["pkg-config", "--modversion", package], timeout=3.0)
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    return "unknown"
