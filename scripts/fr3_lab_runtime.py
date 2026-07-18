#!/usr/bin/env python3
"""Native-lab operator facade for the bounded COLMAG real-FR3 route.

The public interface is intentionally limited to gate-b, gate-c, and status.
This module selects already-validated launch modes and observes their evidence;
it does not implement controller, trajectory, lifecycle, or command logic.
"""

from __future__ import annotations

import argparse
import ast
import csv
import getpass
import hashlib
import io
import json
import math
import os
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


EXIT_SUCCESS = 0
EXIT_PREREQUISITE = 20
EXIT_CONFIRMATION = 21
EXIT_RUNTIME_EVIDENCE = 22
EXIT_CLEANUP = 23
EXIT_GATE_B_EVIDENCE = 24
EXIT_USAGE = 64
EXIT_INTERNAL = 70

EVIDENCE_SCHEMA_VERSION = 1
EVIDENCE_MAX_AGE_SECONDS = 30 * 60
OPERATOR_SESSION_TIMEOUT_SECONDS = 120.0
CONTROLLER_NAME = "position_joint_trajectory_controller"
ACTION_NAME = "/position_joint_trajectory_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/franka_state_controller/joint_states"
ADAPTER_STATE_TOPIC = "/colmag/fr3_task_trajectory_adapter_state"
ADAPTER_SESSION_TOPIC = "/colmag/fr3_adapter_session"
TASK_COMMAND_TOPIC = "/colmag/task_command"
IMAGE_NAME = "magnetic-trajectory-interface:fr3-noetic"

TERMINAL_STATES = {
    "FINAL_STATE_VERIFIED",
    "GOAL_REJECTED",
    "GOAL_ABORTED",
    "GOAL_PREEMPTED",
    "GOAL_TIMEOUT",
    "FINAL_STATE_VERIFY_FAILED",
    "SOFTWARE_CANCEL_COMPLETE",
    "SHUTDOWN_CANCELLED",
}
FAILURE_TERMINALS = TERMINAL_STATES - {"FINAL_STATE_VERIFIED"}
ACTIVE_GOAL_STATUS_CODES = {0, 1, 6, 7}


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""


class CommandRunner:
    """Small subprocess seam used by the operator and its tests."""

    def run(self, args: Sequence[str], timeout: Optional[float] = None) -> CommandResult:
        try:
            completed = subprocess.run(
                list(args),
                text=True,
                capture_output=True,
                timeout=timeout,
                check=False,
            )
        except (subprocess.TimeoutExpired, TimeoutError) as exc:
            stdout = _text_or_empty(getattr(exc, "stdout", ""))
            stderr = _text_or_empty(getattr(exc, "stderr", ""))
            return CommandResult(124, stdout, stderr)
        except OSError as exc:
            return CommandResult(127, "", str(exc))
        return CommandResult(completed.returncode, completed.stdout, completed.stderr)


@dataclass(frozen=True)
class GatePlan:
    command: str
    launch_file: str
    hardware_profile: str
    hardware_execution_enabled: bool
    send_goals: bool
    realtime_config: str = "enforce"
    one_shot: bool = True
    max_goals: int = 1
    primitive: str = ""
    expected_label: str = ""
    launch_arguments: Tuple[str, ...] = ()

    def launch_command(self, robot_ip: str) -> Tuple[str, ...]:
        args = [
            "roslaunch",
            "colmag_ros",
            self.launch_file,
            "robot_ip:=" + robot_ip,
            "hardware_profile:=" + self.hardware_profile,
            "hardware_execution_enabled:=" + _ros_bool(self.hardware_execution_enabled),
            "send_goals:=" + _ros_bool(self.send_goals),
            "realtime_config:=" + self.realtime_config,
        ]
        args.extend(
            [
                "one_shot_hardware_mode:=" + _ros_bool(self.one_shot),
                "max_motion_goals_per_process:=" + str(self.max_goals),
            ]
        )
        args.extend(self.launch_arguments)
        return tuple(args)


def gate_b_plan() -> GatePlan:
    return GatePlan(
        command="gate-b",
        launch_file="colmag_fr3_hardware_stack.launch",
        hardware_profile="dry-run",
        hardware_execution_enabled=False,
        send_goals=False,
    )


def gate_c_plan(
    launch_file: str,
    primitive: str,
    expected_label: str,
    launch_arguments: Tuple[str, ...] = (),
) -> GatePlan:
    return GatePlan(
        command="gate-c",
        launch_file=launch_file,
        hardware_profile="fr3-hardware-active",
        hardware_execution_enabled=True,
        send_goals=True,
        primitive=primitive,
        expected_label=expected_label,
        launch_arguments=launch_arguments,
    )


@dataclass(frozen=True)
class RepositoryState:
    revision: str
    clean: bool
    staged_empty: bool


@dataclass(frozen=True)
class JointSample:
    names: Tuple[str, ...]
    positions: Tuple[float, ...]


@dataclass(frozen=True)
class LiveContract:
    controller_joints: Tuple[str, ...]
    live_joints: Tuple[str, ...]
    adapter_joints: Tuple[str, ...]
    adapter_state: Mapping[str, object]
    adapter_session_present: bool
    ros_version: str
    franka_control_version: str
    libfranka_version: str


class OperatorBlocked(RuntimeError):
    def __init__(self, reason: str, exit_code: int):
        super().__init__(reason)
        self.reason = reason
        self.exit_code = exit_code


class OperatorInterrupted(RuntimeError):
    pass


class DiagnosticLog:
    def __init__(self, path: Path, secrets: Iterable[str]):
        self.path = path
        self.secrets = tuple(secret for secret in secrets if secret)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("a", encoding="utf-8")

    def write(self, message: str) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        self._stream.write("{} {}\n".format(timestamp, redact(message, self.secrets)))
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()


class OwnedProcess:
    """Own one process group and clean up only that group."""

    def __init__(
        self,
        args: Sequence[str],
        log: DiagnosticLog,
        secrets: Iterable[str],
        popen_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
        killpg: Callable[[int, int], None] = os.killpg,
        getpgid: Callable[[int], int] = os.getpgid,
    ):
        self.args = tuple(args)
        self.log = log
        self.secrets = tuple(secret for secret in secrets if secret)
        self._killpg = killpg
        self._process = popen_factory(
            list(self.args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.pid = int(self._process.pid)
        self.pgid = int(getpgid(self.pid))
        self.started_at = time.monotonic()
        self.command_fingerprint = command_fingerprint(self.args, self.secrets)
        self._reader = threading.Thread(target=self._pump_output, daemon=True)
        self._reader.start()
        self.log.write(
            "owned_process_started pid={} pgid={} fingerprint={}".format(
                self.pid, self.pgid, self.command_fingerprint
            )
        )

    def _pump_output(self) -> None:
        stream = self._process.stdout
        if stream is None:
            return
        for line in stream:
            self.log.write("process " + line.rstrip("\n"))

    def poll(self) -> Optional[int]:
        return self._process.poll()

    def cleanup(self, interrupt_timeout: float = 10.0, term_timeout: float = 5.0) -> bool:
        if self._process.poll() is not None:
            self._reader.join(timeout=1.0)
            self.log.write("owned_process_already_stopped")
            return True
        if not self._signal_and_wait(signal.SIGINT, interrupt_timeout):
            if not self._signal_and_wait(signal.SIGTERM, term_timeout):
                self.log.write("owned_process_cleanup_incomplete")
                return False
        self._reader.join(timeout=1.0)
        self.log.write("owned_process_cleanup_complete")
        return True

    def _signal_and_wait(self, sig: int, timeout: float) -> bool:
        try:
            self._killpg(self.pgid, sig)
        except ProcessLookupError:
            return True
        except OSError as exc:
            self.log.write("owned_process_signal_failed signal={} error={}".format(sig, exc))
            return False
        try:
            self._process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            self.log.write("owned_process_wait_timeout signal={}".format(sig))
            return False


class GoalObserver:
    """Count FJT goal messages without storing their payloads."""

    def __init__(self, topic: str, log: DiagnosticLog):
        self._count = 0
        self._lock = threading.Lock()
        self._process = subprocess.Popen(
            ["rostopic", "echo", "-p", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._pgid = os.getpgid(self._process.pid)
        self._log = log
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self) -> None:
        stream = self._process.stdout
        if stream is None:
            return
        header_seen = False
        for line in stream:
            if not line.strip():
                continue
            if not header_seen:
                header_seen = True
                continue
            with self._lock:
                self._count += 1

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def cleanup(self) -> bool:
        if self._process.poll() is not None:
            return True
        try:
            os.killpg(self._pgid, signal.SIGINT)
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self._pgid, signal.SIGTERM)
                self._process.wait(timeout=2.0)
            except (subprocess.TimeoutExpired, OSError):
                self._log.write("goal_observer_cleanup_incomplete")
                return False
        except ProcessLookupError:
            return True
        self._reader.join(timeout=1.0)
        return True


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


def maximum_joint_delta(start: JointSample, end: JointSample) -> float:
    if start.names != end.names or len(start.positions) != len(end.positions):
        raise OperatorBlocked("joint_name_mismatch", EXIT_RUNTIME_EVIDENCE)
    return max((abs(after - before) for before, after in zip(start.positions, end.positions)), default=0.0)


def build_gate_b_evidence(
    now: datetime,
    repository: RepositoryState,
    robot_ip: str,
    live: LiveContract,
    max_observed_joint_delta: float,
    cleanup_complete: bool,
) -> Dict[str, object]:
    return {
        "schema_version": EVIDENCE_SCHEMA_VERSION,
        "result": "PASS",
        "completed_at_utc": _utc(now).isoformat().replace("+00:00", "Z"),
        "git_revision": repository.revision,
        "repository_clean": repository.clean and repository.staged_empty,
        "host_fingerprint": host_fingerprint(),
        "robot_ip_sha256": sha256_text(robot_ip),
        "ros_version": live.ros_version,
        "franka_control_version": live.franka_control_version,
        "libfranka_version": live.libfranka_version,
        "controller_name": CONTROLLER_NAME,
        "controller_joint_names": list(live.controller_joints),
        "live_joint_names": list(live.live_joints),
        "adapter_joint_names": list(live.adapter_joints),
        "joint_contract_sha256": joint_contract_fingerprint(
            CONTROLLER_NAME, ACTION_NAME, live.controller_joints
        ),
        "action_name": ACTION_NAME,
        "adapter_session_present": live.adapter_session_present,
        "send_goals": False,
        "hardware_execution_enabled": False,
        "hardware_profile": "dry-run",
        "one_shot": True,
        "max_goals": 1,
        "no_task_publisher": True,
        "no_goal_observed": True,
        "no_motion_confirmed": True,
        "max_observed_joint_delta": max_observed_joint_delta,
        "cleanup_complete": cleanup_complete,
    }


def write_private_json(path: Path, value: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".new")
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    descriptor = os.open(str(temporary), flags, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
        os.chmod(path, 0o600)
    except Exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def load_gate_b_evidence(path: Path, required: bool = True) -> Optional[Dict[str, object]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        if required:
            raise OperatorBlocked("gate_b_evidence_missing", EXIT_GATE_B_EVIDENCE)
        return None
    if not isinstance(value, dict):
        if required:
            raise OperatorBlocked("gate_b_evidence_missing", EXIT_GATE_B_EVIDENCE)
        return None
    return value


def validate_gate_b_evidence(
    evidence: Optional[Mapping[str, object]],
    now: datetime,
    repository: RepositoryState,
    host: str,
    robot_ip: str,
    expected_controller: str,
    expected_action: str,
    expected_joint_fingerprint: str,
    expected_versions: Optional[Tuple[str, str, str]] = None,
) -> str:
    if not evidence:
        return "gate_b_evidence_missing"
    if evidence.get("schema_version") != EVIDENCE_SCHEMA_VERSION or evidence.get("result") != "PASS":
        return "gate_b_evidence_missing"
    completed = parse_utc(str(evidence.get("completed_at_utc", "")))
    age = (_utc(now) - completed).total_seconds() if completed is not None else None
    if age is None or age < 0.0 or age > EVIDENCE_MAX_AGE_SECONDS:
        return "gate_b_evidence_expired"
    required_true = (
        "repository_clean",
        "adapter_session_present",
        "one_shot",
        "no_task_publisher",
        "no_goal_observed",
        "no_motion_confirmed",
        "cleanup_complete",
    )
    mismatched = (
        evidence.get("git_revision") != repository.revision
        or not repository.clean
        or not repository.staged_empty
        or evidence.get("host_fingerprint") != host
        or evidence.get("robot_ip_sha256") != sha256_text(robot_ip)
        or evidence.get("controller_name") != expected_controller
        or evidence.get("action_name") != expected_action
        or evidence.get("joint_contract_sha256") != expected_joint_fingerprint
        or evidence.get("hardware_profile") != "dry-run"
        or evidence.get("send_goals") is not False
        or evidence.get("hardware_execution_enabled") is not False
        or evidence.get("max_goals") != 1
        or any(evidence.get(key) is not True for key in required_true)
    )
    if expected_versions is not None:
        mismatched = mismatched or expected_versions != (
            evidence.get("ros_version"),
            evidence.get("franka_control_version"),
            evidence.get("libfranka_version"),
        )
    return "gate_b_environment_changed" if mismatched else ""


def installed_versions(runner: CommandRunner) -> Tuple[str, str, str]:
    return (
        _command_value(runner, ["rosversion", "-d"]),
        _command_value(runner, ["rosversion", "franka_control"]),
        _libfranka_version(runner),
    )


def evidence_summary(evidence: Optional[Mapping[str, object]], now: datetime) -> Mapping[str, str]:
    if not evidence:
        return {"result": "missing"}
    completed = parse_utc(str(evidence.get("completed_at_utc", "")))
    if completed is None or (_utc(now) - completed).total_seconds() > EVIDENCE_MAX_AGE_SECONDS:
        return {"result": "expired"}
    return {"result": "PASS" if evidence.get("result") == "PASS" else "missing"}


def read_adapter_config(repo: Path) -> Dict[str, object]:
    path = repo / "colmag_ros/config/colmag_fr3_task_trajectory_adapter.yaml"
    values: Dict[str, object] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.startswith("  ") or raw.startswith("    ") or ":" not in raw:
            continue
        key, value = raw.strip().split(":", 1)
        value = value.strip().strip('"\'')
        if not value:
            continue
        values[key] = _parse_scalar(value)
    values.setdefault("target_joint_name", "fr3_joint1")
    return values


def operator_hexagon_offsets(config: Mapping[str, object]) -> Tuple[float, float]:
    try:
        primary = float(config.get("hexagon_primary_offset_rad", math.nan))
        secondary = float(config.get("hexagon_secondary_offset_rad", math.nan))
    except (TypeError, ValueError):
        primary, secondary = math.nan, math.nan
    if any(not math.isfinite(value) or value <= 0.0 or value > 0.12
           for value in (primary, secondary)):
        raise OperatorBlocked("hexagon_offset_invalid", EXIT_GATE_B_EVIDENCE)
    return primary, secondary


def configured_joint_contract(repo: Path) -> str:
    config = read_adapter_config(repo)
    arm_id = str(config.get("arm_id", "fr3"))
    joints = tuple("{}_joint{}".format(arm_id, index) for index in range(1, 8))
    return joint_contract_fingerprint(CONTROLLER_NAME, ACTION_NAME, joints)


def joint_contract_fingerprint(controller: str, action: str, joints: Sequence[str]) -> str:
    value = json.dumps(
        {"controller": controller, "action": action, "joints": list(joints)},
        sort_keys=True,
        separators=(",", ":"),
    )
    return sha256_text(value)


def host_fingerprint() -> str:
    parts = [
        _read_text(Path("/etc/machine-id")).strip(),
        platform.node(),
        platform.system(),
        platform.release(),
        platform.machine(),
    ]
    return sha256_text("\n".join(parts))


def gate_c_outcome_reason(
    terminal: str,
    goal_count: int,
    send_attempts: int,
    observed_task: str,
    expected_task: str,
) -> str:
    if goal_count > 1 or send_attempts > 1:
        return "multiple_goals_observed"
    if observed_task != expected_task:
        return "unexpected_motion_primitive"
    if terminal == "FINAL_STATE_VERIFIED" and goal_count == 1 and send_attempts == 1:
        return "pass"
    if terminal in FAILURE_TERMINALS:
        return terminal.lower()
    if terminal == "FINAL_STATE_VERIFIED":
        return "goal_count_mismatch"
    return "runtime_evidence_failed"


def terminal_state(state: Mapping[str, object]) -> str:
    for key in ("lifecycle_state", "state"):
        value = str(state.get(key, ""))
        if value in TERMINAL_STATES:
            return value
    return ""


def docker_image_revision(runner: CommandRunner) -> str:
    result = runner.run(
        [
            "docker",
            "image",
            "inspect",
            IMAGE_NAME,
            "--format",
            "{{ index .Config.Labels \"org.opencontainers.image.revision\" }}",
        ],
        timeout=3.0,
    )
    if result.returncode != 0 or not result.stdout.strip():
        return "unavailable"
    return result.stdout.strip()[:12]


def parse_sequence(text: str) -> List[object]:
    stripped = text.strip()
    if not stripped:
        return []
    if stripped.startswith("["):
        try:
            value = ast.literal_eval(stripped)
            return list(value) if isinstance(value, (list, tuple)) else []
        except (SyntaxError, ValueError):
            return [item.strip().strip('"\'') for item in stripped.strip("[]").split(",") if item.strip()]
    return [line.strip()[1:].strip().strip('"\'') for line in text.splitlines() if line.strip().startswith("-")]


def parse_named_sequence(text: str, key: str) -> List[object]:
    lines = text.splitlines()
    for index, raw in enumerate(lines):
        if raw.strip().startswith(key + ":"):
            inline = raw.split(":", 1)[1].strip()
            if inline:
                return parse_sequence(inline)
            collected: List[str] = []
            for following in lines[index + 1 :]:
                stripped = following.strip()
                if stripped.startswith("-"):
                    collected.append(stripped)
                    continue
                if stripped:
                    break
            return parse_sequence("\n".join(collected))
    return []


def redact(text: str, secrets: Iterable[str]) -> str:
    result = text
    for secret in secrets:
        if secret:
            result = result.replace(secret, "<redacted>")
    result = re.sub(r"(?i)(FR3_ROBOT_IP\s*[:=]\s*)\S+", r"\1<redacted>", result)
    result = re.sub(
        r'(?i)(adapter_session_id["\']?\s*[:=]\s*["\']?)[^"\',\s}]+',
        r"\1<redacted>",
        result,
    )
    return result


def command_fingerprint(args: Sequence[str], secrets: Iterable[str]) -> str:
    safe = [redact(str(arg), secrets) for arg in args]
    return sha256_text("\0".join(safe))


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def parse_utc(value: str) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return _utc(parsed)


def _utc(value: datetime) -> datetime:
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _text_or_empty(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _ros_bool(value: bool) -> str:
    return "true" if value else "false"


def _format_float(value: float) -> str:
    return "{:.6g}".format(value)


def _unquote(value: str) -> str:
    return value.strip().strip('"\'')


def _string_tuple(value: object) -> Tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    return tuple(str(item) for item in value)


def _int_value(value: object, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


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


def _parse_scalar(value: str) -> object:
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _yes_no(value: object) -> str:
    if value is None:
        return "no"
    return "yes" if bool(value) else "no"


def _enabled_value(value: object) -> str:
    if value is True:
        return "enabled"
    if value is False:
        return "disabled"
    return "unknown"


def _goal_count_value(state: Mapping[str, object]) -> str:
    if "motion_goal_send_attempts" not in state:
        return "unknown"
    return "{}/1".format(_int_value(state.get("motion_goal_send_attempts"), 0))
