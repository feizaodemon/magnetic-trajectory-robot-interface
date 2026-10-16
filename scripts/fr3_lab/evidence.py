"""Gate-B evidence: build, store, load and validate.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

from fr3_lab.constants import (
    ACTION_NAME,
    CONTROLLER_NAME,
    EVIDENCE_MAX_AGE_SECONDS,
    EVIDENCE_SCHEMA_VERSION,
    EXIT_GATE_B_EVIDENCE,
    EXIT_RUNTIME_EVIDENCE,
)
from fr3_lab.textparse import (
    parse_utc,
    sha256_text,
    _utc,
)
from fr3_lab.process import (
    CommandRunner,
)
from fr3_lab.model import (
    JointSample,
    LiveContract,
    OperatorBlocked,
    RepositoryState,
)
from fr3_lab.ros_probe import (
    _command_value,
    _libfranka_version,
)
from fr3_lab.config import (
    host_fingerprint,
    joint_contract_fingerprint,
)


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
