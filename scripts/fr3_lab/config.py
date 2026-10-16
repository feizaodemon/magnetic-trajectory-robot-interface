"""Adapter config, fingerprints and adapter outcome checks.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

import json
import math
import platform
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

from fr3_lab.constants import (
    ACTION_NAME,
    CONTROLLER_NAME,
    EXIT_GATE_B_EVIDENCE,
    FAILURE_TERMINALS,
    IMAGE_NAME,
    TERMINAL_STATES,
)
from fr3_lab.textparse import (
    _parse_scalar,
    _read_text,
    sha256_text,
)
from fr3_lab.process import (
    CommandRunner,
)
from fr3_lab.model import (
    OperatorBlocked,
)


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
