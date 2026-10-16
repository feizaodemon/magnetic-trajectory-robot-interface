"""Gate plans, observed-state records and operator errors.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Tuple

from fr3_lab.textparse import (
    _ros_bool,
)


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
