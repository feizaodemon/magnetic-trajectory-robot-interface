#!/usr/bin/env python3
"""Native-lab operator facade for the bounded COLMAG real-FR3 route.

The public interface is intentionally limited to gate-b, gate-c, and status.
This module selects already-validated launch modes and observes their evidence;
it does not implement controller, trajectory, lifecycle, or command logic.
"""

from __future__ import annotations

# The code lives in the fr3_lab package next to this file; this module
# re-exports every name it used to define, plus the stdlib modules that
# callers and tests reach through it (for example fr3_lab_runtime.subprocess).
import ast  # noqa: F401
import csv  # noqa: F401
import hashlib  # noqa: F401
import io  # noqa: F401
import json  # noqa: F401
import math  # noqa: F401
import os  # noqa: F401
import platform  # noqa: F401
import re  # noqa: F401
import shutil  # noqa: F401
import signal  # noqa: F401
import socket  # noqa: F401
import subprocess  # noqa: F401
import threading  # noqa: F401
import time  # noqa: F401
from dataclasses import dataclass  # noqa: F401
from datetime import datetime, timezone  # noqa: F401
from pathlib import Path  # noqa: F401
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple  # noqa: F401

from fr3_lab.constants import (
    EXIT_SUCCESS,
    EXIT_PREREQUISITE,
    EXIT_CONFIRMATION,
    EXIT_RUNTIME_EVIDENCE,
    EXIT_CLEANUP,
    EXIT_GATE_B_EVIDENCE,
    EXIT_USAGE,
    EXIT_INTERNAL,
    EVIDENCE_SCHEMA_VERSION,
    EVIDENCE_MAX_AGE_SECONDS,
    OPERATOR_SESSION_TIMEOUT_SECONDS,
    CONTROLLER_NAME,
    ACTION_NAME,
    JOINT_STATE_TOPIC,
    ADAPTER_STATE_TOPIC,
    ADAPTER_SESSION_TOPIC,
    TASK_COMMAND_TOPIC,
    IMAGE_NAME,
    TERMINAL_STATES,
    FAILURE_TERMINALS,
    ACTIVE_GOAL_STATUS_CODES,
)
from fr3_lab.textparse import (
    parse_sequence,
    parse_named_sequence,
    redact,
    command_fingerprint,
    sha256_text,
    parse_utc,
    _utc,
    _read_text,
    _text_or_empty,
    _ros_bool,
    _unquote,
    _string_tuple,
    _int_value,
    _parse_scalar,
    _yes_no,
    _enabled_value,
    _goal_count_value,
)
from fr3_lab.process import (
    CommandResult,
    CommandRunner,
    DiagnosticLog,
    OwnedProcess,
    GoalObserver,
)
from fr3_lab.model import (
    GatePlan,
    gate_b_plan,
    gate_c_plan,
    RepositoryState,
    JointSample,
    LiveContract,
    OperatorBlocked,
    OperatorInterrupted,
)
from fr3_lab.ros_probe import (
    repository_state,
    is_native_linux,
    ros_noetic_available,
    franka_stack_available,
    detect_control_processes,
    probe_franka_port,
    ros_master_available,
    ros_nodes,
    task_publishers,
    parse_ros_topic_publishers,
    read_controllers,
    parse_controller_states,
    read_controller_joints,
    read_joint_sample,
    read_adapter_state,
    read_adapter_session_present,
    read_string_topic,
    collect_live_contract,
    validate_live_contract,
    validate_graph_contract,
    active_goal_present,
    observe_one_message,
    _command_value,
    _command_value_tuple,
    _libfranka_version,
)
from fr3_lab.config import (
    read_adapter_config,
    operator_hexagon_offsets,
    configured_joint_contract,
    joint_contract_fingerprint,
    host_fingerprint,
    gate_c_outcome_reason,
    terminal_state,
    docker_image_revision,
)
from fr3_lab.evidence import (
    maximum_joint_delta,
    build_gate_b_evidence,
    write_private_json,
    load_gate_b_evidence,
    validate_gate_b_evidence,
    installed_versions,
    evidence_summary,
)

__all__ = [
    "EXIT_SUCCESS",
    "EXIT_PREREQUISITE",
    "EXIT_CONFIRMATION",
    "EXIT_RUNTIME_EVIDENCE",
    "EXIT_CLEANUP",
    "EXIT_GATE_B_EVIDENCE",
    "EXIT_USAGE",
    "EXIT_INTERNAL",
    "EVIDENCE_SCHEMA_VERSION",
    "EVIDENCE_MAX_AGE_SECONDS",
    "OPERATOR_SESSION_TIMEOUT_SECONDS",
    "CONTROLLER_NAME",
    "ACTION_NAME",
    "JOINT_STATE_TOPIC",
    "ADAPTER_STATE_TOPIC",
    "ADAPTER_SESSION_TOPIC",
    "TASK_COMMAND_TOPIC",
    "IMAGE_NAME",
    "TERMINAL_STATES",
    "FAILURE_TERMINALS",
    "ACTIVE_GOAL_STATUS_CODES",
    "parse_sequence",
    "parse_named_sequence",
    "redact",
    "command_fingerprint",
    "sha256_text",
    "parse_utc",
    "_utc",
    "_read_text",
    "_text_or_empty",
    "_ros_bool",
    "_unquote",
    "_string_tuple",
    "_int_value",
    "_parse_scalar",
    "_yes_no",
    "_enabled_value",
    "_goal_count_value",
    "CommandResult",
    "CommandRunner",
    "DiagnosticLog",
    "OwnedProcess",
    "GoalObserver",
    "GatePlan",
    "gate_b_plan",
    "gate_c_plan",
    "RepositoryState",
    "JointSample",
    "LiveContract",
    "OperatorBlocked",
    "OperatorInterrupted",
    "repository_state",
    "is_native_linux",
    "ros_noetic_available",
    "franka_stack_available",
    "detect_control_processes",
    "probe_franka_port",
    "ros_master_available",
    "ros_nodes",
    "task_publishers",
    "parse_ros_topic_publishers",
    "read_controllers",
    "parse_controller_states",
    "read_controller_joints",
    "read_joint_sample",
    "read_adapter_state",
    "read_adapter_session_present",
    "read_string_topic",
    "collect_live_contract",
    "validate_live_contract",
    "validate_graph_contract",
    "active_goal_present",
    "observe_one_message",
    "_command_value",
    "_command_value_tuple",
    "_libfranka_version",
    "read_adapter_config",
    "operator_hexagon_offsets",
    "configured_joint_contract",
    "joint_contract_fingerprint",
    "host_fingerprint",
    "gate_c_outcome_reason",
    "terminal_state",
    "docker_image_revision",
    "maximum_joint_delta",
    "build_gate_b_evidence",
    "write_private_json",
    "load_gate_b_evidence",
    "validate_gate_b_evidence",
    "installed_versions",
    "evidence_summary",
]
