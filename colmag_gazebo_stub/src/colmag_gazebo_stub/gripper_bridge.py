"""ROS-free rules for the Gazebo TELEOP gripper bridge.

The bridge turns keyboard open/close commands into franka_gripper actions on
the simulated Panda hand. It admits a command only while TELEOP is the observed
control mode, and only a fresh command with a new sequence id.
"""

import json
import math

from colmag_ros import control_mode

OPEN = "open"
CLOSE = "close"
COMMANDS = (OPEN, CLOSE)

FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")

# Bridge states published on the gripper state topic.
OPENING = "OPENING"
CLOSING = "CLOSING"
OPENED = "OPEN"
CLOSED = "CLOSED"
HOLDING = "HOLDING"
REJECTED = "REJECTED"
FAILED = "FAILED"
UNKNOWN = "UNKNOWN"


def parse_command(data):
    """``(command, sequence_id, timestamp)`` from a JSON payload; ``ValueError`` if invalid."""
    try:
        payload = json.loads(data)
    except (TypeError, ValueError):
        raise ValueError("invalid_gripper_payload")
    if not isinstance(payload, dict):
        raise ValueError("invalid_gripper_payload")
    command = str(payload.get("command", "")).strip().lower()
    if command not in COMMANDS:
        raise ValueError("unknown_gripper_command")
    try:
        sequence_id = int(payload["sequence_id"])
        timestamp = float(payload["timestamp"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("invalid_gripper_payload")
    if not math.isfinite(timestamp):
        raise ValueError("invalid_gripper_payload")
    return command, sequence_id, timestamp


class CommandGate:
    """Fail-closed admission: TELEOP observed, fresh, and a new sequence id."""

    def __init__(self, max_age_sec=1.0):
        self.max_age_sec = float(max_age_sec)
        self.last_sequence_id = None

    def admit(self, sequence_id, timestamp, current_mode, mode_observed, now):
        """Return ``""`` when admitted, otherwise the rejection reason."""
        if not control_mode.mode_allows(control_mode.TELEOP, current_mode, mode_observed):
            return "teleop_not_owner"
        if self.last_sequence_id is not None and sequence_id <= self.last_sequence_id:
            return "non_monotonic_gripper_command"
        if abs(now - timestamp) > self.max_age_sec:
            return "stale_gripper_command"
        self.last_sequence_id = sequence_id
        return ""


def finger_width(names, positions):
    """Opening between the fingers in metres, or ``None`` if the joints are missing."""
    try:
        return sum(float(positions[list(names).index(joint)]) for joint in FINGER_JOINTS)
    except (ValueError, IndexError, TypeError):
        return None


def settled_state(command, succeeded, width, holding_min_width_m):
    """State after an action ends: ``OPEN``, ``HOLDING``, ``CLOSED`` or ``FAILED``."""
    if command == OPEN:
        return OPENED if succeeded else FAILED
    if not succeeded:
        return FAILED
    # A grasp that stops on an object leaves the fingers apart.
    if width is not None and width >= holding_min_width_m:
        return HOLDING
    return CLOSED


def state_payload(state, reason, command, width):
    return {
        "state": state,
        "reason": reason,
        "command": command or "",
        "width_m": None if width is None else round(float(width), 4),
    }
