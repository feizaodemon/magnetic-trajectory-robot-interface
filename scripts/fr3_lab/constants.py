"""Exit codes, controller/action/topic names and adapter terminal states.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations


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
