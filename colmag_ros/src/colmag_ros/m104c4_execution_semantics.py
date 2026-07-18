"""Gazebo-only execution semantics for M104-C4.

This module is intentionally ROS-free. It is the shared C4 metadata seam for
the dispatcher, Gazebo bridge, tests, and docs.
"""

C4_MAPPING_NAME = "m104c4_8symbol_gazebo"

C4_TARGET_LABELS = ["1", "2", "3", "V", "O", "X", "A", "C"]

C4_EXECUTION_SEMANTICS = {
    "1": {
        "task": "MOVE_LEFT",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Existing Gazebo Panda body left-motion primitive.",
    },
    "2": {
        "task": "HEXAGON_TRAJECTORY",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Existing Gazebo Panda body hexagon placeholder primitive.",
    },
    "3": {
        "task": "MOVE_RIGHT",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Existing Gazebo Panda body right-motion primitive.",
    },
    "V": {
        "task": "HOVER_APPROACH",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Non-contact hover/approach preview in Gazebo only.",
    },
    "O": {
        "task": "ORBIT_SMALL",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Small non-contact orbit/circle preview in Gazebo only.",
    },
    "X": {
        "task": "STOP_OR_CANCEL",
        "moves_gazebo_robot_body": False,
        "execution_class": "cancel_no_motion",
        "note": "Safe cancel/status-only behavior; no forceful motion.",
    },
    "A": {
        "task": "HOME_OR_READY",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Gazebo-only move toward a neutral ready pose.",
    },
    "C": {
        "task": "SOFT_DESCEND_PREVIEW",
        "moves_gazebo_robot_body": True,
        "execution_class": "joint_trajectory",
        "note": "Non-contact soft-descend preview only; no compliance/contact control.",
    },
}

C4_LABEL_TO_TASK = {
    label: semantics["task"]
    for label, semantics in C4_EXECUTION_SEMANTICS.items()
}

C4_SAFE_GAZEBO_TASKS = frozenset(C4_LABEL_TO_TASK.values())

C4_MOVING_TASKS = frozenset(
    semantics["task"]
    for semantics in C4_EXECUTION_SEMANTICS.values()
    if semantics["moves_gazebo_robot_body"]
)

C4_NO_MOTION_TASKS = frozenset(
    semantics["task"]
    for semantics in C4_EXECUTION_SEMANTICS.values()
    if not semantics["moves_gazebo_robot_body"]
)


def validated_execution_contract(namespace=None):
    """Return the C4 contract or raise when required semantics are malformed."""
    values = globals() if namespace is None else vars(namespace)
    required = (
        "C4_MAPPING_NAME",
        "C4_TARGET_LABELS",
        "C4_EXECUTION_SEMANTICS",
        "C4_LABEL_TO_TASK",
        "C4_SAFE_GAZEBO_TASKS",
        "C4_MOVING_TASKS",
        "C4_NO_MOTION_TASKS",
    )
    missing = [name for name in required if name not in values]
    if missing:
        raise RuntimeError("C4 execution semantics missing attributes: %s" % ", ".join(missing))

    mapping = values["C4_LABEL_TO_TASK"]
    semantics = values["C4_EXECUTION_SEMANTICS"]
    labels = values["C4_TARGET_LABELS"]
    safe_tasks = frozenset(values["C4_SAFE_GAZEBO_TASKS"])
    moving_tasks = frozenset(values["C4_MOVING_TASKS"])
    no_motion_tasks = frozenset(values["C4_NO_MOTION_TASKS"])
    if not isinstance(mapping, dict) or not mapping:
        raise RuntimeError("C4_LABEL_TO_TASK must be a non-empty mapping")
    if not isinstance(semantics, dict) or set(semantics) != set(mapping):
        raise RuntimeError("C4_EXECUTION_SEMANTICS must match C4_LABEL_TO_TASK labels")
    if set(labels) != set(mapping):
        raise RuntimeError("C4_TARGET_LABELS must match C4_LABEL_TO_TASK labels")
    if any(semantics[label].get("task") != task for label, task in mapping.items()):
        raise RuntimeError("C4 execution semantics task values do not match mapping")
    if safe_tasks != frozenset(mapping.values()):
        raise RuntimeError("C4_SAFE_GAZEBO_TASKS must match mapping task values")
    if moving_tasks & no_motion_tasks or moving_tasks | no_motion_tasks != safe_tasks:
        raise RuntimeError("C4 moving/no-motion task sets must partition safe tasks")

    return {
        "mapping_name": str(values["C4_MAPPING_NAME"]),
        "label_to_task": dict(mapping),
        "safe_tasks": safe_tasks,
        "moving_tasks": moving_tasks,
        "no_motion_tasks": no_motion_tasks,
    }


def target_labels_csv():
    return ",".join(C4_TARGET_LABELS)


def execution_semantics_for_label(label):
    return C4_EXECUTION_SEMANTICS.get(str(label or "").strip().upper())


def label_to_task_mapping():
    return dict(C4_LABEL_TO_TASK)


def task_names():
    return set(C4_SAFE_GAZEBO_TASKS)


def task_moves_gazebo_robot_body(task):
    return str(task or "").strip().upper() in C4_MOVING_TASKS


def task_is_safe_no_motion(task):
    return str(task or "").strip().upper() in C4_NO_MOTION_TASKS
