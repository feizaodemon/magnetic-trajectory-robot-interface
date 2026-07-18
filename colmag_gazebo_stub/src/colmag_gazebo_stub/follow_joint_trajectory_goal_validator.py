"""ROS-free FollowJointTrajectory goal-shape validation helpers."""

from dataclasses import dataclass
import math


DEFAULT_EXPECTED_JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

SUCCESSFUL = 0
INVALID_GOAL = -1
INVALID_JOINTS = -2


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    error_code: int = SUCCESSFUL
    error_string: str = ""


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def duration_to_sec(value):
    if value is None:
        return 0.0
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    if hasattr(value, "secs") or hasattr(value, "nsecs"):
        return float(getattr(value, "secs", 0)) + float(getattr(value, "nsecs", 0)) / 1e9
    return float(value)


def _finite_duration(value):
    try:
        seconds = duration_to_sec(value)
    except (TypeError, ValueError):
        return None
    return seconds if math.isfinite(seconds) else None


def validate_follow_joint_trajectory_goal(
    goal,
    expected_joint_names=DEFAULT_EXPECTED_JOINT_NAMES,
    require_exact_joint_names=True,
    min_duration=0.0,
    max_duration=30.0,
    max_points=200,
    allow_empty_goal=False,
):
    trajectory = getattr(goal, "trajectory", None)
    if trajectory is None:
        return ValidationResult(False, INVALID_GOAL, "goal missing trajectory")

    joint_names = list(getattr(trajectory, "joint_names", []) or [])
    points = list(getattr(trajectory, "points", []) or [])

    if not joint_names and not bool_param(allow_empty_goal):
        return ValidationResult(False, INVALID_JOINTS, "trajectory joint_names are required")

    if not joint_names and points:
        return ValidationResult(False, INVALID_JOINTS, "points require joint_names")

    expected_joint_names = list(expected_joint_names or [])
    if bool_param(require_exact_joint_names) and joint_names != expected_joint_names:
        return ValidationResult(False, INVALID_JOINTS, "trajectory joint_names do not match expected_joint_names")

    if len(points) > int(max_points):
        return ValidationResult(False, INVALID_GOAL, "trajectory has too many points")

    previous_time = None
    for index, point in enumerate(points):
        point_time = _finite_duration(getattr(point, "time_from_start", 0.0))
        if point_time is None:
            return ValidationResult(False, INVALID_GOAL, "point %d has invalid time_from_start" % index)
        if previous_time is not None and point_time < previous_time:
            return ValidationResult(False, INVALID_GOAL, "time_from_start must be nondecreasing")
        previous_time = point_time

        positions = getattr(point, "positions", [])
        if positions and len(list(positions)) != len(joint_names):
            return ValidationResult(False, INVALID_GOAL, "point %d positions length does not match joint_names" % index)

    if points:
        final_duration = previous_time if previous_time is not None else 0.0
        if final_duration < float(min_duration) or final_duration > float(max_duration):
            return ValidationResult(False, INVALID_GOAL, "final duration outside configured bounds")

    return ValidationResult(True)
