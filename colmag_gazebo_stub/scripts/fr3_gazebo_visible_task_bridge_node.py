#!/usr/bin/env python3
import copy
import json
import math
import sys
import threading
import time
from pathlib import Path

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

PANDA_JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]

JOINT_LIMITS = [
    (-1.0, 1.0),
    (-1.4, 0.2),
    (-1.0, 1.0),
    (-2.8, -1.2),
    (-1.0, 1.0),
    (0.5, 2.2),
    (-1.2, 1.2),
]

ACTION_PENDING = 0
ACTION_ACTIVE = 1
ACTION_SUCCEEDED = 3
GOAL_TOLERANCE_VIOLATED = -5
DEFAULT_FOLLOW_JOINT_TRAJECTORY_ACTION = "/position_joint_trajectory_controller/follow_joint_trajectory"


_COLMAG_ROS_SRC = Path(__file__).resolve().parents[2] / "colmag_ros" / "src"
if _COLMAG_ROS_SRC.is_dir() and str(_COLMAG_ROS_SRC) not in sys.path:
    sys.path.insert(0, str(_COLMAG_ROS_SRC))

from colmag_ros import m104c4_execution_semantics

_C4_CONTRACT = m104c4_execution_semantics.validated_execution_contract()
C4_NO_MOTION_TASKS = set(_C4_CONTRACT["no_motion_tasks"])

HOME_OR_READY_POSITIONS = [0.0, -0.4, 0.0, -2.1, 0.0, 1.6, 0.8]


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_task_command(data):
    try:
        payload = json.loads(data)
    except ValueError:
        return None, "invalid_json"
    if not isinstance(payload, dict):
        return None, "payload_not_object"
    return payload, ""


class Fr3GazeboVisibleTaskBridgeNode:
    """FR3-oriented Gazebo-visible task bridge.

    This node bridges high-level semantic task commands to Gazebo placeholder motions.
    The underlying Gazebo asset is currently Panda-based, so joint names use panda_joint*.
    This does not prove real FR3 fidelity.
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~task_command_topic", "/colmag/task_command")
        self.output_topic = rospy.get_param("~demo_task_state_topic", "/colmag/fr3_demo_task_state")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        legacy_action_server = rospy.get_param("~action_server", DEFAULT_FOLLOW_JOINT_TRAJECTORY_ACTION)
        self.action_server = rospy.get_param("~follow_joint_trajectory_action", legacy_action_server)
        self.allow_missing_gazebo_flag = bool_param(rospy.get_param("~allow_missing_gazebo_flag", False))
        self.server_wait_sec = float(rospy.get_param("~server_wait_sec", 3.0))
        self.callback_server_wait_sec = float(rospy.get_param("~callback_server_wait_sec", 0.2))
        self.joint_state_max_age_sec = float(rospy.get_param("~joint_state_max_age_sec", 2.0))
        self.goal_timeout_sec = float(rospy.get_param("~goal_timeout_sec", 10.0))
        self.goal_tolerance_rad = float(rospy.get_param("~goal_tolerance_rad", 0.01))
        self.path_tolerance_rad = float(rospy.get_param("~path_tolerance_rad", 0.05))
        self.goal_time_tolerance_sec = float(rospy.get_param("~goal_time_tolerance_sec", 1.0))
        self.readiness_timeout_sec = float(rospy.get_param("~readiness_timeout_sec", 15.0))
        self.motion_duration_sec = float(rospy.get_param("~motion_duration_sec", 2.0))
        self.home_or_ready_motion_duration_sec = float(rospy.get_param("~home_or_ready_motion_duration_sec", 4.0))
        self.home_or_ready_final_hold_sec = float(rospy.get_param("~home_or_ready_final_hold_sec", 1.0))
        self.home_or_ready_bounded_success_enabled = bool_param(
            rospy.get_param("~home_or_ready_bounded_success_enabled", True)
        )
        self.home_or_ready_bounded_success_tolerance_rad = float(
            rospy.get_param("~home_or_ready_bounded_success_tolerance_rad", 0.01)
        )
        self.visible_motion_scale = float(rospy.get_param("~visible_motion_scale", 1.0))
        self.queue_until_ready = bool_param(rospy.get_param("~queue_until_ready", True))
        self.allow_initial_pose_fallback = bool_param(rospy.get_param("~allow_initial_pose_fallback", False))
        self.control_mode_topic = str(rospy.get_param("~control_mode_topic", "")).strip()
        self.required_control_mode = str(
            rospy.get_param("~required_control_mode", "")
        ).strip().upper()
        if self.required_control_mode not in ("", "TASK"):
            raise ValueError("~required_control_mode must be TASK or empty")
        if self.required_control_mode and not self.control_mode_topic:
            raise ValueError("~control_mode_topic is required for mode-managed execution")

        self.latest_joint_positions = None
        self.latest_joint_state_wall_time = 0.0
        self.server_ready = False

        self.queued_task = None
        self.queued_task_payload = None
        self.queued_time = 0.0
        self.queue_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.mode_lock = threading.Lock()
        self.goal_active = False
        self.current_control_mode = None
        self.control_mode_observed = False

        self.status_pub = rospy.Publisher(self.output_topic, String, queue_size=10, latch=True)
        self.client = actionlib.SimpleActionClient(self.action_server, FollowJointTrajectoryAction)

        rospy.Subscriber(self.joint_state_topic, JointState, self._handle_joint_state, queue_size=10)
        rospy.Subscriber(self.input_topic, String, self._handle_task_command, queue_size=10)
        if self.required_control_mode:
            rospy.Subscriber(
                self.control_mode_topic,
                String,
                self._handle_control_mode,
                queue_size=1,
            )

        rospy.loginfo(
            "fr3_gazebo_visible_task_bridge_node started: follow_joint_trajectory_action=%s "
            "goal_tolerance_rad=%.4f path_tolerance_rad=%.4f goal_time_tolerance_sec=%.2f "
            "home_or_ready_motion_duration_sec=%.2f home_or_ready_final_hold_sec=%.2f "
            "home_or_ready_bounded_success_enabled=%s "
            "home_or_ready_bounded_success_tolerance_rad=%.4f",
            self.action_server,
            self.goal_tolerance_rad,
            self.path_tolerance_rad,
            self.goal_time_tolerance_sec,
            self.home_or_ready_motion_duration_sec,
            self.home_or_ready_final_hold_sec,
            self.home_or_ready_bounded_success_enabled,
            self.home_or_ready_bounded_success_tolerance_rad,
        )
        self._publish_status("waiting_for_server", task="NO_OP", phase="waiting_for_action_server")
        self._refresh_server_ready(self.server_wait_sec)

        # Start a background thread to process the queued task once ready
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def _publish_status(self, status, task, phase, reason="", **extra):
        payload = {
            "timestamp": time.time(),
            "status": status,
            "task": task,
            "phase": phase,
            "reason": reason,
            "robot_family": "FR3_GAZEBO_PLACEHOLDER",
            "controls_real_robot": False,
            "gazebo_only": True,
            "placeholder_only": True,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        rospy.loginfo("fr3_demo_task_state=%s", msg.data)

    def _refresh_server_ready(self, timeout_sec):
        if self.server_ready:
            return True
        ready = self.client.wait_for_server(rospy.Duration(max(0.0, timeout_sec)))
        self.server_ready = bool(ready)
        return self.server_ready

    def _send_goal(self, goal):
        with self.mode_lock:
            if not self._mode_allows_task_unlocked():
                return False
            with self.goal_lock:
                self.client.send_goal(goal)
                self.goal_active = True
        return True

    def _claim_cancellable_goal(self):
        with self.goal_lock:
            if not self.goal_active:
                return False
            self.goal_active = False
            return self.client.get_state() in (ACTION_PENDING, ACTION_ACTIVE)

    def _clear_active_goal(self):
        with self.goal_lock:
            self.goal_active = False

    def _mode_allows_task(self):
        with self.mode_lock:
            return self._mode_allows_task_unlocked()

    def _mode_allows_task_unlocked(self):
        return not self.required_control_mode or (
            self.control_mode_observed
            and self.current_control_mode == self.required_control_mode
        )

    def _handle_control_mode(self, message):
        selected = str(message.data or "").strip().upper()
        if selected not in ("TASK", "TELEOP"):
            selected = None
        with self.mode_lock:
            previously_allowed = self._mode_allows_task_unlocked()
            self.current_control_mode = selected
            self.control_mode_observed = selected is not None
            relinquish = previously_allowed and not self._mode_allows_task_unlocked()
        if relinquish:
            with self.queue_lock:
                self.queued_task = None
                self.queued_task_payload = None
            cancelled = self._claim_cancellable_goal()
            if cancelled:
                self.client.cancel_goal()
            self._publish_status(
                "task_ownership_relinquished",
                task="NO_OP",
                phase="control_mode_transition",
                reason="owned_goal_cancel_requested" if cancelled else "no_owned_active_goal",
            )

    def _handle_joint_state(self, message):
        positions_by_name = dict(zip(message.name, message.position))
        try:
            positions = [float(positions_by_name[name]) for name in PANDA_JOINTS]
        except (KeyError, TypeError, ValueError):
            return
        self.latest_joint_positions = positions
        self.latest_joint_state_wall_time = time.time()

    def _current_joint_positions(self):
        if self.latest_joint_positions is None:
            return None
        age = time.time() - self.latest_joint_state_wall_time
        if age > self.joint_state_max_age_sec:
            return None
        return list(self.latest_joint_positions)

    def _clamp(self, value, limits):
        low, high = limits
        return min(max(value, low), high)

    def _clamp_positions(self, positions):
        return [self._clamp(value, limits) for value, limits in zip(positions, JOINT_LIMITS)]

    def _offset(self, positions, deltas):
        target = copy.copy(positions)
        for index, delta in deltas.items():
            target[index] += delta * self.visible_motion_scale
        return self._clamp_positions(target)

    def _get_task_waypoints(self, task, current_positions):
        if current_positions is None:
            return None, []

        if task == "HEXAGON_TRAJECTORY":
            return [
                self._offset(current_positions, {0: 0.10, 5: 0.12}),
                self._offset(current_positions, {0: 0.05, 5: 0.20}),
                self._offset(current_positions, {0: -0.05, 5: 0.20}),
                self._offset(current_positions, {0: -0.10, 5: 0.12}),
                self._offset(current_positions, {0: -0.05, 5: 0.05}),
                current_positions,
            ], ["planning_hexagon_placeholder", "waypoint_1", "waypoint_2", "waypoint_3", "waypoint_4", "waypoint_5", "waypoint_6"]
        elif task == "PICK_PLACE":
            return [
                self._offset(current_positions, {1: -0.10, 3: 0.10}),
                self._offset(current_positions, {1: 0.00, 3: 0.00}),
                self._offset(current_positions, {1: -0.10, 3: -0.10}),
                current_positions,
            ], ["planning_pick_place_placeholder", "approach_stub", "grasp_stub", "place_stub", "release_stub"]
        elif task == "COMPLIANT_CONTROL":
            return [
                self._offset(current_positions, {5: 0.05, 6: -0.05}),
                self._offset(current_positions, {5: -0.05, 6: 0.05}),
                current_positions,
            ], ["planning_compliance_placeholder", "small_compliant_motion_stub_1", "small_compliant_motion_stub_2", "small_compliant_motion_stub_3"]
        elif task == "AVOID_OBSTACLE":
            return [
                self._offset(current_positions, {0: 0.20}),
                self._offset(current_positions, {0: -0.20}),
                current_positions,
            ], ["planning_avoidance_placeholder", "avoidance_left_stub", "avoidance_right_stub", "avoidance_return_stub"]
        elif task == "MOVE_LEFT":
            return [
                self._offset(current_positions, {0: 0.20}),
                current_positions,
            ], ["planning_directional_placeholder", "waypoint_1", "waypoint_2"]
        elif task == "MOVE_RIGHT":
            return [
                self._offset(current_positions, {0: -0.20}),
                current_positions,
            ], ["planning_directional_placeholder", "waypoint_1", "waypoint_2"]
        elif task == "MOVE_UP":
            return [
                self._offset(current_positions, {1: -0.20, 3: 0.10}),
                current_positions,
            ], ["planning_directional_placeholder", "waypoint_1", "waypoint_2"]
        elif task == "MOVE_DOWN":
            return [
                self._offset(current_positions, {1: 0.10, 3: -0.05}),
                current_positions,
            ], ["planning_directional_placeholder", "waypoint_1", "waypoint_2"]
        elif task == "HOVER_APPROACH":
            return [
                self._offset(current_positions, {1: -0.12, 3: 0.08, 5: 0.05}),
                self._offset(current_positions, {1: -0.06, 3: 0.04, 5: 0.02}),
                current_positions,
            ], ["planning_hover_approach_preview", "hover_lift_preview", "hover_settle_preview", "hover_return_preview"]
        elif task == "ORBIT_SMALL":
            return [
                self._offset(current_positions, {0: 0.08, 5: 0.08}),
                self._offset(current_positions, {0: 0.00, 5: 0.14, 6: 0.05}),
                self._offset(current_positions, {0: -0.08, 5: 0.08}),
                self._offset(current_positions, {0: 0.00, 5: 0.02, 6: -0.05}),
                current_positions,
            ], ["planning_orbit_small_preview", "orbit_waypoint_1", "orbit_waypoint_2", "orbit_waypoint_3", "orbit_waypoint_4", "orbit_return_preview"]
        elif task == "HOME_OR_READY":
            ready = self._clamp_positions(list(HOME_OR_READY_POSITIONS))
            midpoint = self._clamp_positions([(a + b) / 2.0 for a, b in zip(current_positions, ready)])
            return [
                midpoint,
                ready,
            ], ["planning_home_or_ready_preview", "ready_midpoint_preview", "ready_pose_preview"]
        elif task == "SOFT_DESCEND_PREVIEW":
            return [
                self._offset(current_positions, {1: 0.06, 3: -0.04}),
                self._offset(current_positions, {1: 0.03, 3: -0.02}),
                current_positions,
            ], ["planning_soft_descend_non_contact_preview", "soft_descend_preview", "soft_settle_preview", "soft_return_preview"]
        return None, []

    def _build_joint_tolerances(self, tolerance):
        joint_tolerances = []
        for name in PANDA_JOINTS:
            joint_tolerance = JointTolerance()
            joint_tolerance.name = name
            joint_tolerance.position = tolerance
            joint_tolerance.velocity = 0.0
            joint_tolerance.acceleration = 0.0
            joint_tolerances.append(joint_tolerance)
        return joint_tolerances

    def _build_goal(self, current_positions, waypoints, task=""):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(PANDA_JOINTS)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        all_points = [self._clamp_positions(current_positions)] + [
            self._clamp_positions(point) for point in waypoints
        ]
        if len(all_points) == 1:
            all_points.append(all_points[0])

        motion_duration_sec = self.motion_duration_sec
        final_hold_sec = 0.0
        if task == "HOME_OR_READY":
            motion_duration_sec = max(self.motion_duration_sec, self.home_or_ready_motion_duration_sec)
            final_hold_sec = max(0.0, self.home_or_ready_final_hold_sec)
            if final_hold_sec > 0.0:
                all_points.append(list(all_points[-1]))

        step_duration = max(0.2, motion_duration_sec / max(1, len(all_points) - 1))
        for index, positions in enumerate(all_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(PANDA_JOINTS)
            point_time_sec = 0.2 + step_duration * index
            if task == "HOME_OR_READY" and final_hold_sec > 0.0 and index == len(all_points) - 1:
                point_time_sec += final_hold_sec
            point.time_from_start = rospy.Duration(point_time_sec)
            goal.trajectory.points.append(point)

        goal_tolerance = max(0.0, self.goal_tolerance_rad)
        path_tolerance = max(0.0, self.path_tolerance_rad)
        goal.goal_tolerance = self._build_joint_tolerances(goal_tolerance)
        goal.path_tolerance = self._build_joint_tolerances(path_tolerance)
        goal.goal_time_tolerance = rospy.Duration(max(0.0, self.goal_time_tolerance_sec))

        if task == "HOME_OR_READY":
            rospy.loginfo(
                "HOME_OR_READY goal config: points=%d motion_duration_sec=%.2f "
                "final_hold_sec=%.2f goal_tolerance_rad=%.4f path_tolerance_rad=%.4f "
                "goal_time_tolerance_sec=%.2f",
                len(goal.trajectory.points),
                motion_duration_sec,
                final_hold_sec,
                goal_tolerance,
                path_tolerance,
                self.goal_time_tolerance_sec,
            )

        return goal

    def _handle_task_command(self, message):
        task_payload, reason = parse_task_command(message.data)
        if task_payload is None:
            self._publish_status("rejected", task="NO_OP", phase="rejected", reason=reason)
            return

        task = str(task_payload.get("task", "NO_OP")).strip().upper() or "NO_OP"

        if not self._mode_allows_task():
            self._publish_status(
                "rejected",
                task=task,
                phase="control_mode_rejected",
                reason=(
                    "task_mode_not_selected"
                    if self.control_mode_observed
                    else "control_mode_not_observed"
                ),
            )
            return

        # Safety checks
        if bool_param(task_payload.get("controls_real_robot", False)):
            self._publish_status("rejected", task=task, phase="rejected", reason="real_robot_not_supported_in_gazebo_bridge")
            return

        gazebo_only = bool_param(task_payload.get("gazebo_only", False))
        if not gazebo_only and not self.allow_missing_gazebo_flag:
            self._publish_status("rejected", task=task, phase="rejected", reason="gazebo_only_flag_missing_or_false")
            return

        accepted = bool_param(task_payload.get("accepted", task != "NO_OP"))
        if not accepted:
            self._publish_status("rejected", task=task, phase="rejected", reason="task_command_not_accepted")
            return

        self._publish_status("received", task=task, phase="received")

        if task == "STOP" or task in C4_NO_MOTION_TASKS:
            self._publish_status("running", task=task, phase="stop_requested")
            if self._claim_cancellable_goal():
                self.client.cancel_goal()
            self._publish_status("succeeded", task=task, phase="succeeded", no_motion=True)
            return

        with self.queue_lock:
            self.queued_task = task
            self.queued_task_payload = task_payload
            self.queued_time = time.time()
            if self.queue_until_ready:
                self._publish_status("waiting", task=task, phase="queued_until_ready")

    def _worker_loop(self):
        last_publish_time = 0.0
        while not rospy.is_shutdown():
            task_to_execute = None
            task_payload_to_execute = None
            with self.queue_lock:
                if self.queued_task:
                    # check timeout
                    if time.time() - self.queued_time > self.readiness_timeout_sec:
                        self._publish_status("failed", task=self.queued_task, phase="failed", reason="readiness_timeout_exceeded")
                        self.queued_task = None
                        self.queued_task_payload = None
                        continue

                    # Check readiness
                    server_ready = self._refresh_server_ready(0.0)
                    joint_positions = self._current_joint_positions()

                    if not server_ready or joint_positions is None:
                        if time.time() - last_publish_time > 1.0:
                            if not server_ready:
                                self._publish_status("waiting", task=self.queued_task, phase="waiting_for_action_server")
                            elif joint_positions is None:
                                self._publish_status("waiting", task=self.queued_task, phase="waiting_for_joint_states")
                            last_publish_time = time.time()

                        if not server_ready:
                            pass
                        elif joint_positions is None and self.allow_initial_pose_fallback:
                            rospy.logwarn("Using initial pose fallback for joint states!")
                            joint_positions = [0.0] * len(PANDA_JOINTS)

                    if server_ready and joint_positions is not None:
                        task_to_execute = self.queued_task
                        task_payload_to_execute = self.queued_task_payload
                        self.queued_task = None
                        self.queued_task_payload = None

            if task_to_execute:
                if self._mode_allows_task():
                    self._execute_task(task_to_execute, joint_positions, task_payload_to_execute)
                else:
                    self._publish_status(
                        "rejected",
                        task=task_to_execute,
                        phase="control_mode_rejected",
                        reason="task_mode_not_selected",
                    )

            time.sleep(0.1)

    def _home_or_ready_within_gazebo_tolerance(self, task, waypoints, task_payload, result):
        if task != "HOME_OR_READY" or not self.home_or_ready_bounded_success_enabled:
            return False, {}
        if not isinstance(task_payload, dict):
            return False, {}
        if bool_param(task_payload.get("controls_real_robot", False)):
            return False, {}
        if not bool_param(task_payload.get("gazebo_only", False)):
            return False, {}
        if int(getattr(result, "error_code", 0)) != GOAL_TOLERANCE_VIOLATED:
            return False, {}

        actual_positions = self._current_joint_positions()
        if actual_positions is None or not waypoints:
            return False, {}
        final_target = self._clamp_positions(waypoints[-1])
        errors = [abs(actual - target) for actual, target in zip(actual_positions, final_target)]
        max_error = max(errors) if errors else float("inf")
        tolerance = max(0.0, self.home_or_ready_bounded_success_tolerance_rad)
        details = {
            "result_error_code": int(getattr(result, "error_code", 0)),
            "max_joint_error_rad": max_error,
            "gazebo_success_tolerance_rad": tolerance,
            "joint_errors_rad": dict(zip(PANDA_JOINTS, errors)),
        }
        return max_error <= tolerance, details

    def _execute_task(self, task, current_positions, task_payload=None):
        if not self._mode_allows_task():
            self._publish_status(
                "rejected",
                task=task,
                phase="control_mode_rejected",
                reason="task_mode_not_selected",
            )
            return
        waypoints, phases = self._get_task_waypoints(task, current_positions)
        if waypoints is None:
            reason = "no_op_task" if task == "NO_OP" else "unsupported_task"
            self._publish_status("rejected", task=task, phase="rejected", reason=reason)
            return

        # Start sequence
        self._publish_status("running", task=task, phase="executing")
        self._publish_status("running", task=task, phase=phases[0])

        goal = self._build_goal(current_positions, waypoints, task=task)
        try:
            if not self._send_goal(goal):
                self._publish_status(
                    "rejected",
                    task=task,
                    phase="control_mode_rejected",
                    reason="task_mode_not_selected",
                )
                return

            for i in range(1, len(phases)):
                self._publish_status("running", task=task, phase=phases[i])
                rospy.sleep(self.motion_duration_sec / max(1, len(phases)))

            finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout_sec))
            if not finished:
                if self._claim_cancellable_goal():
                    self.client.cancel_goal()
                self._publish_status("failed", task=task, phase="failed", reason="goal_timeout")
                return

            self._clear_active_goal()
            state = self.client.get_state()
            if state == ACTION_SUCCEEDED:
                self._publish_status("succeeded", task=task, phase="succeeded")
            else:
                result = self.client.get_result()
                within_tolerance, tolerance_details = self._home_or_ready_within_gazebo_tolerance(
                    task, waypoints, task_payload, result
                )
                if within_tolerance:
                    self._publish_status(
                        "succeeded",
                        task=task,
                        phase="succeeded_within_gazebo_tolerance",
                        reason="goal_tolerance_violated_but_joint_states_within_tolerance",
                        **tolerance_details,
                    )
                    return
                self._publish_status("failed", task=task, phase="failed", reason=f"action_state_{state}")
        except Exception as exc:
            self._publish_status("failed", task=task, phase="failed", reason=str(exc))
        finally:
            self._clear_active_goal()

def main():
    rospy.init_node("fr3_gazebo_visible_task_bridge_node")
    node = Fr3GazeboVisibleTaskBridgeNode()
    rospy.spin()

if __name__ == "__main__":
    main()
