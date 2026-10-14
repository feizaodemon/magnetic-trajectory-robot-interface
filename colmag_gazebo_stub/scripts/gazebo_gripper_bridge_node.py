#!/usr/bin/env python3
"""Gazebo-only TELEOP gripper bridge: keyboard open/close -> franka_gripper actions.

Commands are admitted only while TELEOP is the observed control mode (fail
closed). The node talks to the simulated franka_gripper action servers only;
it has no real-robot path.
"""

import json
import sys
import threading
from pathlib import Path

import actionlib
import rospy
from franka_gripper.msg import GraspAction, GraspEpsilon, GraspGoal, MoveAction, MoveGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
_COLMAG_ROS_SRC = Path(__file__).resolve().parents[2] / "colmag_ros" / "src"
for _path in (_PACKAGE_SRC, _COLMAG_ROS_SRC):
    if _path.is_dir() and str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from colmag_gazebo_stub import gripper_bridge
from colmag_ros import control_mode


class GazeboGripperBridgeNode:
    def __init__(self):
        self.command_topic = rospy.get_param(
            "~gripper_command_topic", "/colmag/teleop/gripper_command")
        self.state_topic = rospy.get_param("~gripper_state_topic", "/colmag/teleop/gripper_state")
        self.control_mode_topic = rospy.get_param("~control_mode_topic", "/colmag/control_mode")
        self.gripper_ns = rospy.get_param("~gripper_namespace", "/franka_gripper").rstrip("/")
        self.open_width_m = float(rospy.get_param("~open_width_m", 0.08))
        self.grasp_width_m = float(rospy.get_param("~grasp_width_m", 0.04))
        # width +/- epsilon spans fully closed (an empty grasp ends slightly
        # below 0) to fully open, so any object up to 8 cm counts as grasped.
        self.grasp_epsilon_m = float(rospy.get_param("~grasp_epsilon_m", 0.045))
        self.speed_m_s = float(rospy.get_param("~speed_m_s", 0.1))
        self.force_n = float(rospy.get_param("~force_n", 20.0))
        self.holding_min_width_m = float(rospy.get_param("~holding_min_width_m", 0.005))
        self.gate = gripper_bridge.CommandGate(
            max_age_sec=float(rospy.get_param("~command_max_age_sec", 1.0)))

        self.lock = threading.Lock()
        self.current_mode = None
        self.mode_observed = False
        self.width = None
        self.state = gripper_bridge.UNKNOWN
        self.reason = "no_command_yet"
        self.command = ""

        self.move_client = actionlib.SimpleActionClient(self.gripper_ns + "/move", MoveAction)
        self.grasp_client = actionlib.SimpleActionClient(self.gripper_ns + "/grasp", GraspAction)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.control_mode_topic, String, self._handle_mode, queue_size=1)
        rospy.Subscriber(self.gripper_ns + "/joint_states", JointState,
                         self._handle_joint_state, queue_size=1)
        rospy.Subscriber(self.command_topic, String, self._handle_command, queue_size=5)
        rospy.Timer(rospy.Duration(0.2), lambda _event: self._publish())

    def _handle_mode(self, message):
        selected = control_mode.normalize_mode(message.data)
        with self.lock:
            self.current_mode = selected
            self.mode_observed = selected is not None

    def _handle_joint_state(self, message):
        width = gripper_bridge.finger_width(message.name, message.position)
        if width is not None:
            with self.lock:
                self.width = width

    def _handle_command(self, message):
        try:
            command, sequence_id, timestamp = gripper_bridge.parse_command(message.data)
        except ValueError as error:
            self._set_state(gripper_bridge.REJECTED, str(error), "")
            return
        with self.lock:
            reason = self.gate.admit(sequence_id, timestamp, self.current_mode,
                                     self.mode_observed, rospy.Time.now().to_sec())
        if reason:
            self._set_state(gripper_bridge.REJECTED, reason, command)
            return
        client = self.move_client if command == gripper_bridge.OPEN else self.grasp_client
        if not client.wait_for_server(rospy.Duration(0.5)):
            self._set_state(gripper_bridge.FAILED, "gripper_action_unavailable", command)
            return
        if command == gripper_bridge.OPEN:
            goal = MoveGoal(width=self.open_width_m, speed=self.speed_m_s)
            moving = gripper_bridge.OPENING
        else:
            goal = GraspGoal(
                width=self.grasp_width_m,
                epsilon=GraspEpsilon(inner=self.grasp_epsilon_m, outer=self.grasp_epsilon_m),
                speed=self.speed_m_s,
                force=self.force_n,
            )
            moving = gripper_bridge.CLOSING
        # A new goal preempts the other action's goal, if any.
        other = self.grasp_client if client is self.move_client else self.move_client
        other.cancel_all_goals()
        self._set_state(moving, "goal_sent", command)
        client.send_goal(goal, done_cb=lambda status, result, c=command: self._done(c, result))

    def _done(self, command, result):
        succeeded = bool(getattr(result, "success", False))
        with self.lock:
            if self.command != command:
                return
            width = self.width
        state = gripper_bridge.settled_state(
            command, succeeded, width, self.holding_min_width_m)
        reason = "" if succeeded else str(getattr(result, "error", "") or "action_failed")
        self._set_state(state, reason, command)

    def _set_state(self, state, reason, command):
        with self.lock:
            self.state, self.reason, self.command = state, reason, command
        self._publish()

    def _publish(self):
        with self.lock:
            payload = gripper_bridge.state_payload(self.state, self.reason, self.command, self.width)
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("gazebo_gripper_bridge_node")
    GazeboGripperBridgeNode()
    rospy.spin()


if __name__ == "__main__":
    main()
