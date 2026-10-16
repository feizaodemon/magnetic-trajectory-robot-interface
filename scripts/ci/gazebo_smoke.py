#!/usr/bin/env python3
"""Gazebo runtime smoke test for the keyboard TELEOP profile (CI job ``gazebo-smoke``).

Run inside the noetic-gazebo image while ``colmag_keyboard_gazebo_teleop.launch``
is up. It publishes the same payloads as the keyboard terminal and checks:

1. TASK is the startup mode; a gripper command in TASK is rejected.
2. In TELEOP, holding +X moves the flange forward and releasing it stops the robot.
3. The gripper opens and closes through the simulated franka_gripper actions.
4. Back in TASK, the Cartesian adapter reports TASK_MODE and the gripper rejects again.
5. The optional pick objects were spawned.

Gazebo only. Exit code 0 means every check passed.
"""

import json
import sys
import time

import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from colmag_ros.cartesian_keyboard_input import (
    build_cartesian_input_payload,
    build_gripper_payload,
    build_release_payload,
)

MODE_TOPIC = "/colmag/control_mode"
MODE_REQUEST_TOPIC = "/colmag/control_mode/request"
ADAPTER_STATE_TOPIC = "/colmag/gazebo_cartesian_teleop_state"
INPUT_TOPIC = "/colmag/teleop/cartesian_input"
GRIPPER_COMMAND_TOPIC = "/colmag/teleop/gripper_command"
GRIPPER_STATE_TOPIC = "/colmag/teleop/gripper_state"

latest = {"mode": None, "adapter": {}, "gripper": {}, "joints": 0}
adapter_states_seen = set()


def on_mode(message):
    latest["mode"] = message.data


def on_adapter(message):
    payload = json.loads(message.data)
    latest["adapter"] = payload
    adapter_states_seen.add(payload.get("state"))


def on_gripper(message):
    latest["gripper"] = json.loads(message.data)


def on_joints(_message):
    latest["joints"] += 1


def wait_for(what, predicate, timeout):
    """Wait in wall time for ``predicate()``; fail the run on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        if predicate():
            print("ok   %s" % what, flush=True)
            return
        time.sleep(0.1)
    fail("%s (timeout %.0f s; mode=%r adapter=%r gripper=%r)" % (
        what, timeout, latest["mode"], latest["adapter"].get("state"),
        latest["gripper"]))


def fail(message):
    print("FAIL %s" % message, flush=True)
    sys.exit(1)


class Driver:
    def __init__(self):
        self.sequence_id = 0
        self.mode_pub = rospy.Publisher(MODE_REQUEST_TOPIC, String, queue_size=1)
        self.input_pub = rospy.Publisher(INPUT_TOPIC, String, queue_size=10)
        self.gripper_pub = rospy.Publisher(GRIPPER_COMMAND_TOPIC, String, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def next_id(self):
        self.sequence_id += 1
        return self.sequence_id

    def request_mode(self, mode):
        wait_for("%s has a subscriber" % MODE_REQUEST_TOPIC,
                 lambda: self.mode_pub.get_num_connections() > 0, 30)
        self.mode_pub.publish(String(data=mode))
        wait_for("control mode is %s" % mode, lambda: latest["mode"] == mode, 10)

    def flange_x(self):
        transform = self.tf_buffer.lookup_transform(
            "panda_link0", "panda_link8", rospy.Time(0), rospy.Duration(2.0))
        return transform.transform.translation.x

    def hold_key(self, key, seconds):
        """Publish one held key at the terminal repeat rate, then the release."""
        end = time.time() + seconds
        while time.time() < end:
            payload = build_cartesian_input_payload(key, self.next_id(), rospy.Time.now().to_sec())
            self.input_pub.publish(String(data=json.dumps(payload)))
            time.sleep(1.0 / 30.0)
        payload = build_release_payload(self.next_id(), rospy.Time.now().to_sec())
        self.input_pub.publish(String(data=json.dumps(payload)))

    def gripper(self, command, expected_states, timeout=30):
        """Send ``command`` until the bridge settles in one of ``expected_states``.

        A FAILED ``gripper_action_unavailable`` (action server still starting)
        is retried with a new sequence id.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            payload = build_gripper_payload(command, self.next_id(), rospy.Time.now().to_sec())
            self.gripper_pub.publish(String(data=json.dumps(payload)))
            settle = time.time() + 10
            while time.time() < settle:
                state = latest["gripper"]
                if state.get("command") == command and state.get("state") in expected_states:
                    print("ok   gripper %s -> %s %s" % (
                        command, state.get("state"), state.get("width_m")), flush=True)
                    return state
                if state.get("reason") == "gripper_action_unavailable":
                    break
                time.sleep(0.1)
            time.sleep(1.0)
        fail("gripper %s did not reach %s (last %r)" % (
            command, expected_states, latest["gripper"]))


def main():
    rospy.init_node("gazebo_smoke", anonymous=True, disable_signals=True)
    rospy.Subscriber(MODE_TOPIC, String, on_mode, queue_size=5)
    rospy.Subscriber(ADAPTER_STATE_TOPIC, String, on_adapter, queue_size=20)
    rospy.Subscriber(GRIPPER_STATE_TOPIC, String, on_gripper, queue_size=20)
    rospy.Subscriber("/joint_states", JointState, on_joints, queue_size=5)
    driver = Driver()

    wait_for("startup mode is TASK", lambda: latest["mode"] == "TASK", 120)
    wait_for("Gazebo publishes /joint_states", lambda: latest["joints"] > 50, 300)
    wait_for("adapter reports TASK_MODE",
             lambda: latest["adapter"].get("state") == "TASK_MODE", 60)
    wait_for("gripper bridge publishes state", lambda: bool(latest["gripper"]), 60)
    wait_for("gripper publisher connected",
             lambda: driver.gripper_pub.get_num_connections() > 0, 30)

    driver.gripper("close", ("REJECTED",), timeout=15)
    if latest["gripper"].get("reason") != "teleop_not_owner":
        fail("TASK gripper rejection reason %r" % latest["gripper"].get("reason"))

    driver.request_mode("TELEOP")
    wait_for("adapter waits for input in TELEOP",
             lambda: latest["adapter"].get("state") == "WAITING_FOR_INPUT", 20)
    wait_for("input publisher connected",
             lambda: driver.input_pub.get_num_connections() > 0, 30)
    start_x = driver.flange_x()
    adapter_states_seen.clear()
    driver.hold_key("w", 2.0)
    if "COMMAND_PUBLISHED" not in adapter_states_seen:
        fail("no COMMAND_PUBLISHED while +X was held (saw %r)" % sorted(adapter_states_seen))
    print("ok   adapter published commands while +X was held", flush=True)
    wait_for("robot comes to rest after release",
             lambda: latest["adapter"].get("state") == "PAUSED", 10)
    moved = driver.flange_x() - start_x
    if moved < 0.03:
        fail("flange moved only %.3f m in +X" % moved)
    print("ok   flange moved %.3f m in +X" % moved, flush=True)

    opened = driver.gripper("open", ("OPEN",))
    if not opened.get("width_m") or opened["width_m"] < 0.07:
        fail("open width %r" % opened.get("width_m"))
    driver.gripper("close", ("CLOSED", "HOLDING"))

    driver.request_mode("TASK")
    wait_for("adapter reports TASK_MODE again",
             lambda: latest["adapter"].get("state") == "TASK_MODE", 10)
    driver.gripper("open", ("REJECTED",), timeout=15)

    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    for model in ("pick_cube", "place_pad"):
        if not get_state(model, "world").success:
            fail("model %s was not spawned" % model)
        print("ok   model %s exists" % model, flush=True)

    print("PASS gazebo smoke", flush=True)


if __name__ == "__main__":
    main()
