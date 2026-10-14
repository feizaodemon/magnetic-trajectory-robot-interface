#!/usr/bin/env python3
"""Closed-loop keyboard pick-and-place driver for the README demo recording.

Types into the keyboard teleop terminal with xdotool the way a person holds a
key: key down, X server auto-repeat (set to 0.5 s delay and 30 Hz by the
record script), key up. A hold is
released once the flange (``panda_link8`` from tf) is within its braking
distance of the target; a short corrective hold follows while the error is
above the tolerance. ``O`` and ``C`` are single presses.

Usage: pick_and_place_keys.py <keys.txt> [record start time]
Writes one ``start end label`` line per key, relative to the record start, for
the GIF key labels. Gazebo only.
"""

import json
import os
import subprocess
import sys
import time

import rospy
import tf2_ros
from std_msgs.msg import String

TERMINAL_NAME = "keyboard teleop"
AXIS = {"x": 0, "y": 1, "z": 2}
KEYS = {("x", 1): "w", ("x", -1): "s", ("y", 1): "a", ("y", -1): "d",
        ("z", 1): "q", ("z", -1): "e"}
LABEL = {"w": "W  +X", "s": "S  -X", "a": "A  +Y", "d": "D  -Y", "q": "Q  +Z",
         "e": "E  -Z", "o": "O  open", "c": "C  close"}
# Flange travel after a release from speed v is about v * BRAKE_S
# (measured about 6 cm from the 0.1 m/s keyboard speed).
BRAKE_S = 0.62

# Flange (panda_link8) targets in metres. The TCP is 0.1034 m below the
# flange, so GRASP_Z puts the fingers around the middle of the 5 cm cube.
CUBE_XY = (float(os.environ.get("CUBE_X", "0.45")), float(os.environ.get("CUBE_Y", "-0.15")))
PAD_XY = (float(os.environ.get("PAD_X", "0.45")), float(os.environ.get("PAD_Y", "0.15")))
GRASP_Z = 0.135
PLACE_Z = 0.142
LIFT_Z = 0.30
XY_TOL = 0.02


class Driver:
    def __init__(self, record_start):
        self.record_start = record_start
        self.spans = []
        self.gripper = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/colmag/teleop/gripper_state", String,
                         lambda m: self.gripper.update(json.loads(m.data)), queue_size=5)
        self.window = subprocess.check_output(
            ["xdotool", "search", "--onlyvisible", "--name", TERMINAL_NAME]).split()[0].decode()

    def now(self):
        return time.time() - self.record_start

    def flange(self):
        t = self.tf_buffer.lookup_transform(
            "panda_link0", "panda_link8", rospy.Time(0), rospy.Duration(1.0)).transform.translation
        return (t.x, t.y, t.z)

    def sample(self, history):
        """Append ``(time, position)`` and return position and velocity over ~0.1 s."""
        history.append((time.time(), self.flange()))
        t0, p0 = next((h for h in history if history[-1][0] - h[0] <= 0.1), history[0])
        t1, p1 = history[-1]
        dt = max(t1 - t0, 1e-3)
        return p1, tuple((b - a) / dt for a, b in zip(p0, p1))

    def xdo(self, *args):
        subprocess.run(["xdotool"] + list(args), check=True)

    def hold_once(self, axis, target):
        a = AXIS[axis]
        sign = 1 if target > self.flange()[a] else -1
        key = KEYS[(axis, sign)]
        start = self.now()
        # Key down; the X server auto-repeats it (xset r rate 500 30) until key up.
        self.xdo("windowfocus", self.window)
        self.xdo("keydown", key)
        history = []
        deadline = time.time() + 20.0
        try:
            while True:
                p, v = self.sample(history)
                if (target - p[a]) * sign <= abs(v[a]) * BRAKE_S + 0.002:
                    break
                if time.time() > deadline:
                    raise RuntimeError("%s did not reach %s=%.3f" % (LABEL[key], axis, target))
                time.sleep(0.02)
        finally:
            self.xdo("keyup", key)
        self.spans.append((start, self.now(), LABEL[key]))
        history = []
        deadline = time.time() + 10.0
        while time.time() < deadline:
            p, v = self.sample(history)
            if len(history) > 5 and max(abs(c) for c in v) < 0.003:
                break
            time.sleep(0.05)
        print("%-6s target %s=%.3f reached %.3f" % (LABEL[key], axis, target, p[a]), flush=True)

    def move(self, axis, target, tol=0.012, tries=3):
        for _ in range(tries):
            if abs(target - self.flange()[AXIS[axis]]) <= tol:
                return
            self.hold_once(axis, target)

    def press(self, key, settle):
        start = self.now()
        self.xdo("windowfocus", self.window)
        self.xdo("key", key)
        time.sleep(settle)
        self.spans.append((start, start + 1.2, LABEL[key]))
        print("%-6s gripper %s %s" % (LABEL[key], self.gripper.get("state"),
                                      self.gripper.get("width_m")), flush=True)

    def run(self):
        self.press("o", 1.8)
        # The fingers close along y, so a 2 cm horizontal error still grasps the cube;
        # a shorter correction would overshoot (a tap moves the robot about 4 cm).
        self.move("x", CUBE_XY[0], tol=XY_TOL)
        self.move("y", CUBE_XY[1], tol=XY_TOL)
        self.move("z", GRASP_Z)
        self.press("c", 2.5)
        self.move("z", LIFT_Z, tol=0.05)
        self.move("y", PAD_XY[1], tol=XY_TOL)
        self.move("z", PLACE_Z)
        self.press("o", 1.8)
        self.move("z", LIFT_Z, tol=0.05)


def main():
    keys_path = sys.argv[1]
    record_start = float(sys.argv[2]) if len(sys.argv) > 2 else time.time()
    rospy.init_node("pick_and_place_keys", anonymous=True, disable_signals=True)
    driver = Driver(record_start)
    driver.flange()
    driver.run()
    with open(keys_path, "w") as out:
        for start, end, label in driver.spans:
            out.write("%.2f %.2f %s\n" % (start, end, label))


if __name__ == "__main__":
    main()
