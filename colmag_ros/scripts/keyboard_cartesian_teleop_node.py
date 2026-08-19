#!/usr/bin/env python3
"""Raw-terminal Cartesian input source for Gazebo development TELEOP."""

import json
import select
import sys
import termios
import tty
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros import cartesian_keyboard_input


def _read_key(stream):
    key = stream.read(1)
    if key != "\x1b":
        return key
    suffix = ""
    for _ in range(2):
        ready, _, _ = select.select([stream], [], [], 0.02)
        if not ready:
            break
        suffix += stream.read(1)
    return key + suffix


class KeyboardCartesianTeleopNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.output_topic = rospy.get_param(
            "~cartesian_input_topic", "/colmag/teleop/cartesian_input"
        )
        self.require_subscriber = bool(
            rospy.get_param("~require_subscriber", True)
        )
        self.publisher = rospy.Publisher(
            self.output_topic, String, queue_size=10, latch=False
        )
        self.sequence_id = 0

    def publish_key(self, key):
        if (
            cartesian_keyboard_input.direction_for_key(key) is None
            and key not in cartesian_keyboard_input.PAUSE_KEYS
        ):
            return False
        if self.require_subscriber and self.publisher.get_num_connections() < 1:
            self.rospy.logwarn_throttle(
                1.0, "No Cartesian adapter subscriber on %s", self.output_topic
            )
            return False
        self.sequence_id += 1
        payload = cartesian_keyboard_input.build_cartesian_input_payload(
            key,
            sequence_id=self.sequence_id,
            timestamp=self.rospy.Time.now().to_sec(),
        )
        self.publisher.publish(
            self.String(data=json.dumps(payload, sort_keys=True))
        )
        return True

    def run(self, stream=sys.stdin):
        if not stream.isatty():
            raise RuntimeError("keyboard Cartesian teleop requires an interactive TTY")
        for line in cartesian_keyboard_input.help_lines():
            print(line)

        descriptor = stream.fileno()
        original_settings = termios.tcgetattr(descriptor)
        try:
            tty.setcbreak(descriptor)
            while not self.rospy.is_shutdown():
                key = _read_key(stream)
                if key in cartesian_keyboard_input.QUIT_KEYS:
                    break
                if key in cartesian_keyboard_input.HELP_KEYS:
                    print("\n" + "\n".join(cartesian_keyboard_input.help_lines()))
                    continue
                self.publish_key(key)
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, original_settings)
            print()


def main():
    import rospy

    rospy.init_node("keyboard_cartesian_teleop_node")
    KeyboardCartesianTeleopNode().run()


if __name__ == "__main__":
    main()
