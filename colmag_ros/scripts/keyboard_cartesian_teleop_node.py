#!/usr/bin/env python3
"""Raw-terminal Cartesian input source for Gazebo development TELEOP."""

import json
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros import cartesian_keyboard_input


PUBLISH_PERIOD_SEC = 1.0 / 30.0


def _read_char(descriptor):
    # Unbuffered, so select() never misses bytes already pulled into a buffer.
    return os.read(descriptor, 1).decode("utf-8", "ignore")


def _read_key(descriptor):
    key = _read_char(descriptor)
    if key != "\x1b":
        return key
    suffix = ""
    for _ in range(2):
        ready, _, _ = select.select([descriptor], [], [], 0.02)
        if not ready:
            break
        suffix += _read_char(descriptor)
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
        # Covers the terminal's first auto-repeat delay; 0 publishes a tap as
        # a short pulse instead.
        self.hold = cartesian_keyboard_input.KeyHoldTracker(
            first_repeat_window_sec=float(
                rospy.get_param("~first_repeat_window_sec", 0.6)
            ),
            repeat_window_sec=float(rospy.get_param("~repeat_window_sec", 0.12)),
        )

    def publish_key(self, key):
        if (
            cartesian_keyboard_input.direction_for_key(key) is None
            and key not in cartesian_keyboard_input.PAUSE_KEYS
        ):
            return False
        return self._publish(
            cartesian_keyboard_input.build_cartesian_input_payload(
                key,
                sequence_id=self.sequence_id + 1,
                timestamp=self.rospy.Time.now().to_sec(),
            )
        )

    def publish_release(self):
        return self._publish(
            cartesian_keyboard_input.build_release_payload(
                sequence_id=self.sequence_id + 1,
                timestamp=self.rospy.Time.now().to_sec(),
            )
        )

    def _publish(self, payload):
        if self.require_subscriber and self.publisher.get_num_connections() < 1:
            self.rospy.logwarn_throttle(
                1.0, "No Cartesian adapter subscriber on %s", self.output_topic
            )
            return False
        self.sequence_id = payload["sequence_id"]
        self.publisher.publish(
            self.String(data=json.dumps(payload, sort_keys=True))
        )
        return True

    def handle_key(self, key, now):
        """Process one key; ``False`` when the node should quit."""
        if key in cartesian_keyboard_input.QUIT_KEYS:
            return False
        if key in cartesian_keyboard_input.HELP_KEYS:
            print("\n" + "\n".join(cartesian_keyboard_input.help_lines()))
        elif cartesian_keyboard_input.direction_for_key(key) is not None:
            if self.hold.press(key, now):
                self.publish_key(key)
        elif key in cartesian_keyboard_input.PAUSE_KEYS:
            self.hold.clear()
            self.publish_key(key)
        return True

    def tick(self, now):
        """Republish the held key at the publish rate, or report its release."""
        event = self.hold.poll(now)
        if event == cartesian_keyboard_input.HOLD:
            self.publish_key(self.hold.key)
        elif event == cartesian_keyboard_input.RELEASE:
            self.publish_release()

    def run(self, stream=sys.stdin):
        if not stream.isatty():
            raise RuntimeError("keyboard Cartesian teleop requires an interactive TTY")
        for line in cartesian_keyboard_input.help_lines():
            print(line)

        descriptor = stream.fileno()
        original_settings = termios.tcgetattr(descriptor)
        try:
            tty.setcbreak(descriptor)
            next_tick = time.monotonic()
            while not self.rospy.is_shutdown():
                ready, _, _ = select.select(
                    [descriptor], [], [], max(0.0, next_tick - time.monotonic())
                )
                if ready and not self.handle_key(_read_key(descriptor), time.monotonic()):
                    break
                now = time.monotonic()
                if now >= next_tick:
                    next_tick = now + PUBLISH_PERIOD_SEC
                    self.tick(now)
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, original_settings)
            print()


def main():
    import rospy

    rospy.init_node("keyboard_cartesian_teleop_node")
    KeyboardCartesianTeleopNode().run()


if __name__ == "__main__":
    main()
