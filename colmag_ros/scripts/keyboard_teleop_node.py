#!/usr/bin/env python3
"""Interactive terminal source for optional task-level keyboard teleoperation."""

import json
import sys
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros import keyboard_teleop


class KeyboardTeleopNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.output_topic = rospy.get_param(
            "~confirmed_label_topic", "/colmag/teleop/confirmed_label"
        )
        self.require_subscriber = bool(
            rospy.get_param("~require_subscriber", True)
        )
        self.publisher = rospy.Publisher(
            self.output_topic, String, queue_size=10, latch=False
        )
        self.sequence_id = 0

    def publish_command(self, command):
        task = keyboard_teleop.task_for_command(command)
        if task is None:
            self.rospy.logwarn("Unknown teleop command %r; enter help for controls", command)
            return False
        if self.require_subscriber and self.publisher.get_num_connections() < 1:
            self.rospy.logwarn(
                "No dispatcher subscriber on %s; command not published",
                self.output_topic,
            )
            return False

        self.sequence_id += 1
        payload = keyboard_teleop.build_confirmed_label_payload(
            command,
            sequence_id=self.sequence_id,
            timestamp=self.rospy.Time.now().to_sec(),
        )
        message = self.String(data=json.dumps(payload, sort_keys=True))
        self.publisher.publish(message)
        self.rospy.loginfo(
            "keyboard teleop command=%s label=%s task=%s",
            command,
            payload["label"],
            task,
        )
        return True

    def run(self):
        for line in keyboard_teleop.help_lines():
            print(line)

        while not self.rospy.is_shutdown():
            try:
                command = input("teleop> ").strip()
            except EOFError:
                break
            if not command:
                continue
            if command.lower() in keyboard_teleop.QUIT_COMMANDS:
                break
            if command.lower() in keyboard_teleop.HELP_COMMANDS:
                for line in keyboard_teleop.help_lines():
                    print(line)
                continue
            self.publish_command(command)


def main():
    import rospy

    rospy.init_node("keyboard_teleop_node")
    KeyboardTeleopNode().run()


if __name__ == "__main__":
    main()
