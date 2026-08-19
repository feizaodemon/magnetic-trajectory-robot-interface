#!/usr/bin/env python3
"""Latched ROS source of truth for the TASK/TELEOP operator mode."""

import sys
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros import control_mode


class ControlModeNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.state_topic = rospy.get_param("~state_topic", "/colmag/control_mode")
        self.request_topic = rospy.get_param(
            "~request_topic", "/colmag/control_mode/request"
        )
        requested_initial = rospy.get_param("~initial_mode", control_mode.TASK)
        self.mode = control_mode.startup_mode(requested_initial)
        if control_mode.normalize_mode(requested_initial) is None:
            rospy.logwarn(
                "Invalid initial control mode %r; failing closed to TASK",
                requested_initial,
            )

        self.publisher = rospy.Publisher(
            self.state_topic, String, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.request_topic, String, self._handle_request, queue_size=10
        )
        self._publish()
        rospy.loginfo(
            "control_mode_node started mode=%s state_topic=%s request_topic=%s",
            self.mode,
            self.state_topic,
            self.request_topic,
        )

    def _publish(self):
        self.publisher.publish(self.String(data=self.mode))

    def _handle_request(self, message):
        requested = control_mode.normalize_mode(message.data)
        if requested is None:
            self.rospy.logwarn("Rejected invalid control mode request %r", message.data)
            self._publish()
            return
        if requested == self.mode:
            self._publish()
            return
        previous = self.mode
        self.mode = requested
        self._publish()
        self.rospy.loginfo("control mode changed %s -> %s", previous, self.mode)


def main():
    import rospy

    rospy.init_node("control_mode_node")
    ControlModeNode()
    rospy.spin()


if __name__ == "__main__":
    main()
