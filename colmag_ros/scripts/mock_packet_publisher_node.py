#!/usr/bin/env python3

import time

from colmag_ros.adapter import make_mock_raw_packet_json


MOCK_RIGHT_GESTURE = (
    ("active_start", 0.000, 0.000),
    ("active_move", 0.006, 0.000),
    ("active_move", 0.012, 0.000),
    ("active_move", 0.020, 0.000),
    ("active_move", 0.030, 0.000),
    ("active_move", 0.042, 0.000),
    ("active_move", 0.055, 0.000),
    ("active_move", 0.070, 0.000),
    ("hold_end", 0.070, 0.000),
    ("hold_end", 0.070, 0.000),
    ("hold_end", 0.070, 0.000),
    ("hold_end", 0.070, 0.000),
)


def _bool_param(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def wrap_tracking_x(x, x_min, x_max):
    span = x_max - x_min
    if span <= 0.0:
        return x
    return x_min + ((x - x_min) % span)


class MockPacketPublisherNode:
    """Publish mock COLMAG packet JSON for offline ROS pipeline testing."""

    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.message_type = String
        self.publish_topic = rospy.get_param("~publish_topic", "/colmag/raw_packet")
        self.publisher = rospy.Publisher(self.publish_topic, String, queue_size=10)
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.wrap_tracking_position = _bool_param(
            rospy.get_param("~wrap_tracking_position", False)
        )
        self.wrap_x_min = float(rospy.get_param("~wrap_x_min", 0.0))
        self.wrap_x_max = float(rospy.get_param("~wrap_x_max", 0.9))
        self.sequence_index = 0
        self.base_x = 0.0
        self.last_x = 0.0
        self.last_unbounded_x = 0.0
        self.last_phase = ""

    def _next_sample(self):
        phase, relative_x, y = MOCK_RIGHT_GESTURE[self.sequence_index]
        if self.sequence_index == 0:
            self.base_x = self.last_unbounded_x

        unbounded_x = self.base_x + relative_x
        if self.wrap_tracking_position:
            x = wrap_tracking_x(unbounded_x, self.wrap_x_min, self.wrap_x_max)
        else:
            x = unbounded_x
        self.last_x = x
        self.last_unbounded_x = unbounded_x
        self.sequence_index = (self.sequence_index + 1) % len(MOCK_RIGHT_GESTURE)
        return phase, x, y

    def run(self):
        rate = self.rospy.Rate(self.publish_rate)
        self.rospy.loginfo("Publishing mock COLMAG packets on %s", self.publish_topic)
        while not self.rospy.is_shutdown():
            phase, x, y = self._next_sample()
            if phase != self.last_phase:
                self.rospy.loginfo(
                    "Mock packet phase=%s tracking_position=(%.3f, %.3f)",
                    phase,
                    x,
                    y,
                )
                self.last_phase = phase
            timestamp = time.time()
            self.publisher.publish(
                self.message_type(data=make_mock_raw_packet_json(timestamp, x, y))
            )
            rate.sleep()


if __name__ == "__main__":
    import rospy

    rospy.init_node("mock_packet_publisher_node")
    try:
        MockPacketPublisherNode().run()
    except rospy.ROSInterruptException:
        pass
