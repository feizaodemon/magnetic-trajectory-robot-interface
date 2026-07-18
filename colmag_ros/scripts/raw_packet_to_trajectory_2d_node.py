#!/usr/bin/env python3
import json

import rospy
from std_msgs.msg import String


class RawPacketToTrajectory2DNode:
    """Bridge offline /colmag/raw_packet JSON payloads to /colmag/trajectory_2d."""

    def __init__(self):
        self.input_topic = rospy.get_param("~raw_packet_topic", "/colmag/raw_packet")
        self.output_topic = rospy.get_param("~trajectory_topic", "/colmag/trajectory_2d")

        self.sequence_id = 1
        self.sample_index = 0

        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=50)
        self.subscriber = rospy.Subscriber(self.input_topic, String, self._handle_packet, queue_size=50)

        rospy.loginfo("raw_packet_to_trajectory_2d_node started (offline adapter only)")
        rospy.loginfo("Listening on %s, publishing to %s", self.input_topic, self.output_topic)

    def _handle_packet(self, message):
        try:
            payload = json.loads(message.data)
        except ValueError as exc:
            rospy.logwarn("Failed to parse raw_packet JSON: %s", exc)
            return

        if not isinstance(payload, dict):
            rospy.logwarn("raw_packet payload is not a JSON object")
            return

        # Extract timestamp
        timestamp = payload.get("timestamp", rospy.get_time())

        # Extract x/y plus the hardware-qualified orientation interaction clutch.
        # The serialized field remains ``valid`` for route compatibility.
        tracking = payload.get("tracking_outputs")
        if not isinstance(tracking, (list, tuple)) or len(tracking) < 2:
            rospy.logwarn_throttle(1.0, "raw_packet missing valid tracking_outputs (length < 2)")
            return

        try:
            x = float(tracking[0])
            y = float(tracking[1])
            z = float(tracking[2]) if len(tracking) > 2 else 0.0
            interaction_engaged = True
            if len(tracking) >= 6:
                interaction_engaged = float(tracking[5]) > 0.0
        except (ValueError, TypeError):
            rospy.logwarn_throttle(1.0, "raw_packet tracking_outputs contain invalid types")
            return

        out_msg = {
            "timestamp": timestamp,
            "sequence_id": self.sequence_id,
            "sample_index": self.sample_index,
            "x": x,
            "y": y,
            "z": z,
            "valid": interaction_engaged,
            "source": "raw_packet_bridge_offline",
            "source_topic": self.input_topic,
            "phase": "raw_packet_bridge"
        }

        self.sample_index += 1
        self.publisher.publish(String(data=json.dumps(out_msg, sort_keys=True)))

    def run(self):
        rospy.spin()


def main():
    rospy.init_node("raw_packet_to_trajectory_2d_node")
    try:
        RawPacketToTrajectory2DNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
