#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from colmag_ros.adapter import (
    ensure_project_src_path,
    packet_to_raw_json,
)


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 921600
DEFAULT_READ_TIMEOUT = 0.01


class SerialPacketPublisherNode:
    """Read real COLMAG serial packets and publish raw packet JSON."""

    def __init__(self):
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baudrate = int(rospy.get_param("~baudrate", DEFAULT_BAUDRATE))
        self.project_src_path = rospy.get_param("~project_src_path", None)
        self.publish_topic = rospy.get_param("~publish_topic", "/colmag/raw_packet")
        self.read_timeout = float(rospy.get_param("~read_timeout", DEFAULT_READ_TIMEOUT))
        self.debug = bool(rospy.get_param("~debug", False))

        ensure_project_src_path(self.project_src_path)
        from serial_mvp.reader import SerialPacketReader  # pylint: disable=import-outside-toplevel
        from serial_mvp.sources import SerialPortSource  # pylint: disable=import-outside-toplevel

        self.reader_type = SerialPacketReader
        self.source = SerialPortSource(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.read_timeout,
        )
        self.publisher = rospy.Publisher(self.publish_topic, String, queue_size=20)
        self.published_packets = 0

    def _publish_packet(self, packet_index, packet_data):
        del packet_index
        timestamp = rospy.Time.now().to_sec()
        self.publisher.publish(String(data=packet_to_raw_json(timestamp, packet_data)))
        self.published_packets += 1
        if self.debug and self.published_packets % 100 == 0:
            rospy.loginfo("Published %d serial packets", self.published_packets)

    def run(self):
        rospy.loginfo(
            "Opening COLMAG serial port %s at %d baud; publishing %s",
            self.port,
            self.baudrate,
            self.publish_topic,
        )
        connection = None
        try:
            connection = self.source.open()
            reader = self.reader_type(
                connection,
                packet_handler=self._publish_packet,
                verbose_packets=self.debug,
            )
            while not rospy.is_shutdown():
                try:
                    reader.read_once()
                except Exception as exc:  # pylint: disable=broad-except
                    rospy.logerr("Serial read failed: %s", exc)
                    break
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logerr("Failed to open serial port %s: %s", self.port, exc)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:  # pylint: disable=broad-except
                    rospy.logwarn("Failed to close serial port cleanly: %s", exc)
            rospy.loginfo(
                "Serial packet publisher stopped after %d packets",
                self.published_packets,
            )


if __name__ == "__main__":
    rospy.init_node("serial_packet_publisher_node")
    try:
        SerialPacketPublisherNode().run()
    except ValueError as exc:
        rospy.logfatal("Serial packet publisher configuration error: %s", exc)
        raise SystemExit(2) from exc
    except rospy.ROSInterruptException:
        pass
