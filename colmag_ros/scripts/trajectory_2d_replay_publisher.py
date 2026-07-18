#!/usr/bin/env python3
import json
import math
import time


def _bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _digit_two_points():
    points = [
        (-0.024, 0.026),
        (-0.012, 0.034),
        (0.004, 0.031),
        (0.014, 0.020),
        (0.008, 0.008),
        (-0.006, -0.006),
        (-0.020, -0.020),
        (0.018, -0.022),
    ]
    out = []
    for a, b in zip(points, points[1:]):
        for i in range(4):
            alpha = i / 4.0
            out.append((a[0] + alpha * (b[0] - a[0]), a[1] + alpha * (b[1] - a[1])))
    out.append(points[-1])
    return out


def build_default_replay_samples(
    start_time=None,
    sequence_id=1,
    hover_sec=2.4,
    capture_sec=2.2,
    rate_hz=20.0,
):
    start_time = time.time() if start_time is None else float(start_time)
    samples = []
    dt = 1.0 / float(rate_hz)
    timestamp = start_time
    index = 0

    hover_count = int(math.ceil(hover_sec * rate_hz))
    for _ in range(hover_count):
        samples.append(
            {
                "timestamp": timestamp,
                "sequence_id": sequence_id,
                "sample_index": index,
                "x": 0.048,
                "y": 0.030,
                "valid": True,
                "source": "offline_replay",
                "phase": "menu_hover",
            }
        )
        timestamp += dt
        index += 1

    draw_points = _digit_two_points()
    capture_count = int(math.ceil(capture_sec * rate_hz))
    for i in range(capture_count):
        point = draw_points[min(len(draw_points) - 1, int(i * len(draw_points) / capture_count))]
        samples.append(
            {
                "timestamp": timestamp,
                "sequence_id": sequence_id,
                "sample_index": index,
                "x": point[0],
                "y": point[1],
                "valid": True,
                "source": "offline_replay",
                "phase": "symbol_capture",
            }
        )
        timestamp += dt
        index += 1
    return samples


class Trajectory2DReplayPublisher:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.topic = rospy.get_param("~trajectory_topic", "/colmag/trajectory_2d")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.loop = _bool_param(rospy.get_param("~loop", False))
        self.sequence_id = int(rospy.get_param("~sequence_id", 1))
        self.samples = build_default_replay_samples(
            start_time=time.time(),
            sequence_id=self.sequence_id,
            hover_sec=float(rospy.get_param("~hover_sec", 2.4)),
            capture_sec=float(rospy.get_param("~capture_sec", 2.2)),
            rate_hz=self.rate_hz,
        )
        self.publisher = rospy.Publisher(self.topic, String, queue_size=20)
        rospy.loginfo("trajectory_2d_replay_publisher started")
        rospy.loginfo("trajectory_topic=%s samples=%d loop=%s", self.topic, len(self.samples), self.loop)

    def run(self):
        rate = self.rospy.Rate(self.rate_hz)
        while not self.rospy.is_shutdown():
            for sample in self.samples:
                if self.rospy.is_shutdown():
                    break
                self.publisher.publish(self.String(data=json.dumps(sample, sort_keys=True)))
                rate.sleep()
            if not self.loop:
                break


def main():
    import rospy

    rospy.init_node("trajectory_2d_replay_publisher")
    Trajectory2DReplayPublisher().run()
    rospy.spin()


if __name__ == "__main__":
    main()
