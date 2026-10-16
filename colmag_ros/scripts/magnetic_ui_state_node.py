#!/usr/bin/env python3
import json
import math
import sys
from pathlib import Path
import time


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros.controlled_capture_dataset import (
    DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE,
    normalize_capture_points,
    write_controlled_capture_dataset,
)
from colmag_ros.param_utils import bool_param, finite_float


CONTROL_ZONES = {
    "A": (0.024, 0.030),
    "B": (0.048, 0.030),
    "X": (0.024, 0.000),
}

DEFAULT_CONTROLLED_CAPTURE_DATASET_DIR = "outputs/m33e_controlled_capture_dataset"


def compute_point_velocity(prev_point, current_point, dt):
    dt = finite_float(dt, 0.0)
    if dt <= 0.0:
        return 0.0
    return math.hypot(
        finite_float(current_point[0]) - finite_float(prev_point[0]),
        finite_float(current_point[1]) - finite_float(prev_point[1]),
    ) / dt


def should_finish_controlled_capture(
    elapsed_sec,
    ink_sample_count,
    idle_elapsed_sec,
    *,
    min_duration_sec=0.6,
    min_ink_samples=8,
    idle_sec=0.5,
    timeout_sec=4.0,
):
    elapsed_sec = max(0.0, finite_float(elapsed_sec))
    idle_elapsed_sec = max(0.0, finite_float(idle_elapsed_sec))
    min_duration_sec = max(0.0, finite_float(min_duration_sec))
    idle_sec = max(0.0, finite_float(idle_sec))
    timeout_sec = max(0.0, finite_float(timeout_sec))
    ink_sample_count = int(ink_sample_count)
    min_ink_samples = int(min_ink_samples)

    timeout_end = elapsed_sec >= timeout_sec
    min_duration_passed = elapsed_sec >= min_duration_sec
    min_ink_samples_passed = ink_sample_count >= min_ink_samples
    idle_end = min_duration_passed and min_ink_samples_passed and idle_elapsed_sec >= idle_sec
    return {
        "finish": bool(timeout_end or idle_end),
        "idle_end": bool(idle_end),
        "timeout_end": bool(timeout_end),
        "min_duration_passed": bool(min_duration_passed),
        "min_ink_samples_passed": bool(min_ink_samples_passed),
    }


def build_controlled_capture_payload(
    *,
    timestamp,
    sequence_id,
    sample_id,
    points,
    started_at,
    finish_decision,
    dataset_label="",
    test_only=False,
    image_size=DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE,
):
    duration = max(0.0, finite_float(timestamp) - finite_float(started_at))
    normalized_points = normalize_capture_points(points, image_size=image_size)
    payload = {
        "timestamp": finite_float(timestamp),
        "sequence_id": int(sequence_id),
        "sample_id": str(sample_id),
        "points": [[float(x), float(y)] for x, y in points],
        "normalized_points": normalized_points,
        "duration_sec": duration,
        "capture_mode": "controlled_capture",
        "idle_end": bool(finish_decision.get("idle_end", False)),
        "timeout_end": bool(finish_decision.get("timeout_end", False)),
        "min_ink_samples_passed": bool(finish_decision.get("min_ink_samples_passed", False)),
        "source": "magnetic_ui_state_node",
        "source_topic": "/colmag/trajectory_2d",
    }
    if dataset_label:
        payload["dataset_label"] = str(dataset_label)
    if test_only:
        payload["test_only"] = True
    return payload


class MagneticUIStateMachine:
    def __init__(
        self,
        dwell_sec=2.0,
        capture_window_sec=2.0,
        min_capture_points=8,
        zone_radius=0.012,
        controlled_capture_enabled=False,
        controlled_capture_min_duration_sec=0.6,
        controlled_capture_idle_velocity_threshold=0.04,
        controlled_capture_idle_sec=0.5,
        controlled_capture_timeout_sec=4.0,
        controlled_capture_target_window_sec=3.0,
        controlled_capture_min_ink_samples=8,
        controlled_capture_save_dataset=False,
        controlled_capture_dataset_dir=DEFAULT_CONTROLLED_CAPTURE_DATASET_DIR,
        controlled_capture_dataset_label="",
        controlled_capture_render_png=True,
        controlled_capture_image_size=DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE,
        enable_legacy_control_zones=True,
    ):
        self.dwell_sec = float(dwell_sec)
        self.capture_window_sec = float(capture_window_sec)
        self.min_capture_points = int(min_capture_points)
        self.zone_radius = float(zone_radius)
        self.controlled_capture_enabled = bool(controlled_capture_enabled)
        self.controlled_capture_min_duration_sec = float(controlled_capture_min_duration_sec)
        self.controlled_capture_idle_velocity_threshold = float(controlled_capture_idle_velocity_threshold)
        self.controlled_capture_idle_sec = float(controlled_capture_idle_sec)
        self.controlled_capture_timeout_sec = float(controlled_capture_timeout_sec)
        self.controlled_capture_target_window_sec = float(controlled_capture_target_window_sec)
        self.controlled_capture_min_ink_samples = int(controlled_capture_min_ink_samples)
        self.controlled_capture_save_dataset = bool(controlled_capture_save_dataset)
        self.controlled_capture_dataset_dir = str(controlled_capture_dataset_dir)
        self.controlled_capture_dataset_label = str(controlled_capture_dataset_label or "")
        self.controlled_capture_render_png = bool(controlled_capture_render_png)
        self.controlled_capture_image_size = int(controlled_capture_image_size)
        self.enable_legacy_control_zones = bool(enable_legacy_control_zones)
        self.sequence_id = 0
        self.sample_id_counter = 0
        self.reset()

    def reset(self):
        self.state = "IDLE"
        self.hover_started_at = None
        self.capture_started_at = None
        self.capture_points = []
        self.capture_sequence_id = None
        self.capture_test_only = False
        self.controlled_last_point = None
        self.controlled_last_timestamp = None
        self.controlled_idle_started_at = None
        self.last_active_zone = None

    def active_zone(self, x, y):
        for label, (cx, cy) in CONTROL_ZONES.items():
            if math.hypot(x - cx, y - cy) <= self.zone_radius:
                return label
        return ""

    def process_sample(self, sample):
        context = self._sample_context(sample)
        for handler in (
            self._handle_priority_sample,
            self._handle_idle_or_hover_sample,
            self._handle_capture_start_sample,
            self._handle_fixed_capture_sample,
            self._handle_controlled_capture_sample,
        ):
            events = handler(context)
            if events is not None:
                return events
        return [
            self._ui(
                context["timestamp"],
                context["zone"],
                0.0,
                self.state.lower(),
            )
        ]

    def _sample_context(self, sample):
        timestamp = finite_float(sample.get("timestamp"), time.time())
        valid = bool(sample.get("valid", True))
        sequence_id = int(finite_float(sample.get("sequence_id"), self.sequence_id or 1))
        x = finite_float(sample.get("x"), 0.0)
        y = finite_float(sample.get("y"), 0.0)
        zone = (
            self.active_zone(x, y)
            if valid and self.enable_legacy_control_zones
            else ""
        )
        self.sequence_id = sequence_id
        self.last_active_zone = zone
        return {
            "timestamp": timestamp,
            "valid": valid,
            "sequence_id": sequence_id,
            "x": x,
            "y": y,
            "zone": zone,
            "test_only": bool(sample.get("test_only", False)),
        }

    def _handle_priority_sample(self, context):
        timestamp = context["timestamp"]
        valid = context["valid"]
        zone = context["zone"]
        if zone == "A":
            self.state = "MOTION_LOCKED"
            self.hover_started_at = None
            self._clear_capture()
            return [self._ui(timestamp, zone, 0.0, "motion locked by A/Stop zone")]

        if zone == "X":
            self.state = "CANCELLED"
            self.hover_started_at = None
            self._clear_capture()
            return [self._ui(timestamp, zone, 0.0, "capture cancelled by X zone")]

        if self.state == "MOTION_LOCKED":
            return [self._ui(timestamp, zone, 0.0, "motion locked")]

        if not valid:
            return [self._ui(timestamp, zone, 0.0, "invalid sample ignored")]
        return None

    def _handle_idle_or_hover_sample(self, context):
        timestamp = context["timestamp"]
        zone = context["zone"]
        if self.state in ("IDLE", "CANCELLED", "WAIT_CONFIRM", "CONFIRM_PENDING"):
            if zone == "B":
                self.state = "HOVER_MENU"
                self.hover_started_at = timestamp
                return [self._ui(timestamp, zone, 0.0, "hovering B / Digit Mode")]
            return [self._ui(timestamp, zone, 0.0, "idle")]

        if self.state == "HOVER_MENU":
            if zone != "B":
                self.state = "IDLE"
                self.hover_started_at = None
                return [self._ui(timestamp, zone, 0.0, "B dwell cancelled")]

            progress = self._dwell_progress(timestamp)
            if progress >= 1.0:
                if self.controlled_capture_enabled:
                    self.state = "CONTROLLED_CAPTURE_READY"
                    message = "controlled capture ready"
                else:
                    self.state = "DIGIT_MODE_ARMED"
                    message = "digit mode armed"
                return [self._ui(timestamp, zone, 1.0, message)]
            return [self._ui(timestamp, zone, progress, "hovering B / Digit Mode")]
        return None

    def _handle_capture_start_sample(self, context):
        timestamp = context["timestamp"]
        sequence_id = context["sequence_id"]
        x = context["x"]
        y = context["y"]
        zone = context["zone"]
        test_only = context["test_only"]
        if self.state == "CONTROLLED_CAPTURE_READY":
            if zone == "B":
                return [self._ui(timestamp, zone, 1.0, "controlled capture ready")]

            self._start_controlled_capture(timestamp, sequence_id, x, y, test_only)
            return [self._ui(timestamp, zone, 0.0, "controlled capturing")]

        if self.state == "DIGIT_MODE_ARMED":
            if zone == "B":
                return [self._ui(timestamp, zone, 1.0, "digit mode armed")]

            self.state = "CAPTURING_SYMBOL"
            self.capture_started_at = timestamp
            self.capture_sequence_id = sequence_id
            self.capture_points = [(x, y)]
            self.capture_test_only = test_only
            return [self._ui(timestamp, zone, 1.0, "capturing symbol")]
        return None

    def _handle_fixed_capture_sample(self, context):
        if self.state != "CAPTURING_SYMBOL":
            return None
        timestamp = context["timestamp"]
        x = context["x"]
        y = context["y"]
        zone = context["zone"]
        self.capture_points.append((x, y))
        self.capture_test_only = self.capture_test_only or context["test_only"]
        elapsed = max(0.0, timestamp - self.capture_started_at)
        progress = min(1.0, elapsed / max(self.capture_window_sec, 1e-9))

        if elapsed < self.capture_window_sec:
            return [self._ui(timestamp, zone, progress, "capturing symbol")]

        events = []
        if len(self.capture_points) >= self.min_capture_points:
            capture = self._capture_payload(timestamp)
            events.append(("capture", capture))
            self.state = "WAIT_CONFIRM"
            events.append(self._ui(timestamp, zone, 1.0, "symbol capture published"))
        else:
            self.state = "CANCELLED"
            events.append(self._ui(timestamp, zone, 1.0, "capture rejected: too few points"))

        self.capture_started_at = None
        self.capture_points = []
        return events

    def _handle_controlled_capture_sample(self, context):
        if self.state != "CONTROLLED_CAPTURING":
            return None
        timestamp = context["timestamp"]
        zone = context["zone"]
        self._append_controlled_point(
            timestamp,
            context["x"],
            context["y"],
            context["test_only"],
        )
        elapsed = max(0.0, timestamp - self.capture_started_at)
        idle_elapsed = 0.0
        if self.controlled_idle_started_at is not None:
            idle_elapsed = max(0.0, timestamp - self.controlled_idle_started_at)
        decision = should_finish_controlled_capture(
            elapsed,
            len(self.capture_points),
            idle_elapsed,
            min_duration_sec=self.controlled_capture_min_duration_sec,
            min_ink_samples=self.controlled_capture_min_ink_samples,
            idle_sec=self.controlled_capture_idle_sec,
            timeout_sec=self.controlled_capture_timeout_sec,
        )
        progress = min(
            1.0,
            elapsed / max(self.controlled_capture_target_window_sec, 1e-9),
        )

        if not decision["finish"]:
            return [self._ui(timestamp, zone, progress, "controlled capturing")]

        if not decision["min_ink_samples_passed"]:
            self.state = "CANCELLED"
            events = [
                self._ui(
                    timestamp,
                    zone,
                    progress,
                    "controlled capture rejected: too few ink samples",
                )
            ]
            self._clear_capture()
            return events

        capture = self._controlled_capture_payload(timestamp, decision)
        events = [("capture", capture)]
        self.state = "CONTROLLED_CAPTURE_DONE"
        events.append(self._ui(timestamp, zone, 1.0, "controlled capture done"))
        self.state = "CONFIRM_PENDING"
        events.append(self._ui(timestamp, zone, 1.0, "controlled capture published"))
        self._clear_capture()
        return events

    def _dwell_progress(self, timestamp):
        if self.hover_started_at is None:
            return 0.0
        return min(1.0, max(0.0, timestamp - self.hover_started_at) / max(self.dwell_sec, 1e-9))

    def _capture_payload(self, timestamp):
        self.sample_id_counter += 1
        duration = 0.0
        if self.capture_started_at is not None:
            duration = max(0.0, timestamp - self.capture_started_at)
        return {
            "timestamp": timestamp,
            "sequence_id": self.capture_sequence_id or self.sequence_id,
            "sample_id": "m28_symbol_%06d" % self.sample_id_counter,
            "points": [[float(x), float(y)] for x, y in self.capture_points],
            "duration_sec": duration,
            "source_topic": "/colmag/trajectory_2d",
        }

    def _clear_capture(self):
        self.capture_started_at = None
        self.capture_points = []
        self.capture_sequence_id = None
        self.capture_test_only = False
        self.controlled_last_point = None
        self.controlled_last_timestamp = None
        self.controlled_idle_started_at = None

    def _start_controlled_capture(self, timestamp, sequence_id, x, y, test_only=False):
        self.state = "CONTROLLED_CAPTURING"
        self.capture_started_at = timestamp
        self.capture_sequence_id = sequence_id
        self.capture_points = [(x, y)]
        self.capture_test_only = bool(test_only)
        self.controlled_last_point = (x, y)
        self.controlled_last_timestamp = timestamp
        self.controlled_idle_started_at = None

    def _append_controlled_point(self, timestamp, x, y, test_only=False):
        point = (x, y)
        velocity = 0.0
        if self.controlled_last_point is not None and self.controlled_last_timestamp is not None:
            velocity = compute_point_velocity(
                self.controlled_last_point,
                point,
                timestamp - self.controlled_last_timestamp,
            )
        if velocity <= self.controlled_capture_idle_velocity_threshold:
            if self.controlled_idle_started_at is None:
                self.controlled_idle_started_at = timestamp
        else:
            self.controlled_idle_started_at = None

        self.capture_points.append(point)
        self.capture_test_only = self.capture_test_only or bool(test_only)
        self.controlled_last_point = point
        self.controlled_last_timestamp = timestamp

    def _controlled_capture_payload(self, timestamp, decision):
        self.sample_id_counter += 1
        payload = build_controlled_capture_payload(
            timestamp=timestamp,
            sequence_id=self.capture_sequence_id or self.sequence_id,
            sample_id="m33e_controlled_%06d" % self.sample_id_counter,
            points=self.capture_points,
            started_at=self.capture_started_at,
            finish_decision=decision,
            dataset_label=self.controlled_capture_dataset_label,
            test_only=self.capture_test_only,
            image_size=self.controlled_capture_image_size,
        )
        if self.controlled_capture_save_dataset:
            payload = self._write_controlled_capture_dataset(payload)
        return payload

    def _write_controlled_capture_dataset(self, payload):
        return write_controlled_capture_dataset(
            payload,
            self.controlled_capture_dataset_dir,
            render_png=self.controlled_capture_render_png,
            image_size=self.controlled_capture_image_size,
        )

    def _ui(self, timestamp, active_zone, dwell_progress, message):
        return (
            "ui",
            {
                "timestamp": timestamp,
                "state": self.state,
                "active_zone": active_zone,
                "dwell_progress": float(dwell_progress),
                "message": message,
            },
        )


class MagneticUIStateNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.input_topic = rospy.get_param("~trajectory_topic", "/colmag/trajectory_2d")
        self.ui_state_topic = rospy.get_param("~ui_state_topic", "/colmag/ui_state")
        self.capture_topic = rospy.get_param("~symbol_capture_topic", "/colmag/symbol_capture")
        self.machine = MagneticUIStateMachine(
            dwell_sec=float(rospy.get_param("~dwell_sec", 2.0)),
            capture_window_sec=float(rospy.get_param("~capture_window_sec", 2.0)),
            min_capture_points=int(rospy.get_param("~min_capture_points", 8)),
            zone_radius=float(rospy.get_param("~zone_radius", 0.012)),
            controlled_capture_enabled=bool_param(rospy.get_param("~controlled_capture_enabled", False)),
            controlled_capture_min_duration_sec=float(rospy.get_param("~controlled_capture_min_duration_sec", 0.6)),
            controlled_capture_idle_velocity_threshold=float(
                rospy.get_param("~controlled_capture_idle_velocity_threshold", 0.04)
            ),
            controlled_capture_idle_sec=float(rospy.get_param("~controlled_capture_idle_sec", 0.5)),
            controlled_capture_timeout_sec=float(rospy.get_param("~controlled_capture_timeout_sec", 4.0)),
            controlled_capture_target_window_sec=float(rospy.get_param("~controlled_capture_target_window_sec", 3.0)),
            controlled_capture_min_ink_samples=int(rospy.get_param("~controlled_capture_min_ink_samples", 8)),
            controlled_capture_save_dataset=bool_param(rospy.get_param("~controlled_capture_save_dataset", False)),
            controlled_capture_dataset_dir=rospy.get_param(
                "~controlled_capture_dataset_dir",
                DEFAULT_CONTROLLED_CAPTURE_DATASET_DIR,
            ),
            controlled_capture_dataset_label=rospy.get_param("~controlled_capture_dataset_label", ""),
            controlled_capture_render_png=bool_param(rospy.get_param("~controlled_capture_render_png", True)),
            controlled_capture_image_size=int(
                rospy.get_param("~controlled_capture_image_size", DEFAULT_CONTROLLED_CAPTURE_IMAGE_SIZE)
            ),
            enable_legacy_control_zones=bool_param(
                rospy.get_param("~enable_legacy_control_zones", True)
            ),
        )
        self.ui_pub = rospy.Publisher(self.ui_state_topic, String, queue_size=20)
        self.capture_pub = rospy.Publisher(self.capture_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, String, self._handle_sample, queue_size=50)
        rospy.loginfo("magnetic_ui_state_node started")
        rospy.loginfo("trajectory_topic=%s", self.input_topic)
        rospy.loginfo("ui_state_topic=%s", self.ui_state_topic)
        rospy.loginfo("symbol_capture_topic=%s", self.capture_topic)
        rospy.loginfo("controlled_capture_enabled=%s", self.machine.controlled_capture_enabled)
        rospy.loginfo("controlled_capture_save_dataset=%s", self.machine.controlled_capture_save_dataset)
        rospy.loginfo("enable_legacy_control_zones=%s", self.machine.enable_legacy_control_zones)

    def _handle_sample(self, message):
        try:
            sample = json.loads(message.data)
        except ValueError as exc:
            self.rospy.logwarn("failed to parse trajectory_2d JSON: %r", exc)
            return
        if not isinstance(sample, dict):
            self.rospy.logwarn("trajectory_2d payload is not a JSON object")
            return

        for kind, payload in self.machine.process_sample(sample):
            msg = self.String(data=json.dumps(payload, sort_keys=True))
            if kind == "ui":
                self.ui_pub.publish(msg)
            elif kind == "capture":
                self.capture_pub.publish(msg)


def main():
    import rospy

    rospy.init_node("magnetic_ui_state_node")
    MagneticUIStateNode()
    rospy.spin()


if __name__ == "__main__":
    main()
