#!/usr/bin/env python3
import json
import math
import time

try:
    from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
        build_gamepad_button_rects,
        build_virtual_button_rects,
        map_canvas_to_world,
    )
except ImportError:
    from magnetic_trajectory_dashboard_node import (
        build_gamepad_button_rects,
        build_virtual_button_rects,
        map_canvas_to_world,
    )


DEFAULT_BOUNDS = (-0.18, 0.18, -0.18, 0.18)
LAYOUT_RANK_CONFIRM = "rank_confirm"
LAYOUT_GAMEPAD = "gamepad"

TEST_SEQUENCE_RANK_1 = "rank_1"
TEST_SEQUENCE_GAMEPAD_B_THEN_C = "gamepad_b_then_c"
TEST_SEQUENCE_GAMEPAD_A_BLOCK_THEN_X = "gamepad_a_block_then_x"
SUPPORTED_TEST_SEQUENCES = (
    TEST_SEQUENCE_RANK_1,
    TEST_SEQUENCE_GAMEPAD_B_THEN_C,
    TEST_SEQUENCE_GAMEPAD_A_BLOCK_THEN_X,
)


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def finite_float(value, default=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def find_virtual_button_rect(button_rects, target_button_id):
    for rect in button_rects:
        if rect.get("id") == target_button_id:
            return rect
    return None


def rect_center(rect):
    return (rect["x1"] + rect["x2"]) / 2.0, (rect["y1"] + rect["y2"]) / 2.0


def normalize_test_sequence(test_sequence):
    normalized = str(test_sequence or TEST_SEQUENCE_RANK_1).strip().lower()
    if normalized in SUPPORTED_TEST_SEQUENCES:
        return normalized
    return TEST_SEQUENCE_RANK_1


def build_button_rects_for_layout(layout, width, height, padding):
    if str(layout) == LAYOUT_GAMEPAD:
        return build_gamepad_button_rects(width, height, padding)
    return build_virtual_button_rects(width, height, padding)


def target_world_point_for_button(target_button_id, width, height, padding, bounds, layout=LAYOUT_RANK_CONFIRM):
    rects = build_button_rects_for_layout(layout, width, height, padding)
    rect = find_virtual_button_rect(rects, target_button_id)
    if rect is None:
        return None, None, "target_button_not_found"

    canvas_x, canvas_y = rect_center(rect)
    world_x, world_y = map_canvas_to_world(canvas_x, canvas_y, bounds, width, height, padding)
    return (world_x, world_y), (canvas_x, canvas_y), ""


def build_test_sequence_actions(
    test_sequence,
    target_button_id="rank_1",
    hold_sec=2.8,
    between_action_delay_sec=1.0,
    digit_mode_sec=3.0,
):
    test_sequence = normalize_test_sequence(test_sequence)
    hold_sec = max(0.0, float(hold_sec))
    between_action_delay_sec = max(0.0, float(between_action_delay_sec))
    digit_mode_sec = max(0.0, float(digit_mode_sec))

    if test_sequence == TEST_SEQUENCE_GAMEPAD_B_THEN_C:
        return [
            {
                "button_id": "B",
                "layout": LAYOUT_GAMEPAD,
                "hold_sec": hold_sec,
                "delay_after_sec": digit_mode_sec + between_action_delay_sec,
            },
            {
                "button_id": "C",
                "layout": LAYOUT_GAMEPAD,
                "hold_sec": hold_sec,
                "delay_after_sec": 0.0,
            },
        ]

    if test_sequence == TEST_SEQUENCE_GAMEPAD_A_BLOCK_THEN_X:
        return [
            {
                "button_id": "A",
                "layout": LAYOUT_GAMEPAD,
                "hold_sec": hold_sec,
                "delay_after_sec": between_action_delay_sec,
            },
            {
                "button_id": "U",
                "layout": LAYOUT_GAMEPAD,
                "hold_sec": hold_sec,
                "delay_after_sec": between_action_delay_sec,
            },
            {
                "button_id": "X",
                "layout": LAYOUT_GAMEPAD,
                "hold_sec": hold_sec,
                "delay_after_sec": 0.0,
            },
        ]

    return [
        {
            "button_id": str(target_button_id or "rank_1"),
            "layout": LAYOUT_RANK_CONFIRM,
            "hold_sec": hold_sec,
            "delay_after_sec": 0.0,
        },
    ]


def build_test_cursor_payload(
    x,
    y,
    target_button_id,
    canvas_x,
    canvas_y,
    timestamp,
    test_sequence=TEST_SEQUENCE_RANK_1,
    sequence_step=0,
    test_only=True,
):
    return {
        "timestamp": float(timestamp),
        "x": float(x),
        "y": float(y),
        "valid": True,
        "source": "dashboard_dwell_test_cursor",
        "test_only": bool(test_only),
        "test_sequence": str(test_sequence),
        "sequence_step": int(sequence_step),
        "target_button_id": str(target_button_id),
        "canvas_target_x": float(canvas_x),
        "canvas_target_y": float(canvas_y),
    }


def jittered_point(x, y, jitter_radius, step_index):
    jitter_radius = max(0.0, float(jitter_radius))
    if jitter_radius <= 0.0:
        return x, y
    angle = step_index * 2.399963229728653
    radius = jitter_radius * 0.5
    return x + math.cos(angle) * radius, y + math.sin(angle) * radius


class DashboardDwellTestCursorNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String

        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/colmag/trajectory_2d")
        self.test_sequence = normalize_test_sequence(rospy.get_param("~test_sequence", TEST_SEQUENCE_RANK_1))
        self.target_button_id = rospy.get_param("~target_button_id", "rank_1")
        self.start_delay_sec = float(rospy.get_param("~start_delay_sec", 6.0))
        self.hold_sec = float(rospy.get_param("~hold_sec", 2.8))
        self.between_action_delay_sec = float(rospy.get_param("~between_action_delay_sec", 1.0))
        self.digit_mode_sec = float(rospy.get_param("~digit_mode_sec", 3.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.canvas_width = int(rospy.get_param("~canvas_width", 560))
        self.canvas_height = int(rospy.get_param("~canvas_height", 520))
        self.canvas_padding = int(rospy.get_param("~canvas_padding", 20))
        self.x_min = float(rospy.get_param("~x_min", DEFAULT_BOUNDS[0]))
        self.x_max = float(rospy.get_param("~x_max", DEFAULT_BOUNDS[1]))
        self.y_min = float(rospy.get_param("~y_min", DEFAULT_BOUNDS[2]))
        self.y_max = float(rospy.get_param("~y_max", DEFAULT_BOUNDS[3]))
        self.jitter_radius = float(rospy.get_param("~jitter_radius", 0.0))
        self.publish_test_only_marker = bool_param(rospy.get_param("~publish_test_only_marker", True))
        self.bounds = (self.x_min, self.x_max, self.y_min, self.y_max)

        self.actions = build_test_sequence_actions(
            self.test_sequence,
            target_button_id=self.target_button_id,
            hold_sec=self.hold_sec,
            between_action_delay_sec=self.between_action_delay_sec,
            digit_mode_sec=self.digit_mode_sec,
        )
        self.resolved_actions = []
        for index, action in enumerate(self.actions):
            target, canvas_target, reason = target_world_point_for_button(
                action["button_id"],
                self.canvas_width,
                self.canvas_height,
                self.canvas_padding,
                self.bounds,
                layout=action["layout"],
            )
            if target is None:
                raise ValueError("%s:%s" % (reason, action["button_id"]))
            resolved = dict(action)
            resolved["world_target"] = target
            resolved["canvas_target"] = canvas_target
            resolved["sequence_step"] = index
            self.resolved_actions.append(resolved)

        self.publisher = rospy.Publisher(self.trajectory_topic, String, queue_size=20)

        rospy.logwarn(
            "dashboard_dwell_test_cursor_node is TEST-ONLY; it publishes only %s",
            self.trajectory_topic,
        )
        for action in self.resolved_actions:
            target_x, target_y = action["world_target"]
            canvas_x, canvas_y = action["canvas_target"]
            rospy.loginfo(
                "test_sequence=%s step=%d target_button=%s layout=%s canvas=(%.1f, %.1f) world=(%.6f, %.6f)",
                self.test_sequence,
                action["sequence_step"],
                action["button_id"],
                action["layout"],
                canvas_x,
                canvas_y,
                target_x,
                target_y,
            )

    def run(self):
        if self.start_delay_sec > 0.0:
            self.rospy.sleep(self.start_delay_sec)

        rate = self.rospy.Rate(max(self.rate_hz, 1e-9))
        total_samples = 0
        for action in self.resolved_actions:
            if self.rospy.is_shutdown():
                break

            target_x, target_y = action["world_target"]
            canvas_x, canvas_y = action["canvas_target"]
            end_at = self.rospy.get_time() + max(0.0, action["hold_sec"])
            step_index = 0
            while not self.rospy.is_shutdown() and self.rospy.get_time() <= end_at:
                x, y = jittered_point(target_x, target_y, self.jitter_radius, step_index)
                payload = build_test_cursor_payload(
                    x,
                    y,
                    action["button_id"],
                    canvas_x,
                    canvas_y,
                    self.rospy.get_time() or time.time(),
                    test_sequence=self.test_sequence,
                    sequence_step=action["sequence_step"],
                    test_only=self.publish_test_only_marker,
                )
                self.publisher.publish(self.String(data=json.dumps(payload, sort_keys=True)))
                step_index += 1
                total_samples += 1
                rate.sleep()

            delay_after_sec = max(0.0, float(action.get("delay_after_sec", 0.0)))
            if delay_after_sec > 0.0 and not self.rospy.is_shutdown():
                self.rospy.sleep(delay_after_sec)

        self.rospy.loginfo("dashboard_dwell_test_cursor_node finished after %d samples", total_samples)


def main():
    import rospy

    rospy.init_node("dashboard_dwell_test_cursor_node")
    try:
        DashboardDwellTestCursorNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
