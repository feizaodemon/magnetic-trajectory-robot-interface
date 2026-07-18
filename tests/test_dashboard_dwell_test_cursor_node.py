import pytest

from colmag_ros.scripts.dashboard_dwell_test_cursor_node import (
    LAYOUT_GAMEPAD,
    TEST_SEQUENCE_GAMEPAD_A_BLOCK_THEN_X,
    TEST_SEQUENCE_GAMEPAD_B_THEN_C,
    build_test_cursor_payload,
    build_test_sequence_actions,
    find_virtual_button_rect,
    rect_center,
    target_world_point_for_button,
)
from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
    build_canvas_reachable_preview_hit_zones,
    build_gamepad_button_rects,
    build_virtual_button_rects,
    get_virtual_button_under_cursor,
    map_trajectory_point,
    map_world_to_canvas,
    resolve_preview_button_hit,
)


def test_rank_1_button_center_can_be_computed():
    rects = build_virtual_button_rects(560, 520, 20)
    rect = find_virtual_button_rect(rects, "rank_1")
    assert rect is not None
    cx, cy = rect_center(rect)
    assert rect["x1"] < cx < rect["x2"]
    assert rect["y1"] < cy < rect["y2"]


def test_target_world_point_maps_back_into_rank_1_button_rect():
    bounds = (-0.18, 0.18, -0.18, 0.18)
    target, canvas_target, reason = target_world_point_for_button("rank_1", 560, 520, 20, bounds)
    assert reason == ""
    assert target is not None
    assert canvas_target is not None

    canvas_x, canvas_y = map_world_to_canvas(target[0], target[1], bounds, 560, 520, 20)
    rects = build_virtual_button_rects(560, 520, 20)
    assert canvas_x == pytest.approx(canvas_target[0])
    assert canvas_y == pytest.approx(canvas_target[1])
    assert get_virtual_button_under_cursor(canvas_x, canvas_y, rects) == "rank_1"


@pytest.mark.parametrize("button_id", ["B", "C", "A", "U"])
def test_gamepad_button_center_maps_to_world_then_back_into_rect(button_id):
    bounds = (-0.18, 0.18, -0.18, 0.18)
    target, canvas_target, reason = target_world_point_for_button(
        button_id,
        560,
        520,
        20,
        bounds,
        layout=LAYOUT_GAMEPAD,
    )
    assert reason == ""
    assert target is not None
    assert canvas_target is not None

    canvas_x, canvas_y = map_world_to_canvas(target[0], target[1], bounds, 560, 520, 20)
    rects = build_gamepad_button_rects(560, 520, 20)
    assert canvas_x == pytest.approx(canvas_target[0])
    assert canvas_y == pytest.approx(canvas_target[1])
    assert get_virtual_button_under_cursor(canvas_x, canvas_y, rects) == button_id


def test_canvas_reachable_preview_controls_are_inside_board_cursor_domain_and_off_center():
    width, height, padding, button_size = 560, 407, 20, 58
    bounds = (-0.07, 0.07, -0.07, 0.09)
    corners = [
        map_trajectory_point(x, y, bounds, width, height, padding, clamp=True)
        for x in (bounds[0], bounds[1])
        for y in (bounds[2], bounds[3])
    ]
    reach_rect = (
        min(p[0] for p in corners),
        min(p[1] for p in corners),
        max(p[0] for p in corners),
        max(p[1] for p in corners),
    )
    zones = build_canvas_reachable_preview_hit_zones(
        width, height, padding, button_size, reach_rect=reach_rect
    )
    zone_by_id = {zone["id"]: zone for zone in zones}
    assert set(zone_by_id) == {"X", "A", "B", "C"}
    center_x, center_y = width / 2.0, height / 2.0
    for zone in zones:
        radius = zone["radius"]
        assert padding <= zone["cx"] - radius
        assert zone["cx"] + radius <= width - padding
        assert padding <= zone["cy"] - radius
        assert zone["cy"] + radius <= height - padding
        assert abs(zone["cx"] - center_x) > radius
        assert abs(zone["cy"] - center_y) > radius
        assert reach_rect[0] <= zone["cx"] <= reach_rect[2]
        assert reach_rect[1] <= zone["cy"] <= reach_rect[3]
        assert resolve_preview_button_hit(zone["cx"], zone["cy"], zones) == zone["id"]


def test_test_cursor_payload_contains_test_only_marker_and_target():
    payload = build_test_cursor_payload(
        0.1,
        0.2,
        "rank_1",
        481.0,
        55.0,
        123.0,
        test_sequence="rank_1",
        sequence_step=0,
        test_only=True,
    )
    assert payload["x"] == 0.1
    assert payload["y"] == 0.2
    assert payload["timestamp"] == 123.0
    assert payload["source"] == "dashboard_dwell_test_cursor"
    assert payload["test_only"] is True
    assert payload["test_sequence"] == "rank_1"
    assert payload["sequence_step"] == 0
    assert payload["target_button_id"] == "rank_1"
    assert payload["canvas_target_x"] == 481.0
    assert payload["canvas_target_y"] == 55.0


def test_missing_target_button_returns_reason():
    target, canvas_target, reason = target_world_point_for_button(
        "missing",
        560,
        520,
        20,
        (-0.18, 0.18, -0.18, 0.18),
    )
    assert target is None
    assert canvas_target is None
    assert reason == "target_button_not_found"


def test_gamepad_b_then_c_sequence_contains_expected_order_and_delays():
    actions = build_test_sequence_actions(
        TEST_SEQUENCE_GAMEPAD_B_THEN_C,
        hold_sec=2.8,
        between_action_delay_sec=1.0,
        digit_mode_sec=3.0,
    )
    assert [action["button_id"] for action in actions] == ["B", "C"]
    assert [action["layout"] for action in actions] == [LAYOUT_GAMEPAD, LAYOUT_GAMEPAD]
    assert actions[0]["delay_after_sec"] == pytest.approx(4.0)
    assert actions[1]["delay_after_sec"] == pytest.approx(0.0)


def test_gamepad_a_block_then_x_sequence_contains_expected_order():
    actions = build_test_sequence_actions(
        TEST_SEQUENCE_GAMEPAD_A_BLOCK_THEN_X,
        hold_sec=2.8,
        between_action_delay_sec=1.0,
    )
    assert [action["button_id"] for action in actions] == ["A", "U", "X"]
    assert [action["layout"] for action in actions] == [LAYOUT_GAMEPAD, LAYOUT_GAMEPAD, LAYOUT_GAMEPAD]
    assert actions[0]["delay_after_sec"] == pytest.approx(1.0)
    assert actions[1]["delay_after_sec"] == pytest.approx(1.0)
    assert actions[2]["delay_after_sec"] == pytest.approx(0.0)


def test_test_cursor_helpers_do_not_build_command_topics():
    payload = build_test_cursor_payload(
        0.1,
        0.2,
        "B",
        440.0,
        326.0,
        123.0,
        test_sequence=TEST_SEQUENCE_GAMEPAD_B_THEN_C,
        sequence_step=0,
        test_only=True,
    )
    assert "confirmed_label" not in payload
    assert "task_command" not in payload
    assert "robot_command" not in payload
