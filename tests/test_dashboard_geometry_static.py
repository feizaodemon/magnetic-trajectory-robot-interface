import pytest

from colmag_ros.scripts.dashboard_geometry import (
    build_canvas_reachable_drawing_zone,
    build_canvas_reachable_preview_hit_zones,
    canvas_zones_to_stage_zones,
    compute_dashboard_panel_widths,
    compute_ocr_stage_geometry,
    dashboard_minimum_window_size,
    derive_orientation_clutch_mode,
    drawing_zone_style,
    get_virtual_button_under_cursor,
    is_point_inside_rect,
    map_trajectory_point,
    normalize_board_control_layout,
    preview_button_style,
    resolve_preview_button_hit,
    trajectory_point_in_drawing_zone,
)


@pytest.mark.parametrize(
    ("engaged", "inside_drawing", "button", "expected"),
    [
        (False, True, "B", "NAVIGATE"),
        (True, True, "", "DRAW"),
        (True, False, "B", "HOVER — B"),
        (True, False, "C", "HOVER — C"),
        (True, False, "A", "HOVER — A"),
        (True, False, "X", "HOVER — X"),
        (True, False, "", "IDLE"),
    ],
)
def test_orientation_clutch_mode_is_derived_without_parallel_state(
    engaged,
    inside_drawing,
    button,
    expected,
):
    assert derive_orientation_clutch_mode(engaged, inside_drawing, button) == expected


def test_ocr_stage_geometry_preserves_small_window_guard_and_reference_layout():
    assert compute_ocr_stage_geometry(99, 500) is None

    geometry = compute_ocr_stage_geometry(1060, 760)
    assert geometry["stage"] == (12, 14, 1036, 732)
    assert geometry["draw"] == (279.0, 154, 478, 478)
    assert geometry["padding"] == 12
    assert geometry["button_size"] == 64
    assert geometry["hover"] == (12, 710)
    assert geometry["button_rects"] == {
        "X": (486.0, 80),
        "A": (205.0, 361.0),
        "C": (767.0, 361.0),
        "B": (486.0, 642),
        "CLEAR": (767.0, 642),
    }


def test_dashboard_panel_split_keeps_readable_right_width_at_normal_and_maximized_sizes():
    normal_left, normal_right = compute_dashboard_panel_widths(1030)
    maximized_left, maximized_right = compute_dashboard_panel_widths(2470)

    assert (normal_left, normal_right) == (635, 380)
    assert (maximized_left, maximized_right) == (1645, 810)
    assert normal_right >= 380
    assert maximized_right / (maximized_left + maximized_right) == pytest.approx(
        0.33, rel=0.01
    )


def test_ocr_stage_geometry_preserves_clamps_and_canvas_aspect_ratio():
    geometry = compute_ocr_stage_geometry(2000, 1500)
    assert geometry["stage"] == (200, 150, 1600, 1200)
    assert geometry["draw"] == (327.0, 154, 946, 946)
    assert geometry["header_height"] == 80
    assert geometry["footer_height"] == 26
    assert geometry["gap"] == 10
    draw_width = geometry["draw"][2]
    draw_height = geometry["draw"][3]
    assert draw_width / draw_height == pytest.approx(1.0)


def test_minimum_window_and_drawing_zone_use_stable_neutral_contracts():
    assert dashboard_minimum_window_size() == (1060, 760)
    style = drawing_zone_style()
    assert style["outline"] != preview_button_style("C", True)["outline"]
    assert style["text"] != preview_button_style("C", True)["outline"]


def test_rect_hit_test_includes_edges_and_excludes_outside():
    rect = {"id": "rank_1", "x1": 10, "y1": 20, "x2": 50, "y2": 80}
    assert is_point_inside_rect(10, 20, rect)
    assert is_point_inside_rect(50, 80, rect)
    assert is_point_inside_rect(30, 40, rect)
    assert not is_point_inside_rect(9.99, 20, rect)
    assert not is_point_inside_rect(50.01, 80, rect)


def test_virtual_button_hit_uses_first_matching_rect():
    rects = [
        {"id": "first", "x1": 0, "y1": 0, "x2": 20, "y2": 20},
        {"id": "second", "x1": 10, "y1": 10, "x2": 30, "y2": 30},
    ]
    assert get_virtual_button_under_cursor(15, 15, rects) == "first"
    assert get_virtual_button_under_cursor(25, 25, rects) == "second"
    assert get_virtual_button_under_cursor(40, 40, rects) == ""


def test_preview_button_hit_uses_first_matching_zone_and_uppercases_id():
    zones = [
        {"id": "x", "cx": 20.0, "cy": 20.0, "radius": 20.0},
        {"id": "b", "cx": 25.0, "cy": 20.0, "radius": 20.0},
    ]
    assert resolve_preview_button_hit(23.0, 20.0, zones) == "X"
    assert resolve_preview_button_hit(45.0, 20.0, zones) == "B"
    assert resolve_preview_button_hit(100.0, 100.0, zones) == ""


def test_normalize_board_control_layout_aliases_and_fallback():
    assert normalize_board_control_layout("canvas") == "canvas_reachable"
    assert normalize_board_control_layout("INLINE_CONTROLS") == "canvas_reachable"
    assert normalize_board_control_layout("external_toolbar") == "external_toolbar"
    assert normalize_board_control_layout(None) == "external_toolbar"


def test_canvas_reachable_controls_keep_order_and_stay_outside_drawing_zone():
    width, height, padding, button_size = 560, 407, 20, 58
    reach_rect = (padding, padding, width - padding, height - padding)
    zones = build_canvas_reachable_preview_hit_zones(
        width, height, padding, button_size, reach_rect=reach_rect
    )
    drawing_zone = build_canvas_reachable_drawing_zone(
        width, height, padding, reach_rect=reach_rect
    )

    assert [zone["id"] for zone in zones] == ["X", "C", "A", "B"]
    for zone in zones:
        assert resolve_preview_button_hit(zone["cx"], zone["cy"], zones) == zone["id"]
        assert not is_point_inside_rect(zone["cx"], zone["cy"], drawing_zone)


@pytest.mark.parametrize(
    ("width", "height", "button_size"),
    [(478, 478, 64), (946, 946, 64)],
)
def test_canvas_reachable_controls_are_fully_inside_normal_and_maximized_canvas(
    width,
    height,
    button_size,
):
    padding = 20
    zones = build_canvas_reachable_preview_hit_zones(
        width,
        height,
        padding,
        button_size,
        reach_rect=(padding, padding, width - padding, height - padding),
    )

    assert [zone["id"] for zone in zones] == ["X", "C", "A", "B"]
    for zone in zones:
        safe_margin = max(8.0, padding * 0.5)
        assert zone["cx"] - zone["radius"] >= padding + safe_margin - 1e-9
        assert zone["cy"] - zone["radius"] >= padding + safe_margin - 1e-9
        assert zone["cx"] + zone["radius"] <= width - padding - safe_margin + 1e-9
        assert zone["cy"] + zone["radius"] <= height - padding - safe_margin + 1e-9
        assert resolve_preview_button_hit(zone["cx"], zone["cy"], zones) == zone["id"]


def test_minimum_and_maximized_drawing_zones_are_square_and_controls_do_not_overlap():
    for available_width, available_height in ((1060, 760), (2000, 1500)):
        geometry = compute_ocr_stage_geometry(available_width, available_height)
        _x, _y, width, height = geometry["draw"]
        assert width == height
        zones = build_canvas_reachable_preview_hit_zones(
            width, height, geometry["padding"], geometry["button_size"],
            reach_rect=(
                geometry["padding"], geometry["padding"],
                width - geometry["padding"], height - geometry["padding"],
            ),
        )
        drawing = build_canvas_reachable_drawing_zone(
            width, height, geometry["padding"],
            reach_rect=(
                geometry["padding"], geometry["padding"],
                width - geometry["padding"], height - geometry["padding"],
            ),
        )
        for zone in zones:
            assert not is_point_inside_rect(zone["cx"], zone["cy"], drawing)
            radius = zone["radius"]
            assert (
                zone["cx"] + radius <= drawing["x1"]
                or zone["cx"] - radius >= drawing["x2"]
                or zone["cy"] + radius <= drawing["y1"]
                or zone["cy"] - radius >= drawing["y2"]
            )


def test_canvas_reachable_board_layout_uses_hidden_toolbar_space_vertically():
    normal = compute_ocr_stage_geometry(
        615, 608, controls_inside_canvas=True)
    maximized = compute_ocr_stage_geometry(
        1645, 1200, controls_inside_canvas=True)

    assert normal["draw"][2:] == (485, 485)
    assert maximized["draw"][2:] == (1000, 1000)
    assert normal["draw"][3] > compute_ocr_stage_geometry(615, 608)["draw"][3]
    assert maximized["draw"][3] > normal["draw"][3]


def test_preview_button_style_has_one_disabled_contract_for_all_actions():
    disabled = [preview_button_style(button_id, False) for button_id in "BCAX"]
    assert len({style["fill"] for style in disabled}) == 4
    assert len({style["outline"] for style in disabled}) == 4
    assert all(style["text"] == "#94A3B8" for style in disabled)
    assert all(style["fill"] != "#F8FAFC" for style in disabled)

    assert preview_button_style("B", True)["fill"] == "#DBEAFE"
    assert preview_button_style("C", True)["fill"] == "#DCFCE7"
    assert preview_button_style("A", True)["fill"] == "#FEE2E2"
    assert preview_button_style("X", True)["fill"] == "#E2E8F0"
    assert preview_button_style("B", True, active=True)["width"] == 3
    assert preview_button_style("X", True)["outline"] == "#475569"
    assert preview_button_style("X", False)["outline"] == "#94A3B8"
    assert preview_button_style("X", True)["ring"] == "#334155"
    assert preview_button_style("X", True)["fill"] != preview_button_style("B", True)["fill"]
    assert preview_button_style("X", False)["fill"] != preview_button_style("B", False)["fill"]
    assert preview_button_style("A", True)["outline"] == "#DC2626"
    assert preview_button_style("C", True)["outline"] == "#16A34A"


def test_canvas_zones_to_stage_zones_offsets_centers_and_rects():
    zones = [
        {"id": "b", "label": "B\nRecognize", "cx": 10, "cy": 15, "radius": 5},
    ]
    converted = canvas_zones_to_stage_zones(zones, 100, 200)
    assert converted == [
        {
            "id": "B",
            "label": "B\nRecognize",
            "x1": 105.0,
            "y1": 210.0,
            "x2": 115.0,
            "y2": 220.0,
            "cx": 110.0,
            "cy": 215.0,
            "radius": 5.0,
        }
    ]


def test_map_trajectory_point_preserves_clamped_and_unclamped_semantics():
    bounds = (-1.0, 1.0, -1.0, 1.0)
    clamped = map_trajectory_point(2.0, 0.0, bounds, 100, 100, 10, clamp=True)
    unclamped = map_trajectory_point(2.0, 0.0, bounds, 100, 100, 10, clamp=False)

    assert clamped == (90, 50.0)
    assert unclamped[0] > 90
    assert unclamped[1] == pytest.approx(50.0)


def test_trajectory_point_in_drawing_zone_uses_unclamped_mapping_for_control_exclusion():
    bounds = (-1.0, 1.0, -1.0, 1.0)
    drawing_zone = {"x1": 30, "y1": 30, "x2": 70, "y2": 70}

    assert trajectory_point_in_drawing_zone(
        (0.0, 0.0), bounds, 100, 100, 10, drawing_zone
    )
    assert not trajectory_point_in_drawing_zone(
        (2.0, 0.0), bounds, 100, 100, 10, drawing_zone
    )
    assert not trajectory_point_in_drawing_zone(
        None, bounds, 100, 100, 10, drawing_zone
    )
