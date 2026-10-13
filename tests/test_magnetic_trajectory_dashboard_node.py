import pytest
import sys
from pathlib import Path
from magnetic_trajectory_dashboard_node import (
    safe_json_loads,
    extract_xy_points,
    extract_single_point,
    summarize_candidates,
    coerce_float,
    update_dwell_state,
    build_confirmed_label_payload,
    candidate_payload_key,
    real_board_sample_recording_dir,
    should_suppress_repeated_confirm,
)
from colmag_ros.dashboard_controller_input import cancelled_preview_hover_state
from colmag_ros.dashboard_geometry import (
    map_world_to_canvas,
    map_canvas_to_world,
    is_point_inside_rect,
    build_canvas_reachable_drawing_zone,
    build_canvas_reachable_preview_hit_zones,
    trajectory_point_in_drawing_zone,
)
from colmag_ros.dashboard_points import clean_stroke_points
from colmag_ros.dashboard_sample_lifecycle import (
    build_dashboard_drawing_zone_capture_payload,
    build_real_board_sample_record,
    cleanup_drawing_sample_points,
    sample_cleanup_metadata,
    safe_sample_recording_name,
    update_drawing_zone_sample_state,
)
from dashboard_source import dashboard_source


def test_catkin_wrapper_exec_adds_package_source_to_import_path():
    path = (
        Path(__file__).resolve().parents[1]
        / "colmag_ros"
        / "scripts"
        / "magnetic_trajectory_dashboard_node.py"
    )
    package_src = str(path.parents[1] / "src")
    helper_modules = (
        "colmag_ros.trajectory_segment_recorder",
        "colmag_ros.dashboard_confirm_publisher",
        "colmag_ros.dashboard_candidate_display",
        "colmag_ros.dashboard_points",
        "colmag_ros.dashboard_geometry",
        "colmag_ros.dashboard_dwell_status",
    )
    saved_path = list(sys.path)
    saved_modules = {name: sys.modules.get(name) for name in helper_modules}

    try:
        sys.path[:] = [entry for entry in sys.path if entry != package_src]
        for name in helper_modules:
            sys.modules.pop(name, None)

        context = {
            "__file__": str(path),
            "__name__": "catkin_devel_dashboard_wrapper_exec_test",
            "__package__": None,
        }
        exec(compile(path.read_text(), str(path), "exec"), context)

        assert str(context["_PACKAGE_SRC"]) == package_src
        assert sys.path[0] == package_src
        assert callable(context["format_integrated_dwell_status_texts"])
        assert callable(context["extract_xy_points"])
        assert callable(context["recognize_stroke"])
    finally:
        sys.path[:] = saved_path
        for name, module in saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

def test_safe_json_loads():
    assert safe_json_loads('{"a": 1}') == {"a": 1}
    assert safe_json_loads('[1, 2]') is None
    assert safe_json_loads('invalid') is None
    assert safe_json_loads('') is None

def test_extract_xy_points():
    assert extract_xy_points({}) == []

    # trajectory
    data1 = {"trajectory": [{"x": 0.1, "y": 0.2}, {"x": "0.3", "y": "0.4"}]}
    assert extract_xy_points(data1) == [(0.1, 0.2), (0.3, 0.4)]

    # points_2d with position
    data2 = {"points_2d": [{"position": {"x": 1, "y": 2}}, {"position": {"x": 3, "y": 4}}]}
    assert extract_xy_points(data2) == [(1.0, 2.0), (3.0, 4.0)]

    # path with point
    data3 = {"path": [{"point": {"x": 5, "y": 6}}]}
    assert extract_xy_points(data3) == [(5.0, 6.0)]

    # invalid items ignored
    data4 = {"points": [{"x": 1}, {"y": 2}, {"x": 3, "y": "a"}]}
    assert extract_xy_points(data4) == []

def test_extract_single_point():
    assert extract_single_point({"x": 0.5, "y": -0.5}) == (0.5, -0.5)
    assert extract_single_point({"position": {"x": 1, "y": 2}}) == (1.0, 2.0)
    assert extract_single_point({"point": {"x": 3, "y": 4}}) == (3.0, 4.0)
    assert extract_single_point({}) is None
    assert extract_single_point({"x": 1}) is None

def test_map_world_to_canvas():
    bounds = (-1.0, 1.0, -1.0, 1.0)
    w, h, p = 100, 100, 10

    # Center
    cx, cy = map_world_to_canvas(0.0, 0.0, bounds, w, h, p)
    assert cx == 50.0 and cy == 50.0

    # Top right
    cx, cy = map_world_to_canvas(1.0, 1.0, bounds, w, h, p)
    assert cx == 90.0 and cy == 10.0

    # Bottom left
    cx, cy = map_world_to_canvas(-1.0, -1.0, bounds, w, h, p)
    assert cx == 10.0 and cy == 90.0

    # Out of bounds clipping
    cx, cy = map_world_to_canvas(2.0, 2.0, bounds, w, h, p)
    assert cx == 90.0 and cy == 10.0

def test_map_canvas_to_world_inverts_world_to_canvas():
    bounds = (-1.0, 1.0, -1.0, 1.0)
    w, h, p = 100, 100, 10
    cx, cy = map_world_to_canvas(0.25, -0.5, bounds, w, h, p)
    x, y = map_canvas_to_world(cx, cy, bounds, w, h, p)
    assert x == pytest.approx(0.25)
    assert y == pytest.approx(-0.5)

def test_summarize_candidates():
    payload = {
        "candidates": [
            {"rank": 2, "label": "B", "confidence": 0.5},
            {"rank": "1", "label": "A", "confidence": "0.9"},
            {"rank": 3, "label": "C"}
        ]
    }
    tops = summarize_candidates(payload)
    assert len(tops) == 3
    assert tops[0] == (1, "A", 0.9)
    assert tops[1] == (2, "B", 0.5)
    assert tops[2] == (3, "C", 0.0)

def test_coerce_float():
    assert coerce_float(1) == 1.0
    assert coerce_float("1.5") == 1.5
    assert coerce_float("a", 2.0) == 2.0
    assert coerce_float(None, 0.0) == 0.0

def test_point_inside_virtual_button_rect():
    rect = {"id": "rank_1", "x1": 10, "y1": 20, "x2": 50, "y2": 80}
    assert is_point_inside_rect(10, 20, rect)
    assert is_point_inside_rect(30, 40, rect)
    assert is_point_inside_rect(50, 80, rect)

def test_point_outside_virtual_button_rect():
    rect = {"id": "rank_1", "x1": 10, "y1": 20, "x2": 50, "y2": 80}
    assert not is_point_inside_rect(9, 40, rect)
    assert not is_point_inside_rect(30, 81, rect)

def test_drawing_zone_is_central_and_separate_from_canvas_controls():
    from colmag_ros.dashboard_geometry import resolve_preview_button_hit

    width, height, padding, button_size = 560, 407, 20, 58
    reach_rect = (padding, padding, width - padding, height - padding)
    drawing_zone = build_canvas_reachable_drawing_zone(
        width, height, padding, reach_rect=reach_rect
    )
    controls = build_canvas_reachable_preview_hit_zones(
        width, height, padding, button_size, reach_rect=reach_rect
    )

    center_x, center_y = width / 2.0, height / 2.0
    assert is_point_inside_rect(center_x, center_y, drawing_zone)
    for control in controls:
        assert not is_point_inside_rect(control["cx"], control["cy"], drawing_zone)
        assert resolve_preview_button_hit(control["cx"], control["cy"], controls) == control["id"]
        assert control["radius"] < button_size / 2.0

    zone_width = drawing_zone["x2"] - drawing_zone["x1"]
    zone_height = drawing_zone["y2"] - drawing_zone["y1"]
    assert zone_width == pytest.approx(zone_height)
    assert zone_width >= 0.60 * min(width - 2 * padding, height - 2 * padding)

def test_preview_hover_cancel_resets_dwell_only_contract():
    cancelled = cancelled_preview_hover_state()
    assert cancelled == {
        "current_button_id": "",
        "started_at": None,
        "dwell_progress": 0.0,
        "activated_button_id": "",
    }

def test_preview_pointer_leave_does_not_clear_sample_or_candidate_state():
    text = dashboard_source()
    clear_start = text.index("def _clear_ocr_hover_button(")
    clear_body = text[clear_start:text.index("\n    def _update_ocr_toolbar_visuals", clear_start)]
    forbidden = (
        "_clear_trajectory_trace",
        "_reject_preview_candidate",
        "_publish_trajectory_symbol_capture",
        "captured_path = []",
        "current_candidate_payload = None",
        "trajectory_candidates = []",
        "_reset_drawing_sample_state",
        "_clear_published_sample_state",
    )
    for token in forbidden:
        assert token not in clear_body

def test_trajectory_keyboard_c_confirms_and_x_clears():
    text = dashboard_source()
    key_start = text.index("def _on_key_press(")
    key_body = text[key_start:]
    trajectory_branch = key_body[key_body.index("if self.is_trajectory_mode:"):key_body.index("if self.demo_input_mode != \"ocr_canvas\":")]
    c_branch = trajectory_branch[trajectory_branch.index('elif key == "c"'):trajectory_branch.index('elif key in ("a", "x")')]
    assert '_handle_preview_button_action("C")' in c_branch
    assert "_clear_trajectory_trace()" not in c_branch
    ax_branch = trajectory_branch[trajectory_branch.index('elif key in ("a", "x")'):trajectory_branch.index('elif keysym in ("return", "kp_enter")')]
    assert "_handle_preview_button_action(key)" in ax_branch
    action_body_start = text.index("def _handle_preview_button_action(")
    action_body = text[action_body_start:text.index("\n    def _preview_button_enabled", action_body_start)]
    assert "_preview_confirm_candidate()" in action_body
    assert "_clear_trajectory_trace()" in action_body
    assert "_reject_preview_candidate()" in action_body

def test_control_zone_points_are_ignored_by_drawing_zone_filter():
    bounds = (-1.0, 1.0, -1.0, 1.0)
    width, height, padding = 100, 100, 10
    drawing_zone = build_canvas_reachable_drawing_zone(
        width, height, padding, reach_rect=(padding, padding, width - padding, height - padding)
    )
    assert trajectory_point_in_drawing_zone(
        (0.0, 0.0), bounds, width, height, padding, drawing_zone
    )
    assert not trajectory_point_in_drawing_zone(
        (0.95, 0.95), bounds, width, height, padding, drawing_zone
    )
    assert not trajectory_point_in_drawing_zone(
        (-0.95, -0.95), bounds, width, height, padding, drawing_zone
    )

def test_drawing_zone_capture_payload_uses_only_filtered_points():
    drawing_points = [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]
    control_hover_tail = [(0.9, -0.9)]
    payload = build_dashboard_drawing_zone_capture_payload(
        drawing_points,
        "sample_1",
        42,
        123.0,
        "/colmag/trajectory_2d",
        drawing_zone={"x1": 20, "y1": 20, "x2": 80, "y2": 80},
    )
    assert payload["capture_mode"] == "dashboard_drawing_zone"
    assert payload["controls_excluded_from_sample"] is True
    assert payload["controls_real_robot"] is False
    assert payload["points"] == [[float(x), float(y)] for x, y in drawing_points]
    assert payload["points"] != [[float(x), float(y)] for x, y in drawing_points + control_hover_tail]

def test_clean_stroke_points_drops_drawing_zone_breaks():
    assert clean_stroke_points([(0, 0), None, (1, 1)]) == [(0, 0), (1, 1)]

def test_cleanup_drawing_sample_points_removes_near_duplicates_and_preserves_order():
    raw = [
        (0.0, 0.0),
        (0.0001, 0.0),
        (0.0100, 0.0),
        (0.0102, 0.0),
        (0.0200, 0.0),
    ]
    cleaned = cleanup_drawing_sample_points(
        raw,
        min_point_delta=0.005,
        max_points=0,
        enabled=True,
    )
    assert cleaned == [(0.0, 0.0), (0.0100, 0.0), (0.0200, 0.0)]

def test_cleanup_drawing_sample_points_caps_deterministically():
    raw = [(float(i), 0.0) for i in range(10)]
    cleaned = cleanup_drawing_sample_points(
        raw,
        min_point_delta=0.0,
        max_points=4,
        enabled=True,
    )
    assert cleaned == [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0), (9.0, 0.0)]
    assert cleaned == cleanup_drawing_sample_points(raw, min_point_delta=0.0, max_points=4, enabled=True)

def test_dashboard_capture_payload_records_raw_and_published_sample_counts():
    raw = [(0.0, 0.0), (0.0001, 0.0), (0.01, 0.0)]
    cleaned = cleanup_drawing_sample_points(raw, min_point_delta=0.005, max_points=0)
    metadata = sample_cleanup_metadata(raw, cleaned, min_point_delta=0.005, max_points=0, enabled=True)
    payload = build_dashboard_drawing_zone_capture_payload(
        cleaned,
        "sample_cleaned",
        7,
        123.0,
        "/colmag/trajectory_2d",
        raw_points=raw,
        cleanup_metadata=metadata,
    )
    assert payload["points"] == [[0.0, 0.0], [0.01, 0.0]]
    assert payload["raw_point_count"] == 3
    assert payload["published_point_count"] == 2
    assert payload["sample_cleanup_enabled"] is True
    assert payload["sample_min_point_delta"] == 0.005

def test_real_board_sample_recording_dir_is_forced_under_outputs():
    assert real_board_sample_recording_dir("").as_posix() == "outputs/runtime_samples/real_board"
    assert real_board_sample_recording_dir(
        "outputs/runtime_samples/real_board/session_1"
    ).as_posix() == "outputs/runtime_samples/real_board/session_1"
    assert real_board_sample_recording_dir("/tmp/outside").as_posix() == (
        "outputs/runtime_samples/real_board"
    )
    assert real_board_sample_recording_dir("../outside").as_posix() == (
        "outputs/runtime_samples/real_board"
    )

def test_build_real_board_sample_record_keeps_raw_cleaned_and_candidate_payload():
    raw = [(0.0, 0.0), (0.0001, 0.0), (0.01, 0.0)]
    cleaned = cleanup_drawing_sample_points(raw, min_point_delta=0.005)
    metadata = sample_cleanup_metadata(raw, cleaned, min_point_delta=0.005, max_points=160, enabled=True)
    capture_payload = build_dashboard_drawing_zone_capture_payload(
        cleaned,
        "sample/1",
        9,
        123.0,
        "/colmag/trajectory_2d",
        raw_points=raw,
        cleanup_metadata=metadata,
    )
    candidate_payload = {"sample_id": "sample/1", "accepted": True, "candidates": [{"rank": 1, "label": "1"}]}
    record = build_real_board_sample_record(
        capture_payload=capture_payload,
        raw_points=raw,
        cleaned_points=cleaned,
        cleanup_metadata=metadata,
        candidate_payload=candidate_payload,
    )
    assert safe_sample_recording_name("sample/1") == "sample_1"
    assert record["record_schema"] == "m104_g3b_f6_real_board_sample_v1"
    assert record["sample_id"] == "sample/1"
    assert len(record["raw_points"]) == 3
    assert len(record["cleaned_points"]) == 2
    assert record["cleanup"]["published_point_count"] == 2
    assert record["latest_candidate_payload"] == candidate_payload
    assert record["latest_candidate_may_be_stale"] is False
    assert "not used for recognition" in record["note"]

def test_drawing_sample_freezes_when_valid_input_enters_control_zone():
    state = {}
    for point in [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]:
        state = update_drawing_zone_sample_state(
            state, point, inside_drawing_zone=True, input_active=True
        )
    state = update_drawing_zone_sample_state(
        state, (0.7, 0.7), inside_drawing_zone=False, input_active=True
    )
    assert state["frozen_points"] == []
    state = update_drawing_zone_sample_state(
        state, (0.9, 0.9), inside_drawing_zone=False, input_active=True,
        inside_control_zone=True
    )
    state = update_drawing_zone_sample_state(
        state, (0.5, 0.0), inside_drawing_zone=True, input_active=True
    )

    assert clean_stroke_points(state["active_points"]) == [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]
    assert clean_stroke_points(state["frozen_points"]) == [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]
    assert (0.7, 0.7) not in state["frozen_points"]
    assert (0.9, 0.9) not in state["frozen_points"]
    assert (0.5, 0.0) not in state["frozen_points"]

def test_invalid_input_enters_pen_up_pending_without_appending_noise():
    state = update_drawing_zone_sample_state(
        {}, (0.0, 0.0), inside_drawing_zone=True, input_active=True
    )
    state = update_drawing_zone_sample_state(
        state, (0.9, 0.9), inside_drawing_zone=False, input_active=False
    )
    assert clean_stroke_points(state["active_points"]) == [(0.0, 0.0)]
    assert state["active_points"][-1] is None
    assert state["frozen_points"] == []
    assert state["was_inside"] is False
    assert state["phase"] == "PEN_UP_PENDING"

def test_multistroke_sample_can_resume_after_pen_up_before_control_zone():
    state = update_drawing_zone_sample_state(
        {}, (0.0, 0.0), inside_drawing_zone=True, input_active=True
    )
    state = update_drawing_zone_sample_state(
        state, (0.9, 0.9), inside_drawing_zone=False, input_active=False
    )
    state = update_drawing_zone_sample_state(
        state, (0.2, 0.2), inside_drawing_zone=True, input_active=True
    )
    state = update_drawing_zone_sample_state(
        state, (0.4, 0.4), inside_drawing_zone=False, input_active=True,
        inside_control_zone=True
    )
    assert clean_stroke_points(state["frozen_points"]) == [(0.0, 0.0), (0.2, 0.2)]
    assert state["active_points"][1] is None
    assert (0.9, 0.9) not in state["frozen_points"]

def test_transition_from_frozen_sample_to_b_does_not_change_sample():
    state = {}
    drawn = [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]
    for point in drawn:
        state = update_drawing_zone_sample_state(
            state, point, inside_drawing_zone=True, input_active=True
        )
    state = update_drawing_zone_sample_state(
        state, (0.8, 0.8), inside_drawing_zone=False, input_active=False
    )
    state = update_drawing_zone_sample_state(
        state, (0.95, -0.95), inside_drawing_zone=False, input_active=True,
        inside_control_zone=True
    )
    frozen = list(state["frozen_points"])
    state = update_drawing_zone_sample_state(
        state, (0.95, -0.95), inside_drawing_zone=False, input_active=True
    )
    state = update_drawing_zone_sample_state(
        state, (0.40, 0.40), inside_drawing_zone=True, input_active=True
    )
    assert clean_stroke_points(frozen) == drawn
    assert state["frozen_points"] == frozen
    assert (0.95, -0.95) not in state["frozen_points"]
    assert (0.40, 0.40) not in state["frozen_points"]

def test_b_recognize_can_force_freeze_current_drawing_buffer():
    state = {}
    drawn = [(0.0, -0.5), (0.0, 0.0), (0.0, 0.5)]
    for point in drawn:
        state = update_drawing_zone_sample_state(
            state, point, inside_drawing_zone=True, input_active=True
        )
    state = update_drawing_zone_sample_state(
        state, None, inside_drawing_zone=False, input_active=True,
        inside_control_zone=True, append_point=False, force_freeze=True
    )
    assert clean_stroke_points(state["frozen_points"]) == drawn

def test_dwell_does_not_activate_before_threshold():
    state = update_dwell_state({}, "rank_1", 10.0, 2.0)
    state = update_dwell_state(state, "rank_1", 11.0, 2.0)
    assert state["current_button_id"] == "rank_1"
    assert state["dwell_progress"] == 0.5
    assert state["activated_button_id"] == ""

def test_dwell_activates_after_threshold():
    state = update_dwell_state({}, "rank_1", 10.0, 2.0)
    state = update_dwell_state(state, "rank_1", 12.1, 2.0)
    assert state["dwell_progress"] == 1.0
    assert state["activated_button_id"] == "rank_1"

def test_dwell_resets_when_cursor_leaves():
    state = update_dwell_state({}, "rank_1", 10.0, 2.0)
    state = update_dwell_state(state, "", 11.0, 2.0)
    assert state["current_button_id"] == ""
    assert state["started_at"] is None
    assert state["dwell_progress"] == 0.0
    assert state["activated_button_id"] == ""

def test_repeated_confirm_is_suppressed_for_same_sample_and_rank():
    payload = {
        "sample_id": "sample_1",
        "sequence_id": 7,
        "candidates": [{"rank": 1, "label": "2", "confidence": 0.9}],
    }
    last_key = "%s:rank_1" % candidate_payload_key(payload)
    assert should_suppress_repeated_confirm(last_key, payload, 1)
    assert not should_suppress_repeated_confirm(last_key, payload, 2)

def test_confirmed_label_payload_contains_expected_fields():
    payload = {
        "backend": "dtw_trajectory_top3",
        "feature_mode": "trajectory_dtw",
        "source_topic": "/colmag/symbol_capture",
        "sample_id": "sample_1",
        "sequence_id": 7,
        "candidates": [
            {"rank": 1, "label": "2", "confidence": 0.9},
            {"rank": 2, "label": "1", "confidence": 0.7},
        ],
    }
    out, reason = build_confirmed_label_payload(payload, 1, now_fn=lambda: 123.0)
    assert reason == ""
    assert out["timestamp"] == 123.0
    assert out["confirmed"] is True
    assert out["confirmed_by"] == "magnetic_dashboard_dwell"
    assert out["selected_rank"] == 1
    assert out["label"] == "2"
    assert out["confidence"] == 0.9
    assert out["sample_id"] == "sample_1"
    assert out["sequence_id"] == 7
    assert out["backend"] == "dtw_trajectory_top3"
    assert out["candidates"] == payload["candidates"]
    assert out["controls_real_robot"] is False
    assert out["test_only"] is False
    assert out["source_topic"] == "/colmag/symbol_capture"

def test_integrated_confirm_disabled_gate_returns_no_publish_payload():
    payload = {
        "sample_id": "sample_1",
        "sequence_id": 7,
        "candidates": [{"rank": 1, "label": "2", "confidence": 0.9}],
    }
    integrated_confirm_enabled = False
    publish_payload = build_confirmed_label_payload(payload, 1)[0] if integrated_confirm_enabled else None
    assert publish_payload is None
