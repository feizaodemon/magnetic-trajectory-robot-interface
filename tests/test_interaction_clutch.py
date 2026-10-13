from colmag_ros import dashboard_controller_input as controls
from colmag_ros import dashboard_geometry as geometry
from colmag_ros import dashboard_sample_lifecycle as lifecycle
from colmag_ros.dashboard_confirm_publisher import build_mouse_toolbar_stop_payload
import magnetic_trajectory_dashboard_node as dashboard
from magnetic_trajectory_dashboard_node import (
    MagneticTrajectoryDashboardNode,
)
from dashboard_harness import SAMPLE_CAPTURE, DashboardHarness, default_config

CONFIRM_PARAMS = {
    "integrated_dashboard_confirm_enabled": True,
    "publish_confirmed_label": True,
}


def _dispatch_probe():
    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    actions = []
    node.config = default_config(
        confirm_candidate_source="external_symbol_candidates",
        candidate_preview_enabled=True,
        hover_click_enabled=True,
    )
    node._external_confirm_active = lambda: False
    node._publish_trajectory_symbol_capture_from_drawing_zone = (
        lambda: actions.append("recognize")
    )
    node._recognize_current_stroke = lambda: actions.append("internal_recognize")
    node._preview_confirm_candidate = lambda: actions.append("confirm")
    node._reject_preview_candidate = lambda: actions.append("reject")
    node._clear_trajectory_trace = lambda: actions.append("clear")
    node._update_ocr_toolbar_visuals = lambda: None
    node._preview_button_enabled = lambda _button_id: True
    return node, actions


def _path_length(points):
    clean = lifecycle.clean_stroke_points(points)
    return sum(
        ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        for (x1, y1), (x2, y2) in zip(clean, clean[1:])
    )


def test_two_stage_interaction_clutch_end_to_end_contract():
    state = {}
    dwell = {}
    recognition_count = 0
    candidate = None
    confirmed = None

    # 1. NAVIGATE updates position only; the disengaged clutch masks button dwell.
    position = (0.0, 0.0)
    state = lifecycle.update_drawing_zone_sample_state(
        state, position, inside_drawing_zone=True, input_active=False
    )
    dwell = controls.update_dwell_state(dwell, "", 0.0, 1.0)
    assert geometry.derive_orientation_clutch_mode(False, True, "B") == "NAVIGATE"
    assert lifecycle.clean_stroke_points(state["active_points"]) == []
    assert dwell["dwell_progress"] == 0.0
    assert recognition_count == 0

    # 2. DRAW appends points, but completing the drawing does not recognize.
    for point in ((0.0, 0.0), (0.0, 0.2), (0.0, 0.4)):
        position = point
        state = lifecycle.update_drawing_zone_sample_state(
            state, point, inside_drawing_zone=True, input_active=True
        )
    drawn_points = list(lifecycle.clean_stroke_points(state["active_points"]))
    assert geometry.derive_orientation_clutch_mode(True, True, "") == "DRAW"
    assert drawn_points == [(0.0, 0.0), (0.0, 0.2), (0.0, 0.4)]
    assert recognition_count == 0

    # 3. NAVIGATE to B preserves the sample and cannot accumulate dwell.
    state = lifecycle.update_drawing_zone_sample_state(
        state, (0.3, 0.5), inside_drawing_zone=True, input_active=False
    )
    dwell = controls.update_dwell_state(dwell, "", 1.0, 1.0)
    assert lifecycle.clean_stroke_points(state["active_points"]) == drawn_points
    assert dwell["activated_button_id"] == ""

    # 4. HOVER B freezes and submits once; B never confirms.
    state = lifecycle.update_drawing_zone_sample_state(
        state,
        (0.5, 0.5),
        inside_drawing_zone=False,
        input_active=True,
        inside_control_zone=True,
    )
    dwell = controls.update_dwell_state({}, "B", 2.0, 1.0)
    dwell = controls.update_dwell_state(dwell, "B", 3.0, 1.0)
    node, actions = _dispatch_probe()
    node._handle_preview_button_action(dwell["activated_button_id"])
    recognition_count += actions.count("recognize")
    candidate = {
        "sample_id": "sample-1",
        "sequence_id": 1,
        "candidates": [{"rank": 1, "label": "2", "confidence": 0.95}],
    }
    assert geometry.derive_orientation_clutch_mode(True, False, "B") == "HOVER — B"
    assert recognition_count == 1
    assert actions == ["recognize"]
    assert confirmed is None

    # Remaining over B cannot fire a second action without leaving/re-entry.
    dwell = controls.update_dwell_state(dwell, "B", 4.0, 1.0)
    assert dwell["activated_button_id"] == "B"
    assert actions == ["recognize"]

    # 5. NAVIGATE to C preserves candidate/sample identity and resets dwell.
    candidate_before_navigation = candidate
    sample_before_navigation = list(state["frozen_points"])
    dwell = controls.update_dwell_state(dwell, "", 5.0, 1.0)
    assert candidate is candidate_before_navigation
    assert state["frozen_points"] == sample_before_navigation
    assert dwell["dwell_progress"] == 0.0
    assert recognition_count == 1

    # 6. HOVER C confirms the same candidate without another recognition.
    dwell = controls.update_dwell_state({}, "C", 6.0, 1.0)
    dwell = controls.update_dwell_state(dwell, "C", 7.0, 1.0)
    node._handle_preview_button_action(dwell["activated_button_id"])
    confirmed, reason = controls.build_confirmed_label_payload(candidate, 1)
    assert reason == ""
    assert confirmed["label"] == candidate["candidates"][0]["label"]
    assert actions == ["recognize", "confirm"]
    assert recognition_count == 1

    # A rejects only and X clears only; a NAVIGATE crossing dispatches nothing.
    node._handle_preview_button_action("A")
    node._handle_preview_button_action("X")
    assert actions == ["recognize", "confirm", "reject", "clear"]


def test_disengaged_transfer_tail_is_excluded_but_all_engaged_tail_contaminates_geometry():
    clean_state = {}
    contaminated_state = {}
    drawing = [(0.0, 0.0), (0.0, 0.2), (0.0, 0.4)]
    transfer_tail = [(0.1, 0.45), (0.2, 0.5), (0.3, 0.55)]

    for point in drawing:
        clean_state = lifecycle.update_drawing_zone_sample_state(
            clean_state, point, inside_drawing_zone=True, input_active=True
        )
        contaminated_state = lifecycle.update_drawing_zone_sample_state(
            contaminated_state, point, inside_drawing_zone=True, input_active=True
        )
    for point in transfer_tail:
        clean_state = lifecycle.update_drawing_zone_sample_state(
            clean_state, point, inside_drawing_zone=True, input_active=False
        )
        contaminated_state = lifecycle.update_drawing_zone_sample_state(
            contaminated_state, point, inside_drawing_zone=True, input_active=True
        )

    clean_points = lifecycle.clean_stroke_points(clean_state["active_points"])
    contaminated_points = lifecycle.clean_stroke_points(
        contaminated_state["active_points"]
    )
    assert clean_points == drawing
    assert contaminated_points == drawing + transfer_tail
    assert len(contaminated_points) > len(clean_points)
    assert _path_length(contaminated_points) > _path_length(clean_points)


def test_visible_button_dwell_fires_once_and_navigation_resets_it(monkeypatch):
    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    actions = []
    node.latest_input_active = True
    node.latest_inside_drawing_zone = False
    node.config = default_config(
        board_cursor_dwell_enabled=True,
        hover_dwell_enabled=False,
        control_dwell_sec=1.0,
        hover_progress_enabled=True,
    )
    node.mouse_hover_button = ""
    node.board_hover_button = ""
    node.hover_source = "none"
    node.ocr_hover_button = ""
    node._dwell_active_button = ""
    node.ocr_hover_started_at = None
    node.ocr_dwell_progress = 0.0
    node.ocr_dwell_fired = False
    node._board_cursor_preview_button = lambda: "B"
    node._preview_button_enabled = lambda button_id: button_id == "B"
    node._handle_preview_button_action = lambda button_id: actions.append(button_id)
    node._refresh_dwell_status = lambda: None
    node._update_ocr_toolbar_visuals = lambda: None
    node._update_orientation_clutch_mode = lambda button="": None
    node._schedule_ui_refresh = lambda: None
    node._publish_dashboard_state = lambda: None

    clock = iter((10.0, 11.0, 12.0, 13.0))
    monkeypatch.setattr(dashboard.time, "time", lambda: next(clock))
    node._process_preview_dwell()
    node._process_preview_dwell()
    node._process_preview_dwell()
    assert actions == ["B"]

    node.latest_input_active = False
    node._process_preview_dwell()
    assert node.dwell_state == controls.cancelled_preview_hover_state()
    assert actions == ["B"]


def test_disabled_visible_button_does_not_accumulate_dwell_or_fire(monkeypatch):
    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    actions = []
    node.latest_input_active = True
    node.config = default_config(
        board_cursor_dwell_enabled=True,
        hover_dwell_enabled=False,
        control_dwell_sec=1.0,
        hover_progress_enabled=True,
    )
    node.mouse_hover_button = ""
    node.board_hover_button = ""
    node.hover_source = "none"
    node.ocr_hover_button = ""
    node._dwell_active_button = ""
    node.ocr_hover_started_at = None
    node.ocr_dwell_progress = 0.0
    node.ocr_dwell_fired = False
    node._board_cursor_preview_button = lambda: "C"
    node._preview_button_enabled = lambda _button_id: False
    node._handle_preview_button_action = lambda button_id: actions.append(button_id)
    node._refresh_dwell_status = lambda: None
    node._update_ocr_toolbar_visuals = lambda: None
    node._update_orientation_clutch_mode = lambda button="": None
    node._schedule_ui_refresh = lambda: None
    node._publish_dashboard_state = lambda: None

    clock = iter((10.0, 12.0))
    monkeypatch.setattr(dashboard.time, "time", lambda: next(clock))
    node._process_preview_dwell()
    node._process_preview_dwell()

    assert node.ocr_dwell_progress == 0.0
    assert node.dwell_state["current_button_id"] == ""
    assert node.dwell_state["activated_button_id"] == ""
    assert actions == []


def test_navigation_blocks_preview_pointer_and_keyboard_actions():
    node, actions = _dispatch_probe()
    node.latest_input_active = False
    node._preview_pointer_tracking_enabled = lambda: True
    node._preview_button_at_event = lambda _event: "B"
    node._set_ocr_hover_button = lambda button_id: actions.append("hover:" + button_id)

    assert node._handle_preview_pointer_click(object()) is None

    event = type("Event", (), {"char": "b", "keysym": "b"})()
    node._on_key_press(event)
    assert actions == []


def test_normal_dashboard_exposes_mode_but_keeps_sample_metrics_debug_only(monkeypatch):
    normal = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    debug = DashboardHarness(
        monkeypatch, candidate_preview_enabled=True, dashboard_debug_sample_panel=True)
    operator_vars = (
        "traj_clutch_mode_var", "traj_operator_status_var", "traj_operator_result_var")
    debug_only_vars = (
        "traj_result_var", "traj_valid_var", "traj_xyz_var", "traj_count_var",
        "traj_input_gate_var", "traj_captured_sample_var",
        "traj_candidate_points_var", "traj_sample_cleanup_var",
    )
    for harness in (normal, debug):
        for name in operator_vars:
            assert getattr(harness.node, name) in harness.shown_variables(), name
    for name in debug_only_vars:
        assert getattr(normal.node, name) not in normal.shown_variables(), name
        assert getattr(debug.node, name) in debug.shown_variables(), name

    # The captured-sample canvas label is debug-only too.
    for harness in (normal, debug):
        harness.send(harness.node.config.symbol_capture_topic, SAMPLE_CAPTURE)
    assert not any(text.startswith("Captured sample") for text in normal.canvas_texts())
    assert "Captured sample (3 pts)" in debug.canvas_texts()


def test_canvas_reachable_layout_labels_only_the_drawing_zone(monkeypatch):
    harness = DashboardHarness(
        monkeypatch, candidate_preview_enabled=True,
        dashboard_board_control_layout="canvas_reachable")
    harness.fire(harness.node.ocr_gamepad_frame, "<Configure>")

    canvas_texts = harness.canvas_texts()
    widget_texts = [str(widget.options.get("text", "")) for widget in harness.widgets()]
    assert "Drawing zone" in canvas_texts
    for removed in (
        "only this area enters DTW",
        "Controls: hover only",
        "Board input — dwell on controls to activate",
    ):
        assert not any(removed in text for text in canvas_texts + widget_texts)
    assert not hasattr(harness.node, "ocr_hover_label")


def test_disabled_preview_action_is_inert_and_never_dispatches():
    node, actions = _dispatch_probe()
    node._preview_button_enabled = lambda _button_id: False

    assert node._handle_preview_button_action("B") is False
    assert node._handle_preview_button_action("C") is False
    assert node._handle_preview_button_action("A") is False
    assert node._handle_preview_button_action("X") is False
    assert actions == []


def _hover_dwell(harness, button_id, seconds):
    """Mouse-hover a toolbar button and let the 50 ms dwell loop tick twice."""
    harness.fire(harness.node.ocr_button_widgets[button_id]["canvas"], "<Enter>")
    harness.run_jobs(50)
    harness.advance(seconds)
    harness.run_jobs(50)


def test_existing_dwell_progress_remains_the_only_hover_ring_owner(monkeypatch):
    for layout, ring_canvas in (
        ("external_toolbar", lambda node: node.ocr_button_widgets["C"]["canvas"]),
        ("canvas_reachable", lambda node: node.canvas),
    ):
        harness = DashboardHarness(
            monkeypatch, candidate_preview_enabled=True, hover_dwell_enabled=True,
            dashboard_board_control_layout=layout)
        node = harness.node
        harness.fire(node.ocr_gamepad_frame, "<Configure>")

        # B has no drawn sample, so it is disabled: hovering it draws no ring.
        _hover_dwell(harness, "B", 0.5)
        assert node.ocr_dwell_progress == 0.0
        assert ring_canvas(node).drawn("arc") == []

        harness.send_sample_and_candidates()
        _hover_dwell(harness, "C", 0.5)
        ring = geometry.preview_button_style("C", True, active=True)["ring"]
        assert node.ocr_dwell_progress == 0.5, layout
        arcs = ring_canvas(node).drawn("arc")
        assert [(arc["outline"], arc["extent"]) for arc in arcs] == [(ring, -180.0)], layout
        # Drawing the ring schedules nothing; only the dwell loop's next tick is queued.
        assert [job[1] for job in harness.root.jobs] == [50], layout

    # Rejecting mid-dwell disables C; the canvas ring disappears before the next tick.
    harness.press_key("a")
    assert node.ocr_dwell_progress == 0.5
    assert node.canvas.drawn("arc") == []


def test_responsive_operator_shell_enforces_accepted_minimum_and_expands_cards(monkeypatch):
    harness = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    main_frame = harness.root.children[0]

    assert harness.root.window["minsize"] == geometry.dashboard_minimum_window_size()
    assert main_frame.pack_options == {"fill": "both", "expand": True}
    assert harness.card_titles() == ["Interaction", "Progress", "Recognition", "Status"]
    assert harness.card("Recognition").pack_options["expand"] is True


def test_shared_canvas_uses_source_specific_labels_without_behavior_migration(monkeypatch):
    preview = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    mouse = DashboardHarness(monkeypatch, demo_input_mode="ocr_canvas", **CONFIRM_PARAMS)
    preview_labels = {bid: w["base_text"] for bid, w in preview.node.ocr_button_widgets.items()}
    mouse_labels = {bid: w["base_text"] for bid, w in mouse.node.ocr_button_widgets.items()}
    assert (preview_labels["B"], preview_labels["A"]) == ("B\nRecognize", "A\nReject")
    assert (mouse_labels["B"], mouse_labels["A"]) == ("B\nDraw", "A\nStop")

    node = mouse.node
    modes = []
    mouse.fire(node.ocr_button_widgets["B"]["canvas"], "<Button-1>")
    assert node.ocr_panel_state == "DRAWING"
    modes.append(node.mouse_mode_var.get())

    # Releasing a stroke schedules the automatic recognition request.
    mouse.draw_stroke((100, 100), (150, 150))
    assert mouse.run_jobs(node.ocr_auto_recognize_after_ms) == 1
    assert node.ocr_panel_state == "RECOGNIZING"
    assert len(mouse.published_json(node.config.symbol_capture_topic)[0]["points"]) == 2
    modes.append(node.mouse_mode_var.get())

    mouse.fire(node.ocr_button_widgets["A"]["canvas"], "<Button-1>")
    assert node.ocr_panel_state == "BLOCKED"
    assert mouse.published_json(node.config.confirmed_label_topic) == [
        build_mouse_toolbar_stop_payload(mouse.now)]
    modes.append(node.mouse_mode_var.get())
    assert modes == ["Mode: DRAW", "Mode: IDLE", "Mode: IDLE"]


def test_mouse_confirmed_progress_advances_to_act_without_republishing_or_clearing():
    class ImmediateRoot:
        @staticmethod
        def after(_delay, callback):
            callback()

    class Variable:
        def __init__(self):
            self.value = ""

        def set(self, value):
            self.value = value

    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    candidate = {
        "accepted": True,
        "candidates": [
            {
                "rank": 1,
                "label": "2",
                "confidence": 0.75,
                "task": "HEXAGON_TRAJECTORY",
            }
        ],
    }
    events = []
    node.root = ImmediateRoot()
    node.ocr_panel_state = "CONFIRM_PENDING"
    node.current_candidate_payload = candidate
    node.last_confirmed_label = ""
    node.ocr_hover_button = ""
    node.all_strokes = []
    node.mouse_mode_var = Variable()
    node.mouse_workflow_var = Variable()
    node.mouse_operator_status_var = Variable()
    node.mouse_operator_result_var = Variable()
    node.mouse_operator_mapping_var = Variable()
    node.mouse_candidate_debug_var = Variable()
    node.mouse_operator_candidate_vars = [Variable(), Variable(), Variable()]
    node.ui_state_var = Variable()
    node._cancel_ocr_auto_recognize = lambda: None
    node._cancel_ocr_recognition_timeout = lambda: None
    node._update_ocr_toolbar_visuals = lambda: None
    node._schedule_ui_refresh = lambda: None
    node._publish_dashboard_state = lambda: None
    node._trigger_ocr_recognition_if_ready = lambda: events.append("recognize")

    def confirm_once(rank):
        events.append("confirm:%d" % rank)
        node.last_confirmed_label = "2"

    def clear_candidates(_message):
        events.append("clear")
        node.current_candidate_payload = None
        node.last_confirmed_label = ""

    node._confirm_rank_by_dwell = confirm_once
    node._publish_ocr_stop_label = lambda: events.append("stop")
    node._clear_candidates = clear_candidates
    node._refresh_dwell_status = node._refresh_mouse_operator_presentation

    node._handle_ocr_action("C")

    assert node.last_confirmed_label == "2"
    assert node.current_candidate_payload is candidate
    assert node.mouse_operator_status_var.value == "Confirmed: 2"
    assert node.mouse_workflow_var.value == (
        "✓\u00a0Start\u00a0drawing  →  ✓\u00a0Draw\n"
        "✓\u00a0Auto-recognize  →  ✓\u00a0Review  →  ●\u00a0Act"
    )
    assert events == ["confirm:1"]

    node._handle_ocr_action("C")
    assert events == ["confirm:1"]

    node._handle_ocr_action("A")
    assert node.ocr_panel_state == "BLOCKED"
    assert events == ["confirm:1", "stop"]

    node._handle_ocr_action("X")
    assert node.ocr_panel_state == "WAITING"
    assert events == ["confirm:1", "stop", "clear"]
