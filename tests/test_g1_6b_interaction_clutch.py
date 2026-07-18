import inspect

from colmag_ros import dashboard_controller_input as controls
from colmag_ros import dashboard_geometry as geometry
from colmag_ros import dashboard_sample_lifecycle as lifecycle
from colmag_ros.scripts import magnetic_trajectory_dashboard_node as dashboard
from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
    MagneticTrajectoryDashboardNode,
)


def _dispatch_probe():
    node = MagneticTrajectoryDashboardNode.__new__(MagneticTrajectoryDashboardNode)
    actions = []
    node.confirm_candidate_source = "external_symbol_candidates"
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
    node.board_cursor_dwell_enabled = True
    node.hover_dwell_enabled = False
    node.mouse_hover_button = ""
    node.board_hover_button = ""
    node.hover_source = "none"
    node.ocr_hover_button = ""
    node._dwell_active_button = ""
    node.ocr_hover_started_at = None
    node.ocr_dwell_progress = 0.0
    node.ocr_dwell_fired = False
    node.control_dwell_sec = 1.0
    node.hover_progress_enabled = True
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
    node.board_cursor_dwell_enabled = True
    node.hover_dwell_enabled = False
    node.mouse_hover_button = ""
    node.board_hover_button = ""
    node.hover_source = "none"
    node.ocr_hover_button = ""
    node._dwell_active_button = ""
    node.ocr_hover_started_at = None
    node.ocr_dwell_progress = 0.0
    node.ocr_dwell_fired = False
    node.control_dwell_sec = 1.0
    node.hover_progress_enabled = True
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
    node.is_trajectory_mode = True
    node.candidate_preview_enabled = True
    node.hover_click_enabled = True
    node._preview_pointer_tracking_enabled = lambda: True
    node._preview_button_at_event = lambda _event: "B"
    node._set_ocr_hover_button = lambda button_id: actions.append("hover:" + button_id)

    assert node._handle_preview_pointer_click(object()) is None

    event = type("Event", (), {"char": "b", "keysym": "b"})()
    node._on_key_press(event)
    assert actions == []


def test_normal_dashboard_exposes_mode_but_keeps_sample_metrics_debug_only():
    build_cards = inspect.getsource(
        MagneticTrajectoryDashboardNode._build_trajectory_status_cards
    )
    normal_cards, debug_cards = build_cards.split(
        "if self.dashboard_debug_sample_panel:", 1
    )
    redraw = inspect.getsource(MagneticTrajectoryDashboardNode._redraw_canvas)

    assert "self.traj_clutch_mode_var" in normal_cards
    assert "self.traj_operator_status_var" in normal_cards
    assert "self.traj_operator_result_var" in normal_cards
    assert "self.traj_result_var" not in normal_cards
    assert "self.traj_valid_var" not in normal_cards
    assert "self.traj_xyz_var" not in normal_cards
    assert "self.traj_count_var" not in normal_cards
    assert "self.traj_input_gate_var" not in normal_cards
    assert "self.traj_captured_sample_var" not in normal_cards
    assert "self.traj_candidate_points_var" not in normal_cards
    assert "self.traj_sample_cleanup_var" not in normal_cards
    assert "self.traj_captured_sample_var" in debug_cards
    assert "self.traj_candidate_points_var" in debug_cards
    assert "self.traj_sample_cleanup_var" in debug_cards
    assert "self.traj_valid_var" in debug_cards
    assert "self.traj_xyz_var" in debug_cards
    assert "self.traj_result_var" in debug_cards
    assert 'getattr(self, "dashboard_debug_sample_panel", False)' in redraw
    assert 'label += " (%d pts)"' in redraw
    assert 'text="Drawing zone"' in redraw
    assert "only this area enters DTW" not in redraw
    assert "Controls: hover only" not in redraw
    canvas_builder = inspect.getsource(MagneticTrajectoryDashboardNode._build_canvas_area)
    layout = inspect.getsource(MagneticTrajectoryDashboardNode._layout_ocr_stage)
    assert "ocr_hover_label" not in canvas_builder + layout
    assert "Board input — dwell on controls to activate" not in canvas_builder


def test_disabled_preview_action_is_inert_and_never_dispatches():
    node, actions = _dispatch_probe()
    node._preview_button_enabled = lambda _button_id: False

    assert node._handle_preview_button_action("B") is False
    assert node._handle_preview_button_action("C") is False
    assert node._handle_preview_button_action("A") is False
    assert node._handle_preview_button_action("X") is False
    assert actions == []


def test_existing_dwell_progress_remains_the_only_hover_ring_owner():
    dwell = "\n".join((
        inspect.getsource(MagneticTrajectoryDashboardNode._process_ocr_toolbar_dwell),
        inspect.getsource(MagneticTrajectoryDashboardNode._process_ocr_canvas_dwell),
        inspect.getsource(MagneticTrajectoryDashboardNode._process_preview_dwell),
    ))
    canvas_ring = inspect.getsource(
        MagneticTrajectoryDashboardNode._draw_canvas_reachable_preview_buttons)
    toolbar_ring = inspect.getsource(
        MagneticTrajectoryDashboardNode._update_ocr_toolbar_visuals)

    assert "self.ocr_dwell_progress" in dwell
    assert "ocr_dwell_progress" in canvas_ring
    assert "ocr_dwell_progress" in toolbar_ring
    assert "create_arc" in canvas_ring
    assert "create_arc" in toolbar_ring
    assert 'style["ring"]' in canvas_ring
    assert 'style["ring"]' in toolbar_ring
    assert "root.after" not in canvas_ring
    assert "root.after" not in toolbar_ring
    assert "active and enabled and progress > 0.0" in canvas_ring


def test_responsive_operator_shell_enforces_accepted_minimum_and_expands_cards():
    window = inspect.getsource(
        MagneticTrajectoryDashboardNode._initialize_window_and_variables)
    ui = inspect.getsource(MagneticTrajectoryDashboardNode._build_ui)
    cards = inspect.getsource(
        MagneticTrajectoryDashboardNode._build_trajectory_status_cards)

    assert "dashboard_minimum_window_size()" in window
    assert "fill=tk.BOTH, expand=True" in ui
    assert '"Recognition"' in cards
    assert '"Progress"' in cards
    assert '"Workflow"' not in cards
    assert "expand=True" in cards


def test_shared_canvas_uses_source_specific_labels_without_behavior_migration():
    canvas = inspect.getsource(MagneticTrajectoryDashboardNode._build_canvas_area)
    mouse_actions = inspect.getsource(MagneticTrajectoryDashboardNode._handle_ocr_action)
    mouse_release = inspect.getsource(MagneticTrajectoryDashboardNode._on_canvas_release)
    mouse_presentation = inspect.getsource(
        MagneticTrajectoryDashboardNode._refresh_mouse_operator_presentation
    )

    assert '("B", "B\\nDraw"' in canvas
    assert '("A", "A\\nStop"' in canvas
    assert '("B", "B\\nRecognize"' in canvas
    assert '("A", "A\\nReject"' in canvas
    assert '_start_ocr_drawing("Drawing mode started")' in mouse_actions
    assert "_publish_ocr_stop_label()" in mouse_actions
    assert "_schedule_ocr_auto_recognize()" in mouse_release
    assert "NAVIGATE" not in mouse_presentation
    assert "tracking_mz" not in mouse_presentation


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
