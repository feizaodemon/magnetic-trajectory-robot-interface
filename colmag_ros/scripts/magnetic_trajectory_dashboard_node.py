#!/usr/bin/env python3
import json
import math
import sys
import time
from collections import deque
from pathlib import Path


DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR = "outputs/runtime_samples/real_board"


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros.trajectory_segment_recorder import SegmentRecorder
from colmag_ros.dashboard_confirm_publisher import (
    PreviewConfirmPublisher,
    build_mouse_toolbar_stop_payload,
    should_publish_mouse_toolbar_stop,
)
from colmag_ros.dashboard_candidate_display import (
    PreviewCandidateDisplayState,
    backend_display_name,
    candidate_backend_status_text,
    derive_operator_workflow_stage,
    format_candidate_debug_details,
    format_operator_action_status,
    format_candidate_result_summary,
    format_candidate_rows_for_display,
    format_operator_recognition_view,
    format_operator_workflow,
    format_rank_rows,
    format_recognition_labels,
    preview_candidate_ui_texts,
    recognizer_status_texts,
    summarize_candidates,
)
from colmag_ros import dashboard_points as _dashboard_points
from colmag_ros import dashboard_geometry as _dashboard_geometry
from colmag_ros import dashboard_controller_input as _dashboard_controller_input
from colmag_ros import dashboard_sample_lifecycle as _dashboard_sample_lifecycle
from colmag_ros import dashboard_trajectory_processing as _dashboard_trajectory_processing
from colmag_ros import control_mode
from colmag_ros.dashboard_dwell_status import (
    format_interaction_instruction,
    format_integrated_dwell_status_texts,
    format_ocr_canvas_dwell_texts,
    format_trajectory_preview_hint_text,
    format_trajectory_preview_interaction_text,
    normalize_interaction_profile,
)

CONTROLLER_LAYOUT_RANK_CONFIRM = _dashboard_controller_input.CONTROLLER_LAYOUT_RANK_CONFIRM
CONTROLLER_LAYOUT_GAMEPAD = _dashboard_controller_input.CONTROLLER_LAYOUT_GAMEPAD
BOARD_CONTROL_LAYOUT_EXTERNAL = "external_toolbar"
BOARD_CONTROL_LAYOUT_CANVAS_REACHABLE = "canvas_reachable"

GAMEPAD_MODE_MOTION = _dashboard_controller_input.GAMEPAD_MODE_MOTION
GAMEPAD_MODE_BLOCKED = _dashboard_controller_input.GAMEPAD_MODE_BLOCKED
GAMEPAD_MODE_DIGIT = _dashboard_controller_input.GAMEPAD_MODE_DIGIT
GAMEPAD_MODE_CONFIRM_PENDING = _dashboard_controller_input.GAMEPAD_MODE_CONFIRM_PENDING

GAMEPAD_ARROW_BUTTONS = _dashboard_controller_input.GAMEPAD_ARROW_BUTTONS
GAMEPAD_FACE_BUTTONS = _dashboard_controller_input.GAMEPAD_FACE_BUTTONS
GAMEPAD_BUTTONS = _dashboard_controller_input.GAMEPAD_BUTTONS
GAMEPAD_BUTTON_TO_LABEL = _dashboard_controller_input.GAMEPAD_BUTTON_TO_LABEL
GAMEPAD_BUTTON_TO_INTENT = _dashboard_controller_input.GAMEPAD_BUTTON_TO_INTENT

def safe_json_loads(text):
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data
    except (ValueError, TypeError):
        pass
    return None

def extract_xy_points(payload):
    return _dashboard_points.extract_xy_points(payload)

def extract_single_point(payload):
    return _dashboard_points.extract_single_point(payload)

def map_world_to_canvas(x, y, bounds, width, height, padding):
    x_min, x_max, y_min, y_max = bounds
    if x_max <= x_min or y_max <= y_min:
        return width / 2.0, height / 2.0

    draw_w = width - 2 * padding
    draw_h = height - 2 * padding

    # Map X to canvas X (left to right)
    pct_x = (x - x_min) / (x_max - x_min)
    cx = padding + pct_x * draw_w

    # Map Y to canvas Y (bottom to top, so invert Y)
    pct_y = (y - y_min) / (y_max - y_min)
    cy = height - padding - pct_y * draw_h

    # Clip
    cx = max(padding, min(width - padding, cx))
    cy = max(padding, min(height - padding, cy))

    return cx, cy

def map_canvas_to_world(canvas_x, canvas_y, bounds, width, height, padding):
    x_min, x_max, y_min, y_max = bounds
    if x_max <= x_min or y_max <= y_min:
        return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0

    draw_w = width - 2 * padding
    draw_h = height - 2 * padding
    if draw_w <= 0 or draw_h <= 0:
        return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0

    canvas_x = max(padding, min(width - padding, canvas_x))
    canvas_y = max(padding, min(height - padding, canvas_y))

    pct_x = (canvas_x - padding) / draw_w
    pct_y = (height - padding - canvas_y) / draw_h
    x = x_min + pct_x * (x_max - x_min)
    y = y_min + pct_y * (y_max - y_min)
    return x, y

TRAJECTORY_MODE = "trajectory"

def map_trajectory_point(x, y, bounds, width, height, padding,
                         flip_x=False, flip_y=False, swap_xy=False, clamp=True):
    return _dashboard_geometry.map_trajectory_point(
        x, y, bounds, width, height, padding,
        flip_x=flip_x, flip_y=flip_y, swap_xy=swap_xy, clamp=clamp,
    )

def _robust_min_max(values):
    return _dashboard_trajectory_processing.robust_min_max(values)


def _enforce_min_span(lo, hi, min_span):
    return _dashboard_trajectory_processing.enforce_min_span(lo, hi, min_span)


def normalize_scale_mode(value):
    return _dashboard_trajectory_processing.normalize_scale_mode(value)


def normalize_stroke_gate_mode(value):
    return _dashboard_trajectory_processing.normalize_stroke_gate_mode(value)


def opt_float(value):
    return _dashboard_trajectory_processing.optional_float(value)


def stroke_gate_pass(mode, z, speed, z_min=None, z_max=None,
                     min_speed=0.0, max_speed=0.0):
    return _dashboard_trajectory_processing.stroke_gate_pass(
        mode, z, speed, z_min, z_max, min_speed, max_speed,
    )


def compute_trajectory_bounds(points, fixed_bounds, scale_mode="fixed",
                              auto_center=False, padding_ratio=0.12,
                              min_span=0.01, window_points=None,
                              projection_extent=0.06):
    return _dashboard_trajectory_processing.compute_trajectory_bounds(
        points,
        fixed_bounds,
        scale_mode,
        auto_center,
        padding_ratio,
        min_span,
        window_points,
        projection_extent,
    )

def split_trail_segments(trail):
    return _dashboard_trajectory_processing.split_trail_segments(trail)

def clean_stroke_points(trail):
    return _dashboard_points.clean_stroke_points(trail)

def recognize_trajectory_candidates(points, min_points=8, labels=None):
    """Preview-only top-3 recognition over a live stroke point sequence.

    Reuses the existing in-repo ``recognize_top3`` DTW recognizer (pure Python;
    no ROS / numpy / OCR dependency). Returns ``(status, candidates)`` where
    status is one of ``collecting`` / ``ready`` / ``none`` / ``unavailable`` and
    candidates is a list of ``{rank, label, confidence, distance}``. This never
    publishes anything and never touches a robot / task / confirm path.
    """
    pts = clean_stroke_points(points)
    if len(pts) < int(min_points):
        return "collecting", []
    try:
        from trajectory_symbol_top3_recognizer_node import recognize_top3
    except Exception:  # pragma: no cover - defensive import guard
        return "unavailable", []
    kwargs = {}
    if labels:
        kwargs["labels"] = tuple(labels)
    candidates = recognize_top3(pts, **kwargs)
    if not candidates:
        return "none", []
    return "ready", candidates

def build_ocr_stroke_points(trail):
    return _dashboard_points.build_ocr_stroke_points(trail)

def _short_exc(exc, limit=160):
    """One-line, length-bounded description of an exception for UI/log display."""
    text = "%s: %s" % (type(exc).__name__, exc)
    text = " ".join(text.split())  # collapse newlines/whitespace
    return text[:limit]

RECOGNIZER_BACKENDS = ("lightweight", "dtw", "fake")


def recognize_fake(points, min_points=8, labels=None):
    """Deterministic offline recognizer for tests / no-dependency preview.

    Uses pure trajectory DTW and picks a top-3 from a small
    label set using only the cleaned point count, so results are stable and
    reproducible offline. Returns ``(status, candidates)`` matching
    ``recognize_trajectory_candidates``. Preview-only; publishes nothing.
    """
    pts = clean_stroke_points(points)
    if len(pts) < int(min_points):
        return "collecting", []
    pool = list(labels) if labels else ["1", "2", "3", "A", "C", "X"]
    if not pool:
        return "none", []
    n = len(pts)
    ranked = []
    for rank in range(min(3, len(pool))):
        label = pool[(n + rank) % len(pool)]
        confidence = round(max(0.1, 0.9 - 0.25 * rank), 2)
        ranked.append({
            "rank": rank + 1,
            "label": label,
            "confidence": confidence,
            "distance": float(rank),
        })
    return "ready", ranked

def recognize_stroke(points, backend="dtw", ocr_fallback_dtw=True, min_points=8,
                     canvas_size=256, line_width=8, padding_ratio=0.15, labels=None,
                     status_out=None, recognition_model_path=""):
    """Backend-selecting preview recognition.

    Returns ``(status, candidates, backend_label)``. Production routes use the
    external DTW template-bank node; this preview helper keeps only the local DTW
    implementation and the deterministic fake used by offline tests.
    """
    pts = clean_stroke_points(points)
    backend = str(backend or "dtw").strip().lower()

    if len(pts) < int(min_points):
        return "collecting", [], backend_display_name(backend, recognition_model_path)

    if backend == "fake":
        status, cands = recognize_fake(pts, min_points, labels)
        status = "no_text" if status == "none" else status
        return status, cands, "Fake (offline)"
    status, cands = recognize_trajectory_candidates(pts, min_points, labels)
    status = "no_text" if status == "none" else status
    return status, cands, backend_display_name("dtw", recognition_model_path)

def default_header_title(demo_input_mode):
    return "COLMAG Magnetic Trajectory Dashboard"

def default_header_subtitle(demo_input_mode):
    if demo_input_mode == "ocr_canvas":
        return "Draw, review the recognition result, then confirm or clear"
    if demo_input_mode == TRAJECTORY_MODE:
        return "Draw, recognize, review the result, then confirm or clear"
    return "Magnetic trajectory → Symbol recognition → Human confirmation → Gazebo task"

def default_main_panel_title(demo_input_mode):
    if demo_input_mode == "ocr_canvas":
        return "Mouse Trajectory Input"
    if demo_input_mode == TRAJECTORY_MODE:
        return "Live Magnetic Trajectory"
    return "Symbol Recognition Preview"

def default_safety_badge(demo_input_mode):
    return ""

def default_footer_text(demo_input_mode):
    return ""

def coerce_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default

def clamp(value, low, high):
    return _dashboard_geometry.clamp(value, low, high)

def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)

def is_point_inside_rect(x, y, rect):
    return _dashboard_geometry.is_point_inside_rect(x, y, rect)

def get_virtual_button_under_cursor(canvas_x, canvas_y, button_rects):
    return _dashboard_geometry.get_virtual_button_under_cursor(canvas_x, canvas_y, button_rects)

def resolve_preview_button_hit(x, y, hit_zones):
    return _dashboard_geometry.resolve_preview_button_hit(x, y, hit_zones)

def derive_orientation_clutch_mode(
    interaction_engaged,
    inside_drawing_zone=False,
    hovered_button="",
):
    return _dashboard_geometry.derive_orientation_clutch_mode(
        interaction_engaged,
        inside_drawing_zone,
        hovered_button,
    )

def normalize_board_control_layout(value):
    return _dashboard_geometry.normalize_board_control_layout(
        value,
        external_layout=BOARD_CONTROL_LAYOUT_EXTERNAL,
        canvas_reachable_layout=BOARD_CONTROL_LAYOUT_CANVAS_REACHABLE,
    )

def compute_ocr_stage_geometry(
    available_width, available_height, controls_inside_canvas=False,
):
    return _dashboard_geometry.compute_ocr_stage_geometry(
        available_width,
        available_height,
        controls_inside_canvas=controls_inside_canvas,
    )

def compute_dashboard_panel_widths(total_width):
    return _dashboard_geometry.compute_dashboard_panel_widths(total_width)

def dashboard_minimum_window_size():
    return _dashboard_geometry.dashboard_minimum_window_size()

def build_canvas_reachable_preview_hit_zones(width, height, padding, button_size, reach_rect=None):
    return _dashboard_geometry.build_canvas_reachable_preview_hit_zones(
        width, height, padding, button_size, reach_rect=reach_rect,
    )

def build_canvas_reachable_drawing_zone(width, height, padding, reach_rect=None):
    return _dashboard_geometry.build_canvas_reachable_drawing_zone(
        width, height, padding, reach_rect=reach_rect,
    )

def preview_button_availability(has_sample, has_candidate, has_clearable_content):
    return _dashboard_controller_input.preview_button_availability(
        has_sample, has_candidate, has_clearable_content,
    )

def preview_button_style(button_id, enabled, active=False):
    return _dashboard_geometry.preview_button_style(button_id, enabled, active)

def drawing_zone_style():
    return _dashboard_geometry.drawing_zone_style()

def cancelled_preview_hover_state():
    return _dashboard_controller_input.cancelled_preview_hover_state()

def trajectory_point_in_drawing_zone(
    point, bounds, width, height, padding, drawing_zone,
    flip_x=False, flip_y=False, swap_xy=False,
):
    return _dashboard_geometry.trajectory_point_in_drawing_zone(
        point, bounds, width, height, padding, drawing_zone,
        flip_x=flip_x, flip_y=flip_y, swap_xy=swap_xy,
    )

def trajectory_input_inactive_reason(valid=True, z=None, z_min=None, z_max=None, gate_enabled=True):
    return _dashboard_sample_lifecycle.trajectory_input_inactive_reason(
        valid, z, z_min, z_max, gate_enabled,
    )

def cleanup_drawing_sample_points(points, min_point_delta=0.0, max_points=0, enabled=True):
    return _dashboard_sample_lifecycle.cleanup_drawing_sample_points(
        points, min_point_delta, max_points, enabled,
    )

def sample_cleanup_metadata(raw_points, cleaned_points, min_point_delta=0.0, max_points=0, enabled=True):
    return _dashboard_sample_lifecycle.sample_cleanup_metadata(
        raw_points, cleaned_points, min_point_delta, max_points, enabled,
    )

def real_board_sample_recording_dir(path):
    return _dashboard_sample_lifecycle.real_board_sample_recording_dir(path)

def safe_sample_recording_name(sample_id):
    return _dashboard_sample_lifecycle.safe_sample_recording_name(sample_id)

def build_real_board_sample_record(
    *,
    capture_payload,
    raw_points,
    cleaned_points,
    cleanup_metadata,
    candidate_payload=None,
):
    return _dashboard_sample_lifecycle.build_real_board_sample_record(
        capture_payload=capture_payload,
        raw_points=raw_points,
        cleaned_points=cleaned_points,
        cleanup_metadata=cleanup_metadata,
        candidate_payload=candidate_payload,
    )

def trajectory_input_is_active(valid=True, z=None, z_min=None, z_max=None, gate_enabled=True):
    return _dashboard_sample_lifecycle.trajectory_input_is_active(
        valid, z, z_min, z_max, gate_enabled,
    )

def update_drawing_zone_sample_state(
    state, point, inside_drawing_zone, input_active,
    inside_control_zone=False, append_point=True, force_freeze=False,
):
    return _dashboard_sample_lifecycle.update_drawing_zone_sample_state(
        state,
        point,
        inside_drawing_zone,
        input_active,
        inside_control_zone,
        append_point,
        force_freeze,
    )

def build_dashboard_drawing_zone_capture_payload(
    points, sample_id, sequence_id, timestamp, source_topic, drawing_zone=None,
    raw_points=None, cleanup_metadata=None,
):
    return _dashboard_sample_lifecycle.build_dashboard_drawing_zone_capture_payload(
        points,
        sample_id,
        sequence_id,
        timestamp,
        source_topic,
        drawing_zone,
        raw_points,
        cleanup_metadata,
    )

def canvas_zones_to_stage_zones(canvas_zones, canvas_x, canvas_y):
    return _dashboard_geometry.canvas_zones_to_stage_zones(canvas_zones, canvas_x, canvas_y)

def update_dwell_state(previous_state, current_button_id, now, dwell_sec):
    return _dashboard_controller_input.update_dwell_state(
        previous_state, current_button_id, now, dwell_sec,
    )

def normalize_controller_layout(value):
    return _dashboard_controller_input.normalize_controller_layout(value)

def build_virtual_button_rects(width, height, padding):
    return _dashboard_controller_input.build_virtual_button_rects(
        width, height, padding,
    )

def _rect_from_center(button_id, label, cx, cy, size, kind="rect"):
    return _dashboard_controller_input._rect_from_center(
        button_id, label, cx, cy, size, kind,
    )

def build_gamepad_button_rects(width, height, padding):
    return _dashboard_controller_input.build_gamepad_button_rects(
        width, height, padding,
    )

def build_controller_button_rects(layout, width, height, padding):
    return _dashboard_controller_input.build_controller_button_rects(
        layout, width, height, padding,
    )

def gamepad_button_to_label(button_id):
    return _dashboard_controller_input.gamepad_button_to_label(button_id)

def gamepad_button_to_intent(button_id):
    return _dashboard_controller_input.gamepad_button_to_intent(button_id)

def update_controller_mode(previous_mode, activated_button, now, digit_mode_sec, has_candidates=False):
    return _dashboard_controller_input.update_controller_mode(
        previous_mode, activated_button, now, digit_mode_sec, has_candidates,
    )

def should_ignore_gamepad_button(mode, button_id, block_arrows_on_stop=True):
    return _dashboard_controller_input.should_ignore_gamepad_button(
        mode, button_id, block_arrows_on_stop,
    )

def build_gamepad_confirmed_label_payload(button_id, mode, candidate_payload=None, now_fn=time.time):
    return _dashboard_controller_input.build_gamepad_confirmed_label_payload(
        button_id, mode, candidate_payload, now_fn,
    )

def _extract_candidate_by_rank(payload, selected_rank):
    return _dashboard_controller_input._extract_candidate_by_rank(
        payload, selected_rank,
    )

def candidate_payload_key(payload):
    return _dashboard_controller_input.candidate_payload_key(payload)

def should_suppress_repeated_confirm(last_key, payload, selected_rank):
    return _dashboard_controller_input.should_suppress_repeated_confirm(
        last_key, payload, selected_rank,
    )

def build_confirmed_label_payload(candidate_payload, selected_rank, now_fn=time.time):
    return _dashboard_controller_input.build_confirmed_label_payload(
        candidate_payload, selected_rank, now_fn,
    )

class MagneticTrajectoryDashboardNode:
    def __init__(self):
        self._initialize_runtime_dependencies()
        self._read_route_and_trajectory_parameters()
        self._read_preview_and_recording_parameters()
        self._initialize_dashboard_state()
        self._initialize_window_and_variables()
        self._build_ui()
        self._refresh_dwell_status()
        self._schedule_ui_refresh()
        self._initialize_ros_interfaces()

    def _initialize_runtime_dependencies(self):
        import rospy
        from std_msgs.msg import String
        import tkinter as tk
        from tkinter import ttk

        self.rospy = rospy
        self.String = String
        self.tk = tk
        self.ttk = ttk
        self.preview_confirm_publisher = PreviewConfirmPublisher(
            string_factory=String,
            build_confirmed_label_payload=build_confirmed_label_payload,
            should_suppress_repeated_confirm=should_suppress_repeated_confirm,
            candidate_payload_key=candidate_payload_key,
            logger=rospy,
        )

    def _read_route_and_trajectory_parameters(self):
        rospy = self.rospy
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/colmag/trajectory_2d")
        self.ui_state_topic = rospy.get_param("~ui_state_topic", "/colmag/ui_state")
        self.symbol_capture_topic = rospy.get_param("~symbol_capture_topic", "/colmag/symbol_capture")
        self.symbol_candidates_topic = rospy.get_param("~symbol_candidates_topic", "/colmag/symbol_candidates")
        self.confirmed_label_topic = rospy.get_param("~confirmed_label_topic", "/colmag/confirmed_label")
        self.dashboard_state_topic = rospy.get_param("~dashboard_state_topic", "/colmag/trajectory_dashboard_state")
        self.task_command_topic = rospy.get_param("~task_command_topic", "/colmag/task_command")
        self.demo_task_state_topic = rospy.get_param("~demo_task_state_topic", "/colmag/fr3_demo_task_state")
        self._read_control_mode_parameters()
        self.integrated_confirm_enabled = bool_param(rospy.get_param("~integrated_dashboard_confirm_enabled", False))
        # Explicit publish gates. These were previously declared in the launch
        # files but never read (dead/misleading safety args). They are now honored:
        # the confirmed_label publisher additionally requires publish_confirmed_label,
        # and publish_task_command is read so an operator setting it true is warned
        # rather than silently ignored (the preview dashboard has no task_command
        # publisher by design). Preview-only default keeps both False -> no publish.
        self.publish_confirmed_label = bool_param(rospy.get_param("~publish_confirmed_label", False))
        self.publish_task_command = bool_param(rospy.get_param("~publish_task_command", False))
        # Confirm-candidate source for trajectory-mode dwell confirm.
        #   internal (default): confirm the dashboard's in-process preview
        #     recognizer top candidate (self.trajectory_candidates) -> keeps the
        #     M62-A5/G2 behavior unchanged.
        #   external_symbol_candidates: confirm the external ROS
        #     /colmag/symbol_candidates payload (e.g. dtw_template_bank), falling
        #     back to internal only when no external candidates are available.
        self.confirm_candidate_source = str(
            rospy.get_param("~confirm_candidate_source", "internal"))
        self.virtual_buttons_enabled = bool_param(rospy.get_param("~dashboard_virtual_buttons_enabled", True))
        self.dwell_confirm_sec = float(rospy.get_param("~dashboard_dwell_confirm_sec", 2.0))
        self.dwell_reset_margin_sec = float(rospy.get_param("~dashboard_dwell_reset_margin_sec", 0.3))
        requested_layout = normalize_controller_layout(rospy.get_param("~dashboard_controller_layout", CONTROLLER_LAYOUT_RANK_CONFIRM))
        self.gamepad_enabled = bool_param(rospy.get_param("~dashboard_gamepad_enabled", False))
        self.controller_layout = (
            CONTROLLER_LAYOUT_GAMEPAD
            if requested_layout == CONTROLLER_LAYOUT_GAMEPAD and self.gamepad_enabled
            else CONTROLLER_LAYOUT_RANK_CONFIRM
        )
        self.digit_mode_sec = float(rospy.get_param("~dashboard_digit_mode_sec", 3.0))
        self.gamepad_publish_motion_buttons = bool_param(rospy.get_param("~dashboard_gamepad_publish_motion_buttons", True))
        self.gamepad_block_arrows_on_stop = bool_param(rospy.get_param("~dashboard_gamepad_block_arrows_on_stop", True))
        self.demo_input_mode = rospy.get_param("~demo_input_mode", "trajectory")
        self.is_trajectory_mode = self.demo_input_mode == TRAJECTORY_MODE
        self.interaction_profile = normalize_interaction_profile(
            rospy.get_param("~interaction_profile", None),
            self.demo_input_mode,
        )

        self.subtitle_text = rospy.get_param("~subtitle_text", None)
        self.safety_badge = rospy.get_param("~safety_badge", default_safety_badge(self.demo_input_mode))
        self.footer_text = rospy.get_param("~footer_text", default_footer_text(self.demo_input_mode))
        self.route_scope_text = rospy.get_param(
            "~route_scope_text", "Gazebo only. No real robot command.")
        self.confirm_policy_text = rospy.get_param(
            "~confirm_policy_text", "Confirm publishes selected label.")

        self.canvas_width = rospy.get_param("~trajectory_dashboard_canvas_width", 560)
        self.canvas_height = rospy.get_param("~trajectory_dashboard_canvas_height", 520)
        if self.demo_input_mode == "ocr_canvas":
            self.canvas_width = 360
            self.canvas_height = 300
        self.trail_size = rospy.get_param("~trajectory_dashboard_trail_size", 300)
        REFERENCE_MAGNETIC_EXTENT_M = 0.05
        self.x_min = rospy.get_param("~trajectory_dashboard_x_min", -REFERENCE_MAGNETIC_EXTENT_M)
        self.x_max = rospy.get_param("~trajectory_dashboard_x_max", REFERENCE_MAGNETIC_EXTENT_M)
        self.y_min = rospy.get_param("~trajectory_dashboard_y_min", -REFERENCE_MAGNETIC_EXTENT_M)
        self.y_max = rospy.get_param("~trajectory_dashboard_y_max", REFERENCE_MAGNETIC_EXTENT_M)
        self.padding = 20
        self.bounds = (self.x_min, self.x_max, self.y_min, self.y_max)

        # Real magnetic board preview scaling / orientation (trajectory mode only).
        self.traj_flip_x = bool_param(rospy.get_param("~trajectory_flip_x", False))
        self.traj_flip_y = bool_param(rospy.get_param("~trajectory_flip_y", False))
        self.traj_swap_xy = bool_param(rospy.get_param("~trajectory_swap_xy", False))
        # fixed | fixed_symmetric | auto. fixed_symmetric is the recommended demo
        # mode (stable, no auto breathing); auto stays diagnostic. normalize_scale_mode
        # keeps the deprecated internal alias parseable without exposing it.
        self.traj_scale_mode = normalize_scale_mode(rospy.get_param("~trajectory_scale_mode", "fixed"))
        self.traj_auto_center = bool_param(rospy.get_param("~trajectory_auto_center", False))
        # Drawn-trail length (recent points). 180 keeps a real-board symbol
        # visible long enough for a demo while still bounding stale history.
        self.traj_trace_window_points = int(rospy.get_param("~trajectory_trace_window_points",
                                            rospy.get_param("~trajectory_auto_window_points", 180)))
        if self.traj_trace_window_points < 2:
            self.traj_trace_window_points = 2
        self.traj_auto_padding_ratio = float(rospy.get_param("~trajectory_auto_padding_ratio", 0.12))
        self.traj_auto_min_span = float(rospy.get_param("~trajectory_auto_min_span", 0.01))
        # Fixed symmetric extent for fixed_symmetric mode (+/- this value, both axes).
        self.traj_projection_extent = float(rospy.get_param("~trajectory_projection_extent", 0.06))

        # Preview-only stroke gate (a display mask; never a robot/task/confirm signal).
        # off | z_range | speed | legacy_like. Defaults are pass-through so an
        # untuned gate never hides valid strokes.
        self.stroke_gate_mode = normalize_stroke_gate_mode(rospy.get_param("~trajectory_stroke_gate_mode", "off"))
        self.gate_z_min = opt_float(rospy.get_param("~trajectory_z_min", None))
        self.gate_z_max = opt_float(rospy.get_param("~trajectory_z_max", None))
        self.input_valid_gate_enabled = bool_param(rospy.get_param("~trajectory_input_valid_gate_enabled", True))
        raw_input_z_min = rospy.get_param("~trajectory_input_z_min", "inherit")
        raw_input_z_max = rospy.get_param("~trajectory_input_z_max", "inherit")
        self.input_z_min = self.gate_z_min if str(raw_input_z_min).strip().lower() == "inherit" else opt_float(raw_input_z_min)
        self.input_z_max = self.gate_z_max if str(raw_input_z_max).strip().lower() == "inherit" else opt_float(raw_input_z_max)
        self.input_reactivate_samples = max(1, int(rospy.get_param("~trajectory_input_reactivate_samples", 2)))
        self._input_active_streak = 0
        self.board_sample_cleanup_enabled = bool_param(rospy.get_param("~dashboard_board_sample_cleanup_enabled", True))
        self.board_sample_min_point_delta = max(0.0, float(rospy.get_param("~dashboard_board_sample_min_point_delta", 0.0015)))
        self.board_sample_max_points = max(0, int(rospy.get_param("~dashboard_board_sample_max_points", 160)))
        self.dashboard_debug_sample_panel = bool_param(rospy.get_param("~dashboard_debug_sample_panel", False))
        self.real_board_sample_recording_enabled = bool_param(rospy.get_param("~real_board_sample_recording_enabled", False))
        self.real_board_sample_recording_dir = real_board_sample_recording_dir(
            rospy.get_param("~real_board_sample_recording_dir", DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR)
        )
        self.gate_min_speed = float(rospy.get_param("~trajectory_min_speed", 0.0))
        self.gate_max_speed = float(rospy.get_param("~trajectory_max_speed", 0.0))  # <=0 -> no upper limit
        self.gate_invalid_breaks_stroke = bool_param(rospy.get_param("~trajectory_gate_invalid_breaks_stroke", True))
        self._prev_sample = None  # (x, y, timestamp) for speed gating

    def _read_control_mode_parameters(self):
        self.control_mode_enabled = bool_param(
            self.rospy.get_param("~control_mode_enabled", False)
        )
        self.control_mode_topic = self.rospy.get_param(
            "~control_mode_topic", "/colmag/control_mode"
        )
        self.control_mode_request_topic = self.rospy.get_param(
            "~control_mode_request_topic", "/colmag/control_mode/request"
        )

    def _read_preview_and_recording_parameters(self):
        rospy = self.rospy
        # Preview-only DTW candidate display. It NEVER publishes
        # /colmag/confirmed_label, /colmag/task_command, or any robot/task signal.
        self.ocr_preview_enabled = bool_param(rospy.get_param("~ocr_preview_enabled", False))
        self.candidate_preview_enabled = (
            bool_param(rospy.get_param("~candidate_preview_enabled", False))
            or self.ocr_preview_enabled)
        self.recognition_backend = str(rospy.get_param("~recognition_backend", "dtw")).strip().lower()
        if self.recognition_backend not in RECOGNIZER_BACKENDS:
            self.recognition_backend = "dtw"
        self.recognition_model_path = ""
        self.ocr_fallback_dtw = False
        self.candidate_min_points = int(rospy.get_param(
            "~candidate_min_points", rospy.get_param("~ocr_min_points", 8)))
        self.ocr_canvas_size = int(rospy.get_param("~ocr_canvas_size", 256))
        self.ocr_line_width = int(rospy.get_param("~ocr_line_width", 8))
        self.ocr_padding_ratio = float(rospy.get_param("~ocr_padding_ratio", 0.15))
        labels_param = rospy.get_param("~candidate_labels", None)
        if isinstance(labels_param, str):
            labels_param = [t for t in labels_param.replace(",", " ").split() if t]
        self.candidate_labels = list(labels_param) if labels_param else None
        self.auto_recognize = (
            bool_param(rospy.get_param("~candidate_auto_recognize", False))
            or bool_param(rospy.get_param("~ocr_auto_recognize", False)))
        self.ocr_debounce_sec = float(rospy.get_param(
            "~ocr_debounce_sec", rospy.get_param("~candidate_auto_idle_sec", 0.8)))
        # Four preview-only hover/click buttons (M62-A5).
        self.hover_buttons_enabled = bool_param(rospy.get_param("~hover_buttons_enabled", True))
        self.hover_click_enabled = bool_param(rospy.get_param("~hover_click_enabled", True))
        self.hover_dwell_enabled = bool_param(rospy.get_param("~hover_dwell_enabled", False))
        self.control_dwell_sec = float(rospy.get_param(
            "~dashboard_control_dwell_sec",
            rospy.get_param("~hover_dwell_sec", 1.0),
        ))
        self.hover_progress_enabled = bool_param(rospy.get_param("~hover_progress_enabled", True))
        # M62-A5C: the live board's red current point can drive button dwell without
        # a mouse. Primary demo interaction; mouse hover/click stays as fallback.
        self.board_cursor_dwell_enabled = bool_param(rospy.get_param("~board_cursor_dwell_enabled", True))
        self.board_control_layout = normalize_board_control_layout(
            rospy.get_param("~dashboard_board_control_layout", BOARD_CONTROL_LAYOUT_EXTERNAL)
        )
        self.candidate_status = "idle"
        self.recognition_backend_label = backend_display_name(
            self.recognition_backend,
            self.recognition_model_path,
        )
        self.trajectory_candidates = []
        self._candidate_recognize_job = None
        self._recognizing = False
        self.preview_interaction_state = "idle"
        self.preview_confirmed_label = ""

        # M63-B preview-only segment recording. When enabled, Start records ONE
        # complete trajectory segment (time window, or z-lift when available) and
        # recognition runs on that completed segment instead of the live stream.
        # Default OFF so existing continuous live-preview behavior is unchanged.
        self.trajectory_recording_enabled = bool_param(rospy.get_param("~trajectory_recording_enabled", False))
        self.record_duration_sec = float(rospy.get_param("~record_duration_sec", 2.5))
        # z-lift stop is OFF by default: the real board's z reliability is not yet
        # live-validated. Time-window recording is the primary, always-available
        # stop. Enable record_stop_on_lift only after z is validated on hardware.
        self.record_stop_on_lift = bool_param(rospy.get_param("~record_stop_on_lift", False))
        self.record_lift_z_threshold = float(rospy.get_param("~record_lift_z_threshold", 0.05))
        self.record_min_points = int(rospy.get_param("~record_min_points", 8))
        self.record_max_points = int(rospy.get_param("~record_max_points", 600))
        self.recognize_only_completed_segment = bool_param(
            rospy.get_param("~recognize_only_completed_segment", True))
        self.segment_recorder = SegmentRecorder(
            duration_sec=self.record_duration_sec,
            stop_on_lift=self.record_stop_on_lift,
            lift_z_threshold=self.record_lift_z_threshold,
            min_points=self.record_min_points,
            max_points=self.record_max_points,
        )
        self.recorded_segment = []
        # M63-B preview-only semantic library (display only; no dispatch).
        self.semantic_preview_enabled = bool_param(rospy.get_param("~semantic_preview_enabled", True))

    def _initialize_dashboard_state(self):
        rospy = self.rospy
        if self.is_trajectory_mode:
            self.x_min = float(rospy.get_param("~trajectory_x_min", -0.07))
            self.x_max = float(rospy.get_param("~trajectory_x_max", 0.07))
            self.y_min = float(rospy.get_param("~trajectory_y_min", -0.07))
            self.y_max = float(rospy.get_param("~trajectory_y_max", 0.09))
            self.bounds = (self.x_min, self.x_max, self.y_min, self.y_max)
        self._active_bounds = self.bounds
        self.latest_point = None
        self.latest_valid = True
        self.latest_writing = True
        self.latest_input_active = True
        self.latest_input_inactive_reason = ""
        self.latest_inside_drawing_zone = False
        self.latest_inside_control_zone = False
        self.trajectory_sample_count = 0
        self.latest_sample_index = None
        self.current_control_mode = control_mode.TASK
        self.control_mode_observed = False

        trail_maxlen = self.traj_trace_window_points if self.is_trajectory_mode else self.trail_size
        self.trail = deque(maxlen=trail_maxlen)
        self.drawing_zone_trail = deque(maxlen=trail_maxlen)
        self.drawing_sample_state = {
            "active_points": [],
            "frozen_points": [],
            "was_inside": False,
            "phase": "IDLE",
        }
        self.active_drawing_buffer = []
        self.frozen_drawing_sample = []
        self.published_sample_points = []
        self.published_sample_id = ""
        self.published_sample_point_count = 0
        self.published_sample_raw_point_count = 0
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self.current_candidate_payload = None
        self.last_sample_record_path = None
        self.last_sample_record_data = None
        # Latest symbol_capture ids, used as the stale guard reference when
        # confirming external /colmag/symbol_candidates.
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.virtual_button_rects = build_controller_button_rects(
            self.controller_layout,
            self.canvas_width,
            self.canvas_height,
            self.padding,
        )
        if self.demo_input_mode == "ocr_canvas":
            self._ocr_draw_area = (10, 24, self.canvas_width - 10, self.canvas_height - 10)
        else:
            self._ocr_draw_area = None

        self.dwell_state = update_dwell_state({}, "", time.time(), self.dwell_confirm_sec)
        self.ocr_button_widgets = {}
        self.preview_button_hit_zones = []
        self.preview_canvas_button_zones = []
        self.preview_canvas_drawing_zone = None
        # ``ocr_hover_button`` is the EFFECTIVE hovered button that drives visuals,
        # status and dwell. In ocr_canvas mode it is the mouse hover directly. In
        # trajectory preview mode it is recomputed each dwell tick as board-cursor
        # first, then mouse fallback (``mouse_hover_button``). ``hover_source`` names
        # which one is active.
        self.ocr_hover_button = ""
        self.mouse_hover_button = ""
        self.board_hover_button = ""
        self.hover_source = "none"
        self._dwell_active_button = ""
        self.ocr_hover_started_at = None
        self.ocr_dwell_progress = 0.0
        self.ocr_dwell_fired = False
        self.controller_mode = GAMEPAD_MODE_MOTION
        self.digit_mode_expires_at = None
        self.last_confirm_key = ""
        self.last_confirmed_label = ""
        self.last_selected_rank = None
        self.last_command_intent = ""
        self.ocr_panel_state = "WAITING"
        self.is_drawing = False
        self.drawing_stroke = []
        self.all_strokes = []
        self.ocr_auto_recognize_after_ms = 700
        self.ocr_auto_recognize_job = None
        self.ocr_recognition_timeout_ms = int(rospy.get_param("~ocr_recognition_timeout_ms", 12000))
        self.ocr_recognition_timeout_job = None
        self.ocr_last_stroke_time = None

    def _initialize_window_and_variables(self):
        tk = self.tk
        self.root = tk.Tk()
        self.root.title("COLMAG Magnetic Trajectory Dashboard")
        self.root.geometry("1060x760")
        min_width, min_height = dashboard_minimum_window_size()
        self.root.minsize(min_width, min_height)
        self.root.configure(bg="#F0F0F0")
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.ui_state_var = tk.StringVar(value="WAITING")
        self.capture_var = tk.StringVar(value="-")
        self.control_mode_var = tk.StringVar(value="Mode: TASK")
        self.control_mode_button_var = tk.StringVar(value="Switch to TELEOP")

        self.backend_status_var = tk.StringVar(value="Backend: unknown")
        self.recognizer_mode_var = tk.StringVar(value="Mode: unknown")
        self.recognizer_model_var = tk.StringVar(value="Model: n/a")
        self.recognizer_status_var = tk.StringVar(value="Status: unknown")
        self.recognizer_labels_var = tk.StringVar(value="Labels: unknown")
        self.rank_vars = [tk.StringVar(value="-") for _ in range(3)]
        self.dwell_var = tk.StringVar(value="Integrated dwell confirm: disabled")
        self.controller_mode_var = tk.StringVar(value="Controller: rank_confirm")
        self.hover_var = tk.StringVar(value="Hover: -")
        if self.is_trajectory_mode and self.candidate_preview_enabled:
            initial_hint = format_interaction_instruction(self.interaction_profile)
        elif self.is_trajectory_mode:
            initial_hint = "Live preview — not affine-calibrated"
        else:
            initial_hint = "WAITING | Hover B to start drawing"
        self.ocr_hint_var = tk.StringVar(value=initial_hint)
        self.confirm_var = tk.StringVar(value="-")
        self.task_var = tk.StringVar(value="-")
        self.gazebo_var = tk.StringVar(value="-")

        # Trajectory-preview status (trajectory mode only).
        if self.traj_scale_mode == "auto":
            _scale_label = "auto-fit recent (diagnostic)"
        elif self.traj_scale_mode == "fixed_symmetric":
            _scale_label = "fixed symmetric, extent %.3g" % self.traj_projection_extent
        else:
            _scale_label = "fixed-range window"
        self.traj_mode_var = tk.StringVar(value="Scale: %s, uniform" % _scale_label)
        self.traj_window_var = tk.StringVar(value="Trace window: %d points" % self.traj_trace_window_points)
        self.traj_gate_var = tk.StringVar(value="Stroke gate: %s" % self.stroke_gate_mode)
        self.traj_topic_var = tk.StringVar(value="Topic: %s" % self.trajectory_topic)
        self.traj_valid_var = tk.StringVar(value="Valid: -")
        self.traj_xyz_var = tk.StringVar(value="x/y/z: - / - / -")
        self.traj_count_var = tk.StringVar(value="Samples: 0")
        self.traj_filter_var = tk.StringVar(value="Filter: -")
        self.traj_input_gate_var = tk.StringVar(
            value="Input: active - | drawing - | control - | reason -")
        self.traj_clutch_mode_var = tk.StringVar(value="Mode: NAVIGATE")
        self.traj_workflow_var = tk.StringVar(
            value=format_operator_workflow("draw", self.interaction_profile))
        self.traj_operator_status_var = tk.StringVar(
            value="Draw a character in the center")

        # Candidate preview (M62-A4/A5, display only).
        self.traj_preview_mode_var = tk.StringVar(value="Mode: Real magnetic board trajectory recognition")
        self.traj_backend_var = tk.StringVar(value="Backend: %s" % self.recognition_backend_label)
        self.traj_recognizer_detail_var = tk.StringVar(value="External recognizer: waiting for /colmag/symbol_candidates")
        self.traj_candidate_status_var = tk.StringVar(value="Candidate status: idle")
        self.traj_candidate_points_var = tk.StringVar(value="Points collected: 0")
        self.traj_sample_state_var = tk.StringVar(
            value="Sample: IDLE | Live 0 | Raw 0 | Frozen raw 0 | Published 0")
        self.traj_sample_cleanup_var = tk.StringVar(
            value="Published sample: raw 0 -> clean 0 | controls excluded")
        self.traj_captured_sample_var = tk.StringVar(value="Captured sample: none")
        self.traj_result_var = tk.StringVar(value="Result: waiting for candidates")
        self.traj_operator_result_var = tk.StringVar(
            value="No candidates yet")
        self.traj_operator_mapping_var = tk.StringVar(value="")
        self.traj_candidate_debug_var = tk.StringVar(
            value="DTW/template details: unavailable")
        self.traj_recognition_progress_var = tk.StringVar(value="Progress: idle")
        self.traj_interaction_var = tk.StringVar(value="Interaction: idle | Source: none | Active: - | Dwell: 0%")
        self.traj_confirm_var = tk.StringVar(value="Confirm: disabled in preview")
        self.traj_recording_var = tk.StringVar(value="Recording: %s" % (
            "press S/Start to record a segment" if self.trajectory_recording_enabled else "disabled"))
        self.traj_semantic_var = tk.StringVar(value="Meaning: -")
        self.traj_candidate_vars = [tk.StringVar(value="Candidate %d: -" % (i + 1)) for i in range(3)]
        self.traj_operator_candidate_vars = [
            tk.StringVar(value="")
            for i in range(3)
        ]
        self.mouse_mode_var = tk.StringVar(value="Mode: IDLE")
        self.mouse_workflow_var = tk.StringVar(
            value=format_operator_workflow("start", "mouse"))
        self.mouse_operator_status_var = tk.StringVar(value="Hover B to start drawing")
        self.mouse_operator_result_var = tk.StringVar(value="No candidates yet")
        self.mouse_operator_mapping_var = tk.StringVar(value="")
        self.mouse_candidate_debug_var = tk.StringVar(
            value="DTW/template details: unavailable")
        self.mouse_operator_candidate_vars = [
            tk.StringVar(value="")
            for i in range(3)
        ]

    def _initialize_ros_interfaces(self):
        rospy = self.rospy
        String = self.String
        rospy.Subscriber(self.trajectory_topic, String, self._handle_trajectory, queue_size=10)
        rospy.Subscriber(self.ui_state_topic, String, self._handle_ui_state, queue_size=10)
        rospy.Subscriber(self.symbol_capture_topic, String, self._handle_capture, queue_size=10)
        rospy.Subscriber(self.symbol_candidates_topic, String, self._handle_candidates, queue_size=10)
        rospy.Subscriber(self.confirmed_label_topic, String, self._handle_confirmed, queue_size=10)
        rospy.Subscriber(self.task_command_topic, String, self._handle_task, queue_size=10)
        rospy.Subscriber(self.demo_task_state_topic, String, self._handle_gazebo, queue_size=10)
        self.control_mode_request_pub = None
        self.control_mode_subscriber = None
        if self.control_mode_enabled:
            self.control_mode_request_pub = rospy.Publisher(
                self.control_mode_request_topic, String, queue_size=1, latch=False
            )
            self.control_mode_subscriber = rospy.Subscriber(
                self.control_mode_topic,
                String,
                self._handle_control_mode,
                queue_size=1,
            )
        self.confirm_pub = None
        self.dashboard_state_pub = None
        self.symbol_capture_pub = rospy.Publisher(self.symbol_capture_topic, String, queue_size=10, latch=True)
        # Confirmed-label publishing needs BOTH the integrated confirm gate AND
        # the explicit publish_confirmed_label gate. Preview launches keep both
        # false, so no confirmed_label publisher is ever created in preview mode.
        if self.integrated_confirm_enabled:
            if self.publish_confirmed_label:
                self.confirm_pub = rospy.Publisher(self.confirmed_label_topic, String, queue_size=10, latch=True)
                self.dashboard_state_pub = rospy.Publisher(self.dashboard_state_topic, String, queue_size=10, latch=True)
            else:
                rospy.logwarn("integrated confirm enabled but publish_confirmed_label=false "
                              "-> confirmed_label publishing stays disabled (preview only)")
        if self.publish_task_command:
            # No task_command publisher exists in this preview dashboard by design.
            rospy.logwarn("publish_task_command=true has no effect: the preview "
                          "dashboard never publishes /colmag/task_command")

        rospy.loginfo("MagneticTrajectoryDashboardNode started")
        rospy.loginfo("integrated_dashboard_confirm_enabled=%s publish_confirmed_label=%s publish_task_command=%s",
                      self.integrated_confirm_enabled, self.publish_confirmed_label, self.publish_task_command)
        rospy.loginfo("dashboard_controller_layout=%s", self.controller_layout)
        rospy.loginfo("dashboard_board_control_layout=%s", self.board_control_layout)

    def _build_ui(self):
        tk = self.tk
        ttk = self.ttk

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure("Header.TFrame", background="#2C3E50")
        style.configure("HeaderTitle.TLabel", background="#2C3E50", foreground="#ECF0F1", font=("TkDefaultFont", 16, "bold"))
        style.configure("HeaderSub.TLabel", background="#2C3E50", foreground="#BDC3C7", font=("TkDefaultFont", 10))
        style.configure("Badge.TLabel", background="#F39C12", foreground="#FFFFFF", font=("TkDefaultFont", 10, "bold"), padding=4)
        style.configure("Card.TFrame", background="#FFFFFF", relief="flat")
        style.configure("CardTitle.TLabel", background="#FFFFFF", foreground="#7F8C8D", font=("TkDefaultFont", 10, "bold"))

        main_frame = tk.Frame(self.root, bg="#F0F0F0")
        main_frame.pack(fill=tk.BOTH, expand=True)

        self._build_header(main_frame)

        content_frame = tk.Frame(main_frame, bg="#F0F0F0")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        self.dashboard_content_frame = content_frame

        left_frame = tk.Frame(content_frame, bg="#F0F0F0")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.dashboard_left_frame = left_frame

        self._build_canvas_area(left_frame)

        right_frame = tk.Frame(content_frame, bg="#F0F0F0", width=380)
        right_frame.pack_propagate(False)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(15, 0))
        self.dashboard_right_frame = right_frame
        self._build_status_cards(right_frame)
        content_frame.bind("<Configure>", self._layout_dashboard_columns)
        right_frame.bind("<Configure>", self._layout_dashboard_status_wrap)

        self._build_safety_footer(main_frame)

    def _layout_dashboard_columns(self, event=None):
        total_width = getattr(event, "width", self.dashboard_content_frame.winfo_width())
        _left_width, right_width = compute_dashboard_panel_widths(total_width)
        self.dashboard_right_frame.configure(width=right_width)

    def _layout_dashboard_status_wrap(self, event=None):
        right_width = getattr(event, "width", self.dashboard_right_frame.winfo_width())
        for label, base_wraplength in getattr(self, "dashboard_wrapped_labels", []):
            label.configure(wraplength=max(base_wraplength, int(right_width) - 40))

    def _build_header(self, parent):
        tk = self.tk
        ttk = self.ttk
        header = ttk.Frame(parent, style="Header.TFrame")
        header.pack(fill=tk.X)

        left_frame = ttk.Frame(header, style="Header.TFrame")
        left_frame.pack(side=tk.LEFT, padx=15, pady=10)

        title = default_header_title(self.demo_input_mode)
        if self.subtitle_text:
            sub = self.subtitle_text
        else:
            sub = default_header_subtitle(self.demo_input_mode)
        ttk.Label(left_frame, text=title, style="HeaderTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(left_frame, text=sub, style="HeaderSub.TLabel").pack(anchor=tk.W)

        right_frame = ttk.Frame(header, style="Header.TFrame")
        right_frame.pack(side=tk.RIGHT, padx=15, pady=10)
        if self.control_mode_enabled:
            mode_frame = ttk.Frame(right_frame, style="Header.TFrame")
            mode_frame.pack(side=tk.LEFT, padx=(0, 10))
            ttk.Label(
                mode_frame,
                textvariable=self.control_mode_var,
                style="HeaderSub.TLabel",
            ).pack(anchor=tk.E)
            self.control_mode_button = ttk.Button(
                mode_frame,
                textvariable=self.control_mode_button_var,
                command=self._request_control_mode_switch,
            )
            self.control_mode_button.pack(anchor=tk.E, pady=(3, 0))
        if self.safety_badge:
            ttk.Label(
                right_frame, text=self.safety_badge, style="Badge.TLabel"
            ).pack(anchor=tk.E)

    def _build_canvas_area(self, parent):
        tk = self.tk
        stage_host = tk.Frame(parent, bg="#F0F0F0")
        stage_host.pack(fill=tk.BOTH, expand=True, pady=10, padx=10)
        self.ocr_gamepad_frame = stage_host
        stage_host.bind("<Configure>", self._layout_ocr_stage)

        stage = tk.Frame(stage_host, bg="#EEF2F5", bd=1, relief=tk.SOLID, highlightthickness=1, highlightbackground="#CBD5E1")
        self.ocr_stage_card = stage

        title_text = default_main_panel_title(self.demo_input_mode)
        self.ocr_title_label = tk.Label(stage, text=title_text, bg="#EEF2F5", fg="#1F2937", font=("TkDefaultFont", 14, "bold"))
        self.ocr_hint_label = tk.Label(stage, textvariable=self.ocr_hint_var, bg="#EEF2F5", fg="#64748B", font=("TkDefaultFont", 10, "bold"))

        self.canvas = tk.Canvas(stage, bg="white", highlightthickness=1, highlightbackground="#CBD5E1")

        if self.demo_input_mode == "ocr_canvas":
            buttons = [
                ("X", "X\nClear", lambda e, bid="X": self._handle_ocr_action(bid)),
                ("A", "A\nStop", lambda e, bid="A": self._handle_ocr_action(bid)),
                ("C", "C\nConfirm", lambda e, bid="C": self._handle_ocr_action(bid)),
                ("B", "B\nDraw", lambda e, bid="B": self._handle_ocr_action(bid)),
            ]
        elif self.is_trajectory_mode and self.candidate_preview_enabled and self.hover_buttons_enabled:
            buttons = [
                ("X", "X\nClear", lambda e, bid="X": self._handle_preview_button_action(bid)),
                ("A", "A\nReject", lambda e, bid="A": self._handle_preview_button_action(bid)),
                ("C", "C\nConfirm", lambda e, bid="C": self._handle_preview_button_action(bid)),
                ("B", "B\nRecognize", lambda e, bid="B": self._handle_preview_button_action(bid)),
            ]
        elif self.is_trajectory_mode:
            buttons = []
        else:
            buttons = [
                ("X", "A\nRecog", lambda e: self._on_recognize_button()),
                ("A", "X\nReject", lambda e: self._on_reject_button()),
                ("C", "B\nConfirm", lambda e: self._on_confirm_publish_button()),
                ("B", "C\nResamp", lambda e: self._on_resample_button()),
                ("CLEAR", "CLEAR", lambda e: self._on_clear_button()),
            ]

        for button_id, label_text, handler in buttons:
            widget = tk.Canvas(stage, bg="#EEF2F5", highlightthickness=0, cursor="hand2")
            preview_button = (
                self.is_trajectory_mode and self.candidate_preview_enabled
                and self.hover_buttons_enabled
            )
            if self.demo_input_mode == "ocr_canvas" or (
                preview_button and self.hover_dwell_enabled
            ):
                widget.bind("<Enter>", lambda _event, bid=button_id: self._set_ocr_hover_button(bid))
                widget.bind("<Leave>", lambda _event, bid=button_id: self._clear_ocr_hover_button(bid))
                if preview_button:
                    widget.bind("<Motion>", self._handle_preview_pointer_motion)
            else:
                def on_enter(e, cv=widget): cv.config(cursor="hand2")
                def on_leave(e, cv=widget): cv.config(cursor="")
                widget.bind("<Enter>", on_enter)
                widget.bind("<Leave>", on_leave)

            if preview_button:
                widget.bind("<Button-1>", self._handle_preview_pointer_click)
                widget.tag_bind("all", "<Button-1>", self._handle_preview_pointer_click)
            elif self.hover_click_enabled or self.demo_input_mode != TRAJECTORY_MODE:
                widget.bind("<Button-1>", handler)
                # Support text clicks as well if they miss the circle slightly
                widget.tag_bind("all", "<Button-1>", handler)

            self.ocr_button_widgets[button_id] = {
                "canvas": widget,
                "base_text": label_text,
                "size": 82,
            }
        self._update_ocr_toolbar_visuals()

        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.root.bind("<Key>", self._on_key_press)
        if self.is_trajectory_mode and self.candidate_preview_enabled and self.hover_buttons_enabled:
            for widget in (stage, stage_host, self.canvas):
                widget.bind("<Motion>", self._handle_preview_pointer_motion, add="+")
                widget.bind("<Button-1>", self._handle_preview_pointer_click, add="+")
            self.root.bind("<Motion>", self._handle_preview_pointer_motion, add="+")
            self.root.bind("<Leave>", self._handle_preview_pointer_leave, add="+")
        if self.demo_input_mode == "ocr_canvas" or self._preview_dwell_active():
            self.root.after(50, self._process_ocr_toolbar_dwell)

    def _use_canvas_reachable_preview_controls(self):
        return (
            getattr(self, "is_trajectory_mode", False)
            and getattr(self, "candidate_preview_enabled", False)
            and getattr(self, "hover_buttons_enabled", False)
            and getattr(self, "board_control_layout", "") == BOARD_CONTROL_LAYOUT_CANVAS_REACHABLE
        )

    def _reachable_trajectory_canvas_rect(self):
        bounds = getattr(self, "_active_bounds", self.bounds)
        return self._trajectory_canvas_rect_for_bounds(bounds)

    def _trajectory_canvas_rect_for_bounds(self, bounds):
        x_min, x_max, y_min, y_max = bounds
        corners = [
            map_trajectory_point(
                x, y, bounds, self.canvas_width, self.canvas_height, self.padding,
                flip_x=self.traj_flip_x, flip_y=self.traj_flip_y,
                swap_xy=self.traj_swap_xy,
            )
            for x in (x_min, x_max)
            for y in (y_min, y_max)
        ]
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        return (min(xs), min(ys), max(xs), max(ys))

    def _drawing_zone_canvas_rect(self):
        return build_canvas_reachable_drawing_zone(
            self.canvas_width,
            self.canvas_height,
            self.padding,
            reach_rect=self._trajectory_canvas_rect_for_bounds(self.bounds),
        )

    def _trajectory_point_in_drawing_zone(self, pt):
        return trajectory_point_in_drawing_zone(
            pt,
            self.bounds,
            self.canvas_width,
            self.canvas_height,
            self.padding,
            self._drawing_zone_canvas_rect(),
            flip_x=self.traj_flip_x,
            flip_y=self.traj_flip_y,
            swap_xy=self.traj_swap_xy,
        )

    def _trajectory_point_in_control_zone(self, pt):
        if not self._use_canvas_reachable_preview_controls():
            return False
        if pt is None:
            return False
        zones = getattr(self, "preview_canvas_button_zones", []) or []
        if not zones:
            return False
        cx, cy = self._canvas_xy(pt[0], pt[1])
        return bool(resolve_preview_button_hit(cx, cy, zones))

    def _reset_drawing_sample_state(self):
        self.drawing_sample_state = {
            "active_points": [],
            "frozen_points": [],
            "was_inside": False,
            "phase": "IDLE",
        }
        self.active_drawing_buffer = []
        self.frozen_drawing_sample = []
        if hasattr(self, "drawing_zone_trail"):
            self.drawing_zone_trail.clear()

    def _clear_published_sample_state(self):
        self.published_sample_points = []
        self.published_sample_id = ""
        self.published_sample_point_count = 0
        self.published_sample_raw_point_count = 0

    def _clear_sample_record_reference(self):
        self.last_sample_record_path = None
        self.last_sample_record_data = None

    def _write_real_board_sample_record(self, record):
        if not getattr(self, "real_board_sample_recording_enabled", False):
            return None
        try:
            directory = real_board_sample_recording_dir(getattr(self, "real_board_sample_recording_dir", ""))
            directory.mkdir(parents=True, exist_ok=True)
            sample_id = safe_sample_recording_name(record.get("sample_id", "unknown"))
            output_path = directory / ("%s.json" % sample_id)
            output_path.write_text(json.dumps(record, sort_keys=True, indent=2) + "\n")
            self.last_sample_record_path = output_path
            self.last_sample_record_data = record
            return output_path
        except Exception as exc:
            self.rospy.logwarn("real-board sample recording failed: %s", _short_exc(exc))
            return None

    def _record_real_board_sample(self, capture_payload, raw_points, cleaned_points, metadata):
        record = build_real_board_sample_record(
            capture_payload=capture_payload,
            raw_points=raw_points,
            cleaned_points=cleaned_points,
            cleanup_metadata=metadata,
            candidate_payload=getattr(self, "current_candidate_payload", None),
        )
        return self._write_real_board_sample_record(record)

    def _update_real_board_sample_record_candidate(self, candidate_payload):
        if not getattr(self, "real_board_sample_recording_enabled", False):
            return
        record = getattr(self, "last_sample_record_data", None)
        path = getattr(self, "last_sample_record_path", None)
        if not record or path is None:
            return
        sample_id = record.get("sample_id")
        candidate_sample_id = candidate_payload.get("sample_id") if isinstance(candidate_payload, dict) else None
        if sample_id and candidate_sample_id and str(sample_id) != str(candidate_sample_id):
            return
        record["latest_candidate_payload"] = candidate_payload
        record["latest_candidate_may_be_stale"] = False
        self._write_real_board_sample_record(record)

    def _sync_drawing_sample_buffers(self):
        self.active_drawing_buffer = list(self.drawing_sample_state.get("active_points") or [])
        self.frozen_drawing_sample = list(self.drawing_sample_state.get("frozen_points") or [])

    def _sample_lifecycle_phase(self):
        if getattr(self, "captured_sample_point_count", 0):
            return "RECOGNIZED"
        phase = str((getattr(self, "drawing_sample_state", {}) or {}).get("phase") or "IDLE")
        if phase == "PEN_UP_PENDING" and not getattr(self, "active_drawing_buffer", []):
            return "IDLE"
        return phase

    def _freeze_current_drawing_sample(self):
        if self.frozen_drawing_sample:
            return
        if not self.active_drawing_buffer:
            return
        self.drawing_sample_state = update_drawing_zone_sample_state(
            self.drawing_sample_state,
            None,
            inside_drawing_zone=False,
            input_active=True,
            inside_control_zone=True,
            append_point=False,
            force_freeze=True,
        )
        self._sync_drawing_sample_buffers()

    def _start_new_drawing_after_recognized_if_needed(self, inside_drawing_zone, writing):
        if not (inside_drawing_zone and writing):
            return
        if not getattr(self, "captured_sample_point_count", 0):
            return
        if not getattr(self, "frozen_drawing_sample", []):
            return
        self._reset_drawing_sample_state()
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self._clear_published_sample_state()
        self._clear_sample_record_reference()
        self.current_candidate_payload = None
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.trajectory_candidates = []
        self.candidate_status = "idle"
        self.preview_interaction_state = "new_drawing_started"

    def _layout_ocr_stage(self, event=None):
        if not hasattr(self, "ocr_stage_card"):
            return

        available_w = self.ocr_gamepad_frame.winfo_width()
        available_h = self.ocr_gamepad_frame.winfo_height()
        use_canvas_controls = self._use_canvas_reachable_preview_controls()
        geometry = compute_ocr_stage_geometry(
            available_w, available_h,
            controls_inside_canvas=use_canvas_controls,
        )
        if geometry is None:
            return

        stage_x, stage_y, stage_w, stage_h = geometry["stage"]
        draw_x, draw_y, draw_w, draw_h = geometry["draw"]
        pad = geometry["padding"]
        button_size = geometry["button_size"]
        self.ocr_stage_card.place(x=stage_x, y=stage_y, width=stage_w, height=stage_h)

        self.canvas_width = int(draw_w)
        self.canvas_height = int(draw_h)
        self.canvas.config(width=self.canvas_width, height=self.canvas_height)
        self.canvas.place(x=int(draw_x), y=int(draw_y), width=self.canvas_width, height=self.canvas_height)
        self._ocr_draw_area = (10, 24, self.canvas_width - 10, self.canvas_height - 10)

        rospy.loginfo_once("M48 geometry: window=%dx%d left_panel=%dx%d draw_canvas=%dx%d draw_w=%d draw_h=%d",
                           available_w, available_h, stage_w, stage_h,
                           self.canvas_width, self.canvas_height, draw_w, draw_h)

        self.ocr_title_label.place(x=pad, y=14)
        self.ocr_hint_label.configure(wraplength=max(180, stage_w - 2 * pad))
        self.ocr_hint_label.place(x=pad, y=38)
        hit_zones = []
        canvas_control_zones = []
        for button_id, widgets in self.ocr_button_widgets.items():
            widgets["size"] = button_size
            button = widgets["canvas"]
            button.configure(width=button_size, height=button_size)
            if use_canvas_controls and button_id in ("X", "A", "B", "C"):
                button.place_forget()
                continue
            bx, by = geometry["button_rects"][button_id]
            button.place(x=int(bx), y=int(by), width=button_size, height=button_size)
            if self.is_trajectory_mode and self.candidate_preview_enabled and button_id in ("X", "A", "B", "C"):
                hit_zones.append({
                    "id": button_id,
                    "label": widgets["base_text"],
                    "x1": float(bx),
                    "y1": float(by),
                    "x2": float(bx + button_size),
                    "y2": float(by + button_size),
                    "cx": float(bx + button_size / 2.0),
                    "cy": float(by + button_size / 2.0),
                    "radius": float(button_size / 2.0),
                })
        if use_canvas_controls:
            canvas_control_zones = build_canvas_reachable_preview_hit_zones(
                self.canvas_width,
                self.canvas_height,
                self.padding,
                button_size,
                reach_rect=self._reachable_trajectory_canvas_rect(),
            )
            hit_zones = canvas_zones_to_stage_zones(canvas_control_zones, draw_x, draw_y)
        self.preview_button_hit_zones = hit_zones
        self.preview_canvas_button_zones = canvas_control_zones
        self.preview_canvas_drawing_zone = self._drawing_zone_canvas_rect()

        self._redraw_canvas()
        self._update_ocr_toolbar_visuals()

    def _on_clear_button(self):
        if self.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("X")
        else:
            self._clear_trajectory_trace()
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Cleared)"))

    def _on_recognize_button(self):
        self.root.after(0, lambda: self.ui_state_var.set("State: Recognition is automatic; wait for stable trajectory."))

    def _on_confirm_publish_button(self):
        self._confirm_rank_by_dwell(1)

    def _on_reject_button(self):
        if self.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("A")
        else:
            self._clear_candidates("Rejected by REJECT button")
            self.root.after(0, lambda: self.ui_state_var.set("State: BLOCKED (Rejected)"))

    def _on_resample_button(self):
        if self.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("B")
        else:
            self._clear_trajectory_trace()
            self.root.after(0, lambda: self.ui_state_var.set("State: Ready for new sample."))

    def _build_status_cards(self, parent):
        tk = self.tk
        ttk = self.ttk
        self.dashboard_wrapped_labels = []

        def create_card(
            title, *vars, static_texts=None, wraplength=340, font_size=10,
            emphasized_vars=(), secondary_vars=(), expand=False,
        ):
            card = ttk.Frame(parent, style="Card.TFrame", padding=8)
            card.pack(fill=tk.BOTH if expand else tk.X, expand=expand, pady=(0, 8))
            ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor=tk.W, pady=(0, 5))
            if static_texts:
                for st in static_texts:
                    label = ttk.Label(card, text=st, background="white", font=("TkDefaultFont", 9), wraplength=wraplength, justify=tk.LEFT)
                    label.pack(anchor=tk.W, fill=tk.X)
                    self.dashboard_wrapped_labels.append((label, wraplength))
            for var in vars:
                if var:
                    emphasized = var in emphasized_vars
                    font = ("TkDefaultFont", font_size + 3, "bold") if emphasized else ("TkDefaultFont", font_size)
                    foreground = "#64748B" if var in secondary_vars else "#1F2937"
                    label = ttk.Label(
                        card, textvariable=var, background="white",
                        foreground=foreground, font=font,
                        wraplength=wraplength, justify=tk.LEFT,
                    )
                    label.pack(anchor=tk.W, fill=tk.X)
                    self.dashboard_wrapped_labels.append((label, wraplength))
            return card

        if self.is_trajectory_mode:
            self._build_trajectory_status_cards(parent, create_card)
        elif self.demo_input_mode == "ocr_canvas":
            self._build_mouse_status_cards(create_card)
        else:
            create_card("Status", self.ui_state_var, self.capture_var, self.confirm_var)
            create_card(
                "Recognition",
                self.backend_status_var,
                self.recognizer_mode_var,
                self.recognizer_model_var,
                self.recognizer_status_var,
                self.recognizer_labels_var,
                self.rank_vars[0],
                self.rank_vars[1],
                self.rank_vars[2],
            )
            create_card("Safety", static_texts=[
                self.route_scope_text,
                self.confirm_policy_text,
            ])

    def _build_trajectory_status_cards(self, parent, create_card):
        create_card("Interaction", self.traj_clutch_mode_var, font_size=13,
                    emphasized_vars=(self.traj_clutch_mode_var,))
        if not self.candidate_preview_enabled:
            return
        create_card("Progress", self.traj_workflow_var, font_size=11,
                    emphasized_vars=(self.traj_workflow_var,))
        create_card(
            "Recognition", self.traj_operator_result_var,
            self.traj_operator_mapping_var, *self.traj_operator_candidate_vars,
            wraplength=350, font_size=11,
            emphasized_vars=(self.traj_operator_result_var,),
            secondary_vars=tuple(self.traj_operator_candidate_vars[1:]),
            expand=True,
        )
        create_card("Status", self.traj_operator_status_var, font_size=12,
                    emphasized_vars=(self.traj_operator_status_var,))
        if self.dashboard_debug_sample_panel:
            create_card(
                "Trajectory Diagnostics", self.traj_mode_var, self.traj_window_var,
                self.traj_valid_var, self.traj_xyz_var, self.traj_count_var,
                self.traj_input_gate_var, self.traj_backend_var,
                self.traj_recognizer_detail_var, self.traj_result_var,
                self.traj_recognition_progress_var,
                self.traj_candidate_debug_var, font_size=9,
            )
            create_card(
                "Drawing Sample", self.traj_captured_sample_var,
                self.traj_candidate_points_var, self.traj_sample_state_var,
                self.traj_sample_cleanup_var,
            )
            create_card(
                "Interaction", self.traj_recording_var,
                self.traj_interaction_var, self.traj_semantic_var,
                self.traj_confirm_var,
            )
        if self.trajectory_recording_enabled:
            rec_card = self.ttk.Frame(parent, style="Card.TFrame", padding=10)
            rec_card.pack(fill=self.tk.X, pady=(0, 10))
            self.ttk.Label(rec_card, text="Segment recording (preview)",
                           style="CardTitle.TLabel").pack(anchor=self.tk.W, pady=(0, 5))
            self.ttk.Button(rec_card, text="Start (record segment)  [S]",
                            command=self._start_recording).pack(anchor=self.tk.W)

    def _build_mouse_status_cards(self, create_card):
        create_card("Interaction", self.mouse_mode_var, font_size=13,
                    emphasized_vars=(self.mouse_mode_var,))
        create_card("Progress", self.mouse_workflow_var, font_size=11,
                    emphasized_vars=(self.mouse_workflow_var,))
        create_card(
            "Recognition", self.mouse_operator_result_var,
            self.mouse_operator_mapping_var,
            *self.mouse_operator_candidate_vars, font_size=11,
            emphasized_vars=(self.mouse_operator_result_var,),
            secondary_vars=tuple(self.mouse_operator_candidate_vars[1:]),
            expand=True,
        )
        create_card("Status", self.mouse_operator_status_var, font_size=12,
                    emphasized_vars=(self.mouse_operator_status_var,))
        if self.dashboard_debug_sample_panel:
            create_card(
                "Mouse Diagnostics", self.ui_state_var, self.capture_var,
                self.backend_status_var, self.recognizer_mode_var,
                self.recognizer_model_var, self.recognizer_labels_var,
                self.dwell_var, self.mouse_candidate_debug_var, font_size=9,
            )

    def _build_safety_footer(self, parent):
        if not self.footer_text:
            return
        tk = self.tk
        ttk = self.ttk
        footer = tk.Frame(parent, bg="#EAECEE", pady=10, padx=15)
        footer.pack(fill=tk.X, side=tk.BOTTOM)

        text = self.footer_text
        ttk.Label(footer, text=text, foreground="#7F8C8D", background="#EAECEE", font=("TkDefaultFont", 10, "bold")).pack(anchor=tk.CENTER)

    def _redraw_canvas(self):
        self.canvas.delete("all")

        # Refresh the active scaling window once per frame (auto / fixed_symmetric).
        if self.is_trajectory_mode:
            self._active_bounds = self._active_trajectory_bounds()

        if self.demo_input_mode == "ocr_canvas" and self._ocr_draw_area:
            x0, y0, x1, y1 = self._ocr_draw_area
            zone_style = drawing_zone_style()
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill="#FFFFFF",
                outline=zone_style["outline"], width=2, dash=(5, 4))
            self.canvas.create_text(
                self.canvas_width / 2, y0 + 18, text="Drawing zone",
                fill=zone_style["text"], font=("TkDefaultFont", 10, "bold"),
            )
        elif self.is_trajectory_mode:
            # Draw x/y axes through the mapped world origin so orientation and
            # scale are visually checkable.
            ox, oy = self._canvas_xy(0.0, 0.0)
            self.canvas.create_line(self.padding, oy, self.canvas_width-self.padding, oy, fill="#D5DBDB", dash=(4, 4))
            self.canvas.create_line(ox, self.padding, ox, self.canvas_height-self.padding, fill="#D5DBDB", dash=(4, 4))
        else:
            # Grid lines
            self.canvas.create_line(self.padding, self.canvas_height/2, self.canvas_width-self.padding, self.canvas_height/2, fill="#ECF0F1", dash=(4, 4))
            self.canvas.create_line(self.canvas_width/2, self.padding, self.canvas_width/2, self.canvas_height-self.padding, fill="#ECF0F1", dash=(4, 4))

        if self.is_trajectory_mode and self._use_canvas_reachable_preview_controls():
            zone = self.preview_canvas_drawing_zone or self._drawing_zone_canvas_rect()
            zone_style = drawing_zone_style()
            self.canvas.create_rectangle(
                zone["x1"], zone["y1"], zone["x2"], zone["y2"],
                outline=zone_style["outline"], width=2, dash=(6, 4), fill="")
            self.canvas.create_text(
                zone["x1"] + 8, zone["y1"] + 12,
                text="Drawing zone", anchor=self.tk.W,
                fill=zone_style["text"],
                font=("TkDefaultFont", 9, "bold"))

        if self.demo_input_mode != "ocr_canvas":
            for segment in split_trail_segments(getattr(self, "drawing_zone_trail", [])):
                coords = []
                for x, y in segment:
                    cx, cy = self._canvas_xy(x, y)
                    coords.extend([cx, cy])
                self.canvas.create_line(*coords, fill="#16A34A", width=4)

        # Draw frozen sample sent to recognition. It intentionally differs from
        # the live trail color so the operator can inspect what B recognized.
        if len(self.captured_path) > 1:
            pts = []
            for x, y in self.captured_path:
                if self.is_trajectory_mode:
                    cx, cy = self._canvas_xy(x, y)
                else:
                    cx, cy = map_world_to_canvas(
                        x, y, self.bounds, self.canvas_width, self.canvas_height, self.padding)
                pts.extend([cx, cy])
            self.canvas.create_line(*pts, fill="#F59E0B", width=6, smooth=True)
            self.canvas.create_line(*pts, fill="#FEF3C7", width=2, smooth=True)
            if getattr(self, "dashboard_debug_sample_panel", False):
                label = "Captured sample"
                if getattr(self, "captured_sample_point_count", 0):
                    label += " (%d pts)" % self.captured_sample_point_count
                self.canvas.create_text(
                    self.padding + 8, self.padding + 12,
                    text=label, anchor=self.tk.W,
                    fill="#92400E", font=("TkDefaultFont", 10, "bold"))

        # Draw live trail. ``None`` entries are stroke breaks (gated / non-writing
        # points) that split the polyline into separate segments. Single-point
        # segments are dropped so create_line is never called with < 2 points.
        if self.demo_input_mode != "ocr_canvas":
            for segment in split_trail_segments(self.trail):
                coords = []
                for x, y in segment:
                    cx, cy = self._canvas_xy(x, y)
                    coords.extend([cx, cy])
                self.canvas.create_line(*coords, fill="#2563EB", width=3)

        # Draw manual strokes for OCR Canvas
        for stroke in self.all_strokes:
            if len(stroke) > 1:
                pts = []
                for x, y in stroke:
                    cx, cy = map_world_to_canvas(x, y, self.bounds, self.canvas_width, self.canvas_height, self.padding)
                    pts.extend([cx, cy])
                self.canvas.create_line(*pts, fill="#8E44AD", width=3, smooth=True)
        if len(self.drawing_stroke) > 1:
            pts = []
            for x, y in self.drawing_stroke:
                cx, cy = map_world_to_canvas(x, y, self.bounds, self.canvas_width, self.canvas_height, self.padding)
                pts.extend([cx, cy])
            self.canvas.create_line(*pts, fill="#8E44AD", width=3, smooth=True)

        # Draw current point (grey when gated as non-writing or invalid).
        if self.is_trajectory_mode:
            if self.latest_point is not None:
                cx, cy = self._canvas_xy(self.latest_point[0], self.latest_point[1])
                r = 8
                drawing = self.latest_valid and self.latest_writing
                fill, outline = ("#E74C3C", "#C0392B") if drawing else ("#AEB6BF", "#7F8C8D")
                self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=fill, outline=outline)
        elif self.trail and self.demo_input_mode != "ocr_canvas":
            x, y = self.trail[-1]
            cx, cy = self._canvas_xy(x, y)
            r = 8
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#E74C3C", outline="#C0392B")

        self._draw_canvas_reachable_preview_buttons()
        self._draw_virtual_buttons()
        self._draw_controller_mode_indicator()

    def _draw_virtual_buttons(self):
        pass

    def _draw_canvas_reachable_preview_buttons(self):
        if not self._use_canvas_reachable_preview_controls():
            return
        current_button = getattr(self, "ocr_hover_button", "")
        progress = getattr(self, "ocr_dwell_progress", 0.0)
        for zone in getattr(self, "preview_canvas_button_zones", []) or []:
            button_id = str(zone.get("id", "")).upper()
            enabled = self._preview_button_enabled(button_id)
            active = button_id == current_button
            radius = float(zone["radius"])
            cx = float(zone["cx"])
            cy = float(zone["cy"])
            style = preview_button_style(button_id, enabled, active=active)
            self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                    fill=style["fill"], outline=style["outline"], width=style["width"])
            self.canvas.create_text(cx, cy, text=zone.get("label", button_id),
                                    font=("TkDefaultFont", max(8, int(radius * 0.34)), "bold"),
                                    justify=self.tk.CENTER, fill=style["text"])
            if active and enabled and progress > 0.0 and self.hover_progress_enabled:
                self.canvas.create_arc(cx - radius + 3, cy - radius + 3,
                                       cx + radius - 3, cy + radius - 3,
                                       start=90, extent=-360 * progress,
                                       style=self.tk.ARC, outline=style["ring"], width=4)

    def _draw_rank_confirm_buttons(self):
        current_button = self.dwell_state.get("current_button_id", "")
        progress = self.dwell_state.get("dwell_progress", 0.0)
        has_candidates = bool(summarize_candidates(self.current_candidate_payload))
        for rect in self.virtual_button_rects:
            button_id = rect["id"]
            enabled = button_id == "clear" or has_candidates
            active = button_id == current_button
            fill = "#EAF2F8" if enabled else "#F4F6F7"
            outline = "#3498DB" if active else "#95A5A6"
            width = 3 if active else 1
            if self.last_confirmed_label and button_id == "rank_%s" % self.last_selected_rank:
                outline = "#27AE60"

            self.canvas.create_rectangle(rect["x1"], rect["y1"], rect["x2"], rect["y2"], fill=fill, outline=outline, width=width)
            if active and progress > 0.0:
                self.canvas.create_arc(rect["x1"] + 3, rect["y1"] + 3,
                                       rect["x2"] - 3, rect["y2"] - 3,
                                       start=90, extent=-360 * progress,
                                       style=self.tk.ARC, outline="#DC2626", width=4)
            label = rect["label"]
            if button_id.startswith("rank_"):
                rank = int(button_id.split("_")[1])
                candidate = _extract_candidate_by_rank(self.current_candidate_payload, rank)
                if candidate:
                    label = "%s: %s" % (label, candidate["label"])
            self.canvas.create_text((rect["x1"] + rect["x2"]) / 2, (rect["y1"] + rect["y2"]) / 2, text=label, fill="#2C3E50", font=("TkDefaultFont", 9, "bold"))

    def _draw_gamepad_buttons(self):
        current_button = self.dwell_state.get("current_button_id", "")
        progress = self.dwell_state.get("dwell_progress", 0.0)
        has_candidates = bool(summarize_candidates(self.current_candidate_payload))
        for rect in self.virtual_button_rects:
            button_id = rect["id"]
            ignored = should_ignore_gamepad_button(
                self.controller_mode,
                button_id,
                block_arrows_on_stop=self.gamepad_block_arrows_on_stop,
            )
            if button_id == "C" and self.controller_mode == GAMEPAD_MODE_CONFIRM_PENDING and not has_candidates:
                ignored = True
            active = button_id == current_button
            fill = "#F4F6F7" if ignored else "#EAF2F8"
            outline = "#3498DB" if active else "#95A5A6"
            text_fill = "#95A5A6" if ignored else "#2C3E50"
            width = 3 if active else 1
            if button_id == "A" and self.controller_mode == GAMEPAD_MODE_BLOCKED:
                outline = "#C0392B"
                text_fill = "#C0392B"

            if rect.get("kind") == "face":
                self.canvas.create_oval(rect["x1"], rect["y1"], rect["x2"], rect["y2"], fill=fill, outline=outline, width=width)
            else:
                self.canvas.create_rectangle(rect["x1"], rect["y1"], rect["x2"], rect["y2"], fill=fill, outline=outline, width=width)
            if active and progress > 0.0:
                progress_x = rect["x1"] + (rect["x2"] - rect["x1"]) * progress
                self.canvas.create_rectangle(rect["x1"], rect["y2"] - 6, progress_x, rect["y2"], fill="#2ECC71", outline="")
            self.canvas.create_text((rect["x1"] + rect["x2"]) / 2, (rect["y1"] + rect["y2"]) / 2, text=rect["label"], fill=text_fill, font=("TkDefaultFont", 12, "bold"))

    def _draw_controller_mode_indicator(self):
        if self.demo_input_mode == "ocr_canvas":
            return
        if self.controller_layout != CONTROLLER_LAYOUT_GAMEPAD:
            return
        text = "Mode: %s" % self.controller_mode
        if self.controller_mode == GAMEPAD_MODE_BLOCKED:
            text += " | A/STOP active: arrows blocked"
        elif self.controller_mode == GAMEPAD_MODE_DIGIT:
            remaining = 0.0
            if self.digit_mode_expires_at is not None:
                remaining = max(0.0, self.digit_mode_expires_at - time.time())
            text += " | write digit %.1fs" % remaining
        elif self.controller_mode == GAMEPAD_MODE_CONFIRM_PENDING:
            text += " | C confirms rank 1, X cancels"
        self.canvas.create_text(self.canvas_width / 2, self.padding + 14, text=text, fill="#2C3E50", font=("TkDefaultFont", 10, "bold"))

    def _schedule_ui_refresh(self):
        self.root.after(0, self._redraw_canvas)

    def _active_trajectory_bounds(self):
        if self.is_trajectory_mode and self.traj_scale_mode in ("auto", "fixed_symmetric"):
            pts = [p for p in self.trail if p is not None]  # drop stroke breaks
            return compute_trajectory_bounds(
                pts, self.bounds, scale_mode=self.traj_scale_mode,
                auto_center=self.traj_auto_center,
                padding_ratio=self.traj_auto_padding_ratio,
                min_span=self.traj_auto_min_span,
                window_points=self.traj_trace_window_points,
                projection_extent=self.traj_projection_extent,
            )
        return self.bounds

    def _canvas_xy(self, x, y):
        # ``self._active_bounds`` is refreshed once per redraw so auto-fit is not
        # recomputed for every trail point.
        if self.is_trajectory_mode:
            return map_trajectory_point(
                x, y, self._active_bounds,
                self.canvas_width, self.canvas_height, self.padding,
                flip_x=self.traj_flip_x, flip_y=self.traj_flip_y,
                swap_xy=self.traj_swap_xy,
            )
        return map_world_to_canvas(x, y, self.bounds, self.canvas_width, self.canvas_height, self.padding)

    def _canvas_xy_unclamped(self, x, y):
        """Like :meth:`_canvas_xy` but without clamping to the canvas rectangle.

        The buttons surround the trajectory canvas, so a clamped point (which is
        what the red dot is drawn at) can never leave the canvas to reach a
        button. Hit-testing the live board cursor uses the unclamped mapped
        position so pushing the magnet past the drawing region moves the logical
        cursor into the surrounding button zones.
        """
        if self.is_trajectory_mode:
            return map_trajectory_point(
                x, y, self._active_bounds,
                self.canvas_width, self.canvas_height, self.padding,
                flip_x=self.traj_flip_x, flip_y=self.traj_flip_y,
                swap_xy=self.traj_swap_xy, clamp=False,
            )
        return map_world_to_canvas(x, y, self.bounds, self.canvas_width, self.canvas_height, self.padding)

    def _canvas_xy_to_stage_xy(self, canvas_x, canvas_y):
        """Convert a trajectory-canvas pixel to the stage-local coordinate system
        used by the preview button hit zones. Same frame the mouse pointer is
        mapped into by :meth:`_preview_button_at_event`, so the red-point check
        and the visible circles agree."""
        canvas_root_x = float(self.canvas.winfo_rootx())
        canvas_root_y = float(self.canvas.winfo_rooty())
        stage_root_x = float(self.ocr_stage_card.winfo_rootx())
        stage_root_y = float(self.ocr_stage_card.winfo_rooty())
        stage_x = canvas_root_x + float(canvas_x) - stage_root_x
        stage_y = canvas_root_y + float(canvas_y) - stage_root_y
        return stage_x, stage_y

    def _resolve_preview_button_at_stage_xy(self, stage_x, stage_y):
        """Resolve a stage-local (x, y) to a preview button id (or "")."""
        return resolve_preview_button_hit(stage_x, stage_y, self.preview_button_hit_zones)

    def _board_cursor_preview_button(self):
        """Preview button currently under the live board's red current point.

        Returns the button id (X/A/B/C) or "" if the cursor is not over any
        button. Uses the unclamped mapped position so an extreme board position
        reaches the buttons around the canvas, then converts to the same
        stage-local frame as the hit zones."""
        if not self._preview_pointer_tracking_enabled():
            return ""
        if getattr(self, "latest_point", None) is None:
            return ""
        if not getattr(self, "latest_input_active", True):
            return ""
        if not self.preview_button_hit_zones:
            return ""
        try:
            if self._use_canvas_reachable_preview_controls():
                cx, cy = self._canvas_xy(self.latest_point[0], self.latest_point[1])
            else:
                cx, cy = self._canvas_xy_unclamped(self.latest_point[0], self.latest_point[1])
            stage_x, stage_y = self._canvas_xy_to_stage_xy(cx, cy)
        except (AttributeError, TypeError, ValueError, self.tk.TclError):
            return ""
        return self._resolve_preview_button_at_stage_xy(stage_x, stage_y)

    def _handle_trajectory(self, msg):
        data = safe_json_loads(msg.data)
        if not data:
            return
        pt = extract_single_point(data)
        if pt is None:
            return
        if self.is_trajectory_mode:
            self._ingest_trajectory_sample(data, pt)
            return
        self.trail.append(pt)
        if hasattr(self, '_process_virtual_button_dwell'):
            self._process_virtual_button_dwell(pt)
        self._schedule_ui_refresh()

    def _ingest_trajectory_sample(self, data, pt):
        """Pure trajectory viewer: update the trail + live status only.

        Never publishes any intent, task command, or confirmed label. Points that
        are invalid, or that the preview stroke gate marks as non-writing, update
        the live status and current marker but are not appended to the drawn
        trail; instead they insert a stroke break so the next writing point
        starts a fresh segment. Existing history is never cleared here.
        """
        decision = self._classify_trajectory_sample(data, pt)
        inside_drawing_zone, inside_control_zone = (
            self._update_trajectory_sample_buffers(pt, decision)
        )
        self._update_trajectory_sample_status(
            data,
            pt,
            decision,
            inside_drawing_zone,
            inside_control_zone,
        )

        if getattr(self, "trajectory_recording_enabled", False):
            self._feed_recorder(
                pt,
                decision["z"],
                decision["timestamp"],
                decision["valid"],
                decision["writing"],
            )

        if (
            self.candidate_preview_enabled
            and self.auto_recognize
            and decision["writing"]
            and inside_drawing_zone
        ):
            self.root.after(0, self._schedule_auto_recognize)

    def _classify_trajectory_sample(self, data, pt):
        valid = bool(data.get("valid", True))
        z_val = opt_float(data.get("z"))
        input_inactive_reason = trajectory_input_inactive_reason(
            valid,
            z_val,
            z_min=getattr(self, "input_z_min", None),
            z_max=getattr(self, "input_z_max", None),
            gate_enabled=getattr(self, "input_valid_gate_enabled", True),
        )
        raw_input_active = input_inactive_reason == ""
        if raw_input_active:
            self._input_active_streak += 1
        else:
            self._input_active_streak = 0
        input_active = raw_input_active and self._input_active_streak >= self.input_reactivate_samples

        # Instantaneous XY speed from the previous sample (for speed gating).
        ts = data.get("timestamp", data.get("stamp"))
        speed = None
        if self._prev_sample is not None:
            px, py, pts_t = self._prev_sample
            if isinstance(ts, (int, float)) and isinstance(pts_t, (int, float)):
                dt = ts - pts_t
                if dt > 0:
                    speed = math.hypot(pt[0] - px, pt[1] - py) / dt
        self._prev_sample = (pt[0], pt[1], ts if isinstance(ts, (int, float)) else None)

        writing_gate = stroke_gate_pass(
            self.stroke_gate_mode, z_val, speed,
            z_min=self.gate_z_min, z_max=self.gate_z_max,
            min_speed=self.gate_min_speed, max_speed=self.gate_max_speed,
        )
        writing = input_active and writing_gate
        return {
            "valid": valid,
            "z": z_val,
            "timestamp": ts,
            "input_active": input_active,
            "input_inactive_reason": input_inactive_reason,
            "writing": writing,
        }

    def _update_trajectory_sample_buffers(self, pt, decision):
        valid = decision["valid"]
        input_active = decision["input_active"]
        input_inactive_reason = decision["input_inactive_reason"]
        writing = decision["writing"]
        self.latest_point = pt
        self.latest_valid = valid
        self.latest_input_active = input_active
        self.latest_input_inactive_reason = "" if input_active else (input_inactive_reason or "reactivating")
        self.latest_writing = writing
        self.trajectory_sample_count += 1

        inside_drawing_zone = False
        inside_control_zone = False
        if input_active:
            inside_drawing_zone = self._trajectory_point_in_drawing_zone(pt)
            inside_control_zone = self._trajectory_point_in_control_zone(pt)
            self._start_new_drawing_after_recognized_if_needed(inside_drawing_zone, writing)
            prev_frozen = bool(self.drawing_sample_state.get("frozen_points"))
            prev_was_inside = bool(self.drawing_sample_state.get("was_inside"))
            self.drawing_sample_state = update_drawing_zone_sample_state(
                self.drawing_sample_state,
                pt,
                inside_drawing_zone,
                input_active=True,
                inside_control_zone=inside_control_zone,
                append_point=writing,
            )
            self._sync_drawing_sample_buffers()
            if writing and inside_drawing_zone and not prev_frozen:
                self.drawing_zone_trail.append(pt)
            elif prev_was_inside and not inside_drawing_zone and not prev_frozen:
                if self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                    self.drawing_zone_trail.append(None)
        else:
            prev_was_inside = bool(self.drawing_sample_state.get("was_inside"))
            self.drawing_sample_state = update_drawing_zone_sample_state(
                self.drawing_sample_state,
                pt,
                inside_drawing_zone=False,
                input_active=False,
                append_point=False,
            )
            self._sync_drawing_sample_buffers()
            if prev_was_inside and self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                self.drawing_zone_trail.append(None)
        self.latest_inside_drawing_zone = bool(inside_drawing_zone)
        self.latest_inside_control_zone = bool(inside_control_zone)

        if writing:
            self.trail.append(pt)
        elif self.gate_invalid_breaks_stroke or not input_active:
            # Break the drawn stroke (don't append the point, don't clear history).
            if self.trail and self.trail[-1] is not None:
                self.trail.append(None)
            if self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                self.drawing_zone_trail.append(None)
        return inside_drawing_zone, inside_control_zone

    def _update_trajectory_sample_status(
        self,
        data,
        pt,
        decision,
        inside_drawing_zone,
        inside_control_zone,
    ):
        valid = decision["valid"]
        z_val = decision["z"]
        input_active = decision["input_active"]
        writing = decision["writing"]
        z_txt = "%.4f" % z_val if z_val is not None else "-"
        xyz = "x/y/z: %.4f / %.4f / %s" % (pt[0], pt[1], z_txt)
        idx = data.get("sample_index")
        self.latest_sample_index = idx
        count_txt = "Samples: %d" % self.trajectory_sample_count
        if idx is not None:
            count_txt += " (idx %s)" % idx
        valid_txt = "Valid: %s | input active: %s | writing: %s" % (
            "true" if valid else "false",
            "yes" if input_active else "no",
            "yes" if writing else "no")
        filt = data.get("filter_mode", "-")
        input_txt = "Input: active %s | drawing %s | control %s | reason %s" % (
            "yes" if input_active else "no",
            "yes" if inside_drawing_zone else "no",
            "yes" if inside_control_zone else "no",
            self.latest_input_inactive_reason or "-",
        )

        self.root.after(0, lambda t=valid_txt: self.traj_valid_var.set(t))
        self.root.after(0, lambda t=xyz: self.traj_xyz_var.set(t))
        self.root.after(0, lambda t=count_txt: self.traj_count_var.set(t))
        self.root.after(0, lambda t=input_txt: self.traj_input_gate_var.set(t))
        self.root.after(0, lambda t=filt: self.traj_filter_var.set("Filter: %s" % t))
        self._update_orientation_clutch_mode()
        self._refresh_operator_status()
        self._schedule_ui_refresh()

    def _update_orientation_clutch_mode(self, hovered_button=None):
        if hovered_button is None:
            hovered_button = ""
            if (
                getattr(self, "latest_input_active", False)
                and getattr(self, "latest_inside_control_zone", False)
                and getattr(self, "is_trajectory_mode", False)
                and getattr(self, "candidate_preview_enabled", False)
            ):
                hovered_button = self._board_cursor_preview_button()
        mode = derive_orientation_clutch_mode(
            getattr(self, "latest_input_active", False),
            getattr(self, "latest_inside_drawing_zone", False),
            hovered_button,
        )
        if hasattr(self, "traj_clutch_mode_var"):
            self.root.after(
                0,
                lambda t=mode: self.traj_clutch_mode_var.set("Mode: %s" % t),
            )
        return mode

    # --- Preview-only segment recording (M63-B): display only, no publish ---
    def _start_recording(self):
        """Start button: clear the previous segment and record a new one.

        Preview-only. Clears the drawn trail + candidates + preview confirm and
        arms the segment recorder. Recording begins on the first writing sample
        and stops after ``record_duration_sec`` (or z-lift when enabled). Never
        publishes anything.
        """
        if not self.trajectory_recording_enabled:
            return
        # Reuse the clear path so trail/candidates/confirm reset consistently.
        self._clear_trajectory_trace()
        self.recorded_segment = []
        self.segment_recorder.start(time.time())
        self.candidate_status = "recording"
        self.preview_interaction_state = "recording"
        self._update_candidate_ui()
        self._schedule_ui_refresh()

    def _feed_recorder(self, pt, z_val, ts, valid, writing):
        """Feed one live sample into the segment recorder (recording mode only)."""
        rec = self.segment_recorder
        if rec.state not in ("ARMED", "RECORDING"):
            return
        now = ts if isinstance(ts, (int, float)) else time.time()
        state = rec.feed(pt[0], pt[1], now, z=z_val, valid=valid, writing=writing)
        if state == "RECORDING_DONE":
            self._on_recording_done()
        elif state == "RECORDING":
            self.candidate_status = "recording"
            self.preview_interaction_state = "recording"
            self._update_candidate_ui()

    def _on_recording_done(self):
        """A complete segment is captured. Optionally auto-recognize it."""
        self.recorded_segment = self.segment_recorder.segment()
        self.candidate_status = "recording_done"
        self.preview_interaction_state = "recording_done"
        self._update_candidate_ui()
        self._schedule_ui_refresh()
        if self.candidate_preview_enabled and self.auto_recognize:
            self._recognize_current_stroke()

    def _active_recognition_points(self):
        """Points the recognizer should use.

        Real-board canvas controls keep live cursor movement and button hover in
        ``trail`` for operator feedback, but recognition uses the central
        drawing-zone buffer only.

        When segment recording is enabled and configured to recognize only the
        completed segment, use the recorded segment; otherwise use the live trail.
        Finalizes an in-progress recording first so Recognize always works on a
        complete segment.
        """
        if self.is_trajectory_mode and self._use_canvas_reachable_preview_controls():
            frozen = getattr(self, "frozen_drawing_sample", [])
            if frozen:
                return list(frozen)
            return list(getattr(self, "active_drawing_buffer", []))
        rec = getattr(self, "segment_recorder", None)
        if (getattr(self, "trajectory_recording_enabled", False)
                and getattr(self, "recognize_only_completed_segment", True)
                and rec is not None):
            if rec.is_recording:
                rec.finish_now(time.time(), reason="recognize")
                self.recorded_segment = rec.segment()
            return list(getattr(self, "recorded_segment", []) or [])
        return self.trail

    def _recording_status_text(self):
        rec = getattr(self, "segment_recorder", None)
        if not getattr(self, "trajectory_recording_enabled", False) or rec is None:
            return "Recording: disabled"
        state_map = {
            "IDLE": "idle (press S/Start)",
            "ARMED": "armed — waiting for pen down",
            "RECORDING": "recording…",
            "RECORDING_DONE": "segment ready (%d pts, stop=%s)" % (
                len(rec.points), rec.stop_reason or "-"),
            "CLEARED": "cleared",
        }
        return "Recording: %s" % state_map.get(rec.state, rec.state)

    def _semantic_preview_text(self):
        """Preview-only meaning of the top candidate (display only, no dispatch)."""
        if not getattr(self, "semantic_preview_enabled", True):
            return "Meaning: -"
        cands = getattr(self, "trajectory_candidates", None)
        if not cands:
            return "Meaning: -"
        try:
            from colmag_ros.semantic_library import preview_line
        except Exception:
            return "Meaning: -"
        try:
            return "Meaning: %s" % preview_line(cands[0])
        except Exception:
            return "Meaning: -"

    # --- Preview-only OCR/candidate bridge (M62-A4/A5): display only, no publish ---
    def _cancel_candidate_recognize_job(self):
        if self._candidate_recognize_job is None:
            return
        try:
            self.root.after_cancel(self._candidate_recognize_job)
        except self.tk.TclError:
            pass
        self._candidate_recognize_job = None

    def _schedule_auto_recognize(self):
        if not (self.candidate_preview_enabled and self.auto_recognize):
            return
        self._cancel_candidate_recognize_job()
        idle_ms = max(50, int(self.ocr_debounce_sec * 1000))
        self._candidate_recognize_job = self.root.after(idle_ms, self._recognize_current_stroke)

    def _recognize_current_stroke(self):
        self._candidate_recognize_job = None
        if not (self.is_trajectory_mode and self.candidate_preview_enabled):
            return
        self.candidate_status = "recognizing"
        self.preview_interaction_state = "recognizing"
        self._update_candidate_ui()
        status_out = {}
        points = self._active_recognition_points()
        status, candidates, backend_label = recognize_stroke(
            points,
            backend=getattr(self, "recognition_backend", "dtw"),
            ocr_fallback_dtw=getattr(self, "ocr_fallback_dtw", True),
            min_points=getattr(self, "candidate_min_points", 8),
            canvas_size=getattr(self, "ocr_canvas_size", 256),
            line_width=getattr(self, "ocr_line_width", 8),
            padding_ratio=getattr(self, "ocr_padding_ratio", 0.15),
            labels=getattr(self, "candidate_labels", None),
            status_out=status_out,
            recognition_model_path=getattr(self, "recognition_model_path", ""),
        )
        self.recognition_backend_label = backend_label
        self.candidate_status = status
        self.trajectory_candidates = candidates
        if status == "ready":
            self.preview_interaction_state = "candidates_ready"
        elif status == "collecting":
            self.preview_interaction_state = "collecting"
        elif status in ("unavailable", "error"):
            self.preview_interaction_state = "error"
        else:
            self.preview_interaction_state = status
        self._update_candidate_ui()

    def _clear_trajectory_trace(self):
        self._cancel_candidate_recognize_job()
        self.trail.clear()
        self._reset_drawing_sample_state()
        self.latest_point = None
        self._prev_sample = None
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self._clear_published_sample_state()
        self._clear_sample_record_reference()
        self.current_candidate_payload = None
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.trajectory_candidates = []
        self.candidate_status = "idle"
        self.preview_interaction_state = "cleared"
        self.preview_confirmed_label = ""
        if getattr(self, "segment_recorder", None) is not None:
            self.segment_recorder.clear()
            self.recorded_segment = []
        self._update_candidate_ui()
        if hasattr(self, "traj_captured_sample_var"):
            self.root.after(0, lambda: self.traj_captured_sample_var.set("Captured sample: none"))
        if hasattr(self, "capture_var"):
            self.root.after(0, lambda: self.capture_var.set("Cleared trajectory and captured sample"))
        self._schedule_ui_refresh()

    def _reject_preview_candidate(self):
        self._cancel_candidate_recognize_job()
        self.trajectory_candidates = []
        self.current_candidate_payload = None
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self._clear_published_sample_state()
        self._clear_sample_record_reference()
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.candidate_status = "rejected"
        self.preview_interaction_state = "rejected"
        self.preview_confirmed_label = ""
        self._update_candidate_ui()
        if hasattr(self, "traj_captured_sample_var"):
            self.root.after(0, lambda: self.traj_captured_sample_var.set("Captured sample: none"))
        if hasattr(self, "capture_var"):
            self.root.after(0, lambda: self.capture_var.set("Rejected candidate and captured sample"))
        self._schedule_ui_refresh()

    def _preview_confirm_candidate(self):
        label = self._top_preview_candidate_label()
        if not label:
            self.preview_interaction_state = "ready" if self.trajectory_candidates else "idle"
            self._update_candidate_ui()
            return
        self.preview_confirmed_label = label
        self.preview_interaction_state = "preview_confirmed"
        # M95: the trajectory-preview C confirm now also publishes the confirmed
        # label (board red cursor / mouse preview), not just the preview UI. The
        # publish itself is gated inside the helper (double gate + de-dup).
        self._publish_preview_confirmed_label()
        self._update_candidate_ui()
        self._schedule_ui_refresh()

    def _set_captured_sample_display(self, sample_id, points, raw_point_count=None):
        points = list(points or [])
        self.captured_path = points
        self.captured_sample_id = str(sample_id or "unknown")
        self.captured_sample_point_count = len(points)
        self.captured_sample_at = time.time()
        self.published_sample_points = list(points)
        self.published_sample_id = self.captured_sample_id
        self.published_sample_point_count = len(points)
        if raw_point_count is None:
            raw_point_count = len(points)
        self.published_sample_raw_point_count = int(raw_point_count)
        text = "Published sample: %s | raw %d pts -> published %d pts" % (
            self.captured_sample_id,
            int(self.published_sample_raw_point_count),
            self.captured_sample_point_count)
        if hasattr(self, "traj_captured_sample_var"):
            self.root.after(0, lambda t=text: self.traj_captured_sample_var.set(t))
        if hasattr(self, "capture_var"):
            self.root.after(0, lambda t=text: self.capture_var.set(t))

    def _publish_trajectory_symbol_capture_from_drawing_zone(self):
        self._freeze_current_drawing_sample()
        raw_points = clean_stroke_points(self._active_recognition_points())
        points = cleanup_drawing_sample_points(
            raw_points,
            min_point_delta=getattr(self, "board_sample_min_point_delta", 0.0),
            max_points=getattr(self, "board_sample_max_points", 0),
            enabled=getattr(self, "board_sample_cleanup_enabled", True),
        )
        if len(points) < int(getattr(self, "candidate_min_points", 8)):
            self.candidate_status = "collecting"
            self.preview_interaction_state = "too_few_drawing_points"
            text = "Too few published drawing points: %d / %d (raw %d)" % (
                len(points), int(getattr(self, "candidate_min_points", 8)), len(raw_points))
            if hasattr(self, "traj_captured_sample_var"):
                self.root.after(0, lambda t=text: self.traj_captured_sample_var.set(t))
            if hasattr(self, "capture_var"):
                self.root.after(0, lambda t=text: self.capture_var.set(t))
            self._update_candidate_ui()
            return False
        now = time.time()
        sample_id = "dashboard_traj_%d" % int(now * 1000)
        sequence_id = int(self.trajectory_sample_count)
        cleanup_meta = sample_cleanup_metadata(
            raw_points,
            points,
            min_point_delta=getattr(self, "board_sample_min_point_delta", 0.0),
            max_points=getattr(self, "board_sample_max_points", 0),
            enabled=getattr(self, "board_sample_cleanup_enabled", True),
        )
        payload = build_dashboard_drawing_zone_capture_payload(
            points,
            sample_id,
            sequence_id,
            now,
            self.trajectory_topic,
            drawing_zone=self._drawing_zone_canvas_rect(),
            raw_points=raw_points,
            cleanup_metadata=cleanup_meta,
        )
        payload["drawing_zone_sample_frozen"] = bool(getattr(self, "frozen_drawing_sample", []))
        self.last_capture_sample_id = sample_id
        self.last_capture_sequence_id = sequence_id
        self._set_captured_sample_display(sample_id, points, raw_point_count=len(raw_points))
        self._record_real_board_sample(payload, raw_points, points, cleanup_meta)
        self.symbol_capture_pub.publish(self.String(data=json.dumps(payload, sort_keys=True)))
        self.candidate_status = "recognizing"
        self.preview_interaction_state = "symbol_capture_published"
        self._update_candidate_ui()
        self._publish_dashboard_state()
        return True

    def _publish_trajectory_symbol_capture_from_trail(self):
        return self._publish_trajectory_symbol_capture_from_drawing_zone()

    def _external_confirm_active(self):
        """True when the trajectory-mode dwell confirm should confirm the external
        ROS /colmag/symbol_candidates payload (e.g. dtw_template_bank) instead of
        the dashboard's in-process preview candidates. Falls back to internal
        (returns False) when the source is not selected or no external candidates
        are available."""
        return (
            getattr(self, "is_trajectory_mode", False)
            and getattr(self, "confirm_candidate_source", "internal") == "external_symbol_candidates"
            and bool((getattr(self, "current_candidate_payload", None) or {}).get("candidates"))
        )

    def _publish_preview_confirmed_label(self):
        if self._external_confirm_active():
            result = self.preview_confirm_publisher.publish_external_payload(
                confirm_pub=getattr(self, "confirm_pub", None),
                integrated_confirm_enabled=getattr(self, "integrated_confirm_enabled", False),
                candidate_payload=self.current_candidate_payload,
                last_confirm_key=self.last_confirm_key,
                expected_sample_id=getattr(self, "last_capture_sample_id", None),
                expected_sequence_id=getattr(self, "last_capture_sequence_id", None),
            )
        else:
            result = self.preview_confirm_publisher.publish_rank_one(
                confirm_pub=getattr(self, "confirm_pub", None),
                integrated_confirm_enabled=getattr(self, "integrated_confirm_enabled", False),
                trajectory_candidates=self.trajectory_candidates,
                last_confirm_key=self.last_confirm_key,
            )
        if not result.published:
            return
        self.last_confirm_key = result.confirm_key
        self.last_confirmed_label = result.label
        self.last_selected_rank = result.selected_rank
        self.last_command_intent = result.command_intent
        self.root.after(0, lambda: self.confirm_var.set(
            "Confirmed preview '%s' -> /colmag/confirmed_label" % self.last_confirmed_label))
        self._publish_dashboard_state()

    def _top_preview_candidate_label(self):
        if self._external_confirm_active():
            tops = summarize_candidates(self.current_candidate_payload)
            if tops:
                return str(tops[0][1] or "")
        if not self.trajectory_candidates:
            return ""
        return str(self.trajectory_candidates[0].get("label", "") or "")

    def _drawing_sample_ready(self):
        raw_points = clean_stroke_points(self._active_recognition_points())
        points = cleanup_drawing_sample_points(
            raw_points,
            min_point_delta=getattr(self, "board_sample_min_point_delta", 0.0),
            max_points=getattr(self, "board_sample_max_points", 0),
            enabled=getattr(self, "board_sample_cleanup_enabled", True),
        )
        return len(points) >= int(getattr(self, "candidate_min_points", 8))

    def _preview_button_states(self):
        has_sample = self._drawing_sample_ready()
        has_candidate = bool(self._top_preview_candidate_label())
        has_clearable_content = bool(
            clean_stroke_points(getattr(self, "trail", []))
            or clean_stroke_points(getattr(self, "active_drawing_buffer", []))
            or clean_stroke_points(getattr(self, "frozen_drawing_sample", []))
            or getattr(self, "captured_path", [])
            or getattr(self, "current_candidate_payload", None)
            or getattr(self, "trajectory_candidates", [])
            or getattr(self, "recorded_segment", [])
        )
        return preview_button_availability(
            has_sample, has_candidate, has_clearable_content,
        )

    def _handle_preview_button_action(self, button_id):
        button_id = str(button_id or "").upper()
        if not self._preview_button_enabled(button_id):
            self._update_ocr_toolbar_visuals()
            return False
        if button_id == "X":
            self._clear_trajectory_trace()
        elif button_id == "A":
            self._reject_preview_candidate()
        elif button_id == "B":
            if self._external_confirm_active() or self.confirm_candidate_source == "external_symbol_candidates":
                self._publish_trajectory_symbol_capture_from_drawing_zone()
            else:
                self._recognize_current_stroke()
        elif button_id == "C":
            self._preview_confirm_candidate()
        self._update_ocr_toolbar_visuals()
        return True

    def _preview_button_enabled(self, button_id):
        button_id = str(button_id or "").upper()
        return self._preview_button_states().get(button_id, False)

    def _set_operator_recognition_view(
        self, payload, status, sample_ready, result_var, mapping_var, row_vars,
    ):
        view = format_operator_recognition_view(
            payload, status=status, sample_ready=sample_ready)
        self.root.after(0, lambda t=view["headline"]: result_var.set(t))
        self.root.after(0, lambda t=view["mapping"]: mapping_var.set(t))
        for index, text in enumerate(view["rows"]):
            self.root.after(0, lambda i=index, t=text: row_vars[i].set(t))

    def _refresh_operator_status(self, candidate_rows=None):
        if not hasattr(self, "traj_operator_status_var"):
            return
        sample_ready = self._drawing_sample_ready()
        top_candidate = self._top_preview_candidate_label()
        interaction_state = getattr(self, "preview_interaction_state", "")
        text = format_operator_action_status(
            interaction_state=interaction_state,
            confirmed_label=getattr(self, "preview_confirmed_label", ""),
            top_candidate_label=top_candidate,
            sample_ready=sample_ready,
        )
        self.root.after(0, lambda t=text: self.traj_operator_status_var.set(t))
        stage = derive_operator_workflow_stage(
            interaction_state=interaction_state,
            has_sample=sample_ready,
            has_candidate=bool(top_candidate),
        )
        workflow = format_operator_workflow(stage, self.interaction_profile)
        self.root.after(0, lambda t=workflow: self.traj_workflow_var.set(t))
        self._set_operator_recognition_view(
            getattr(self, "current_candidate_payload", None),
            interaction_state,
            sample_ready,
            self.traj_operator_result_var,
            self.traj_operator_mapping_var,
            self.traj_operator_candidate_vars,
        )

    def _update_candidate_ui(self):
        status = self.candidate_status
        live_points = len(build_ocr_stroke_points(self.trail))
        raw_points = len(clean_stroke_points(getattr(self, "active_drawing_buffer", [])))
        frozen_points = len(clean_stroke_points(getattr(self, "frozen_drawing_sample", [])))
        published_points = int(getattr(self, "published_sample_point_count", 0))
        backend = getattr(self, "recognition_backend_label", "DTW")
        external_payload = (
            getattr(self, "current_candidate_payload", None)
            if (
                getattr(self, "is_trajectory_mode", False)
                and getattr(self, "confirm_candidate_source", "") == "external_symbol_candidates"
            )
            else None
        )
        display_state = PreviewCandidateDisplayState(
            status=status,
            point_count=live_points,
            backend=backend,
            raw_point_count=raw_points,
            frozen_point_count=frozen_points,
            published_point_count=published_points,
            sample_lifecycle_phase=(
                self._sample_lifecycle_phase()
                if hasattr(self, "_sample_lifecycle_phase") else "idle"
            ),
            published_sample_raw_point_count=int(getattr(self, "published_sample_raw_point_count", 0)),
            board_sample_cleanup_enabled=getattr(self, "board_sample_cleanup_enabled", True),
            result_payload=getattr(self, "current_candidate_payload", None),
            external_candidate_payload=external_payload,
            hover_button=getattr(self, "ocr_hover_button", ""),
            hover_source=getattr(self, "hover_source", "none"),
            dwell_progress=getattr(self, "ocr_dwell_progress", 0.0),
            hover_progress_enabled=getattr(self, "hover_progress_enabled", True),
            preview_interaction_state=getattr(self, "preview_interaction_state", status),
            preview_confirmed_label=getattr(self, "preview_confirmed_label", ""),
            trajectory_candidates=self.trajectory_candidates,
            sample_ready=self._drawing_sample_ready(),
        )
        texts = preview_candidate_ui_texts(display_state)
        recording_txt = self._recording_status_text()
        semantic_txt = self._semantic_preview_text()
        self.root.after(0, lambda t=texts["status"]: self.traj_candidate_status_var.set(t))
        if hasattr(self, "traj_recording_var"):
            self.root.after(0, lambda t=recording_txt: self.traj_recording_var.set(t))
        if hasattr(self, "traj_semantic_var"):
            self.root.after(0, lambda t=semantic_txt: self.traj_semantic_var.set(t))
        if hasattr(self, "traj_backend_var"):
            self.root.after(0, lambda t=texts["backend"]: self.traj_backend_var.set(t))
        if hasattr(self, "traj_recognizer_detail_var"):
            self.root.after(0, lambda t=texts["recognizer_detail"]: self.traj_recognizer_detail_var.set(t))
        if hasattr(self, "traj_candidate_points_var"):
            self.root.after(0, lambda t=texts["points"]: self.traj_candidate_points_var.set(t))
        if hasattr(self, "traj_sample_state_var"):
            self.root.after(0, lambda t=texts["sample"]: self.traj_sample_state_var.set(t))
        if hasattr(self, "traj_sample_cleanup_var"):
            self.root.after(0, lambda t=texts["cleanup"]: self.traj_sample_cleanup_var.set(t))
        if hasattr(self, "traj_result_var"):
            self.root.after(0, lambda t=texts["result"]: self.traj_result_var.set(t))
        if hasattr(self, "traj_candidate_debug_var"):
            debug_text = format_candidate_debug_details(
                getattr(self, "current_candidate_payload", None))
            self.root.after(
                0, lambda t=debug_text: self.traj_candidate_debug_var.set(t))
        if hasattr(self, "traj_operator_result_var"):
            self.root.after(
                0,
                lambda t=texts["operator_result"]: self.traj_operator_result_var.set(t),
            )
        if hasattr(self, "traj_recognition_progress_var"):
            self.root.after(0, lambda t=texts["progress"]: self.traj_recognition_progress_var.set(t))
        if hasattr(self, "traj_interaction_var"):
            self.root.after(0, lambda t=texts["interaction"]: self.traj_interaction_var.set(t))
        if hasattr(self, "traj_confirm_var"):
            self.root.after(0, lambda t=texts["confirm"]: self.traj_confirm_var.set(t))
        if hasattr(self, "traj_operator_status_var"):
            self.root.after(0, lambda t=texts["operator_status"]: self.traj_operator_status_var.set(t))
        for i, txt in enumerate(texts["candidate_rows"]):
            self.root.after(0, lambda i=i, t=txt: self.traj_candidate_vars[i].set(t))
        self._refresh_operator_status(texts["candidate_rows"])

    def _handle_ui_state(self, msg):
        data = safe_json_loads(msg.data)
        if data and "state" in data:
            state = data["state"]
            dwell = coerce_float(data.get("dwell_progress"))
            if dwell is not None:
                text = f"State: {state} (Dwell: {dwell:.1%})"
            else:
                text = f"State: {state}"
            self.root.after(0, lambda: self.ui_state_var.set(text))

    def _handle_capture(self, msg):
        data = safe_json_loads(msg.data)
        if data:
            sid = data.get("sample_id", "unknown")
            self.last_capture_sample_id = data.get("sample_id")
            self.last_capture_sequence_id = data.get("sequence_id")
            path = extract_xy_points(data)
            raw_count = data.get("raw_point_count")
            self._set_captured_sample_display(sid, path, raw_point_count=raw_count)
            self._schedule_ui_refresh()

    def _handle_candidates(self, msg):
        data = safe_json_loads(msg.data)
        if data:
            if self.demo_input_mode == "ocr_canvas" and self.ocr_panel_state == "RECOGNIZING":
                self._cancel_ocr_recognition_timeout()
                self.ocr_panel_state = "CONFIRM_PENDING"
                self.root.after(0, lambda: self.ui_state_var.set("State: CONFIRM_PENDING"))

            self.current_candidate_payload = data
            self._update_real_board_sample_record_candidate(data)
            backend_text = candidate_backend_status_text(data)
            self.root.after(0, lambda t=backend_text: self.backend_status_var.set(t))
            if self.is_trajectory_mode and self.candidate_preview_enabled:
                self._sync_external_candidate_display(data, backend_text)
            recognizer_texts = recognizer_status_texts(data)
            self.root.after(0, lambda t=recognizer_texts["mode"]: self.recognizer_mode_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["model"]: self.recognizer_model_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["status"]: self.recognizer_status_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["labels"]: self.recognizer_labels_var.set(t))

            tops = summarize_candidates(data)
            if self.demo_input_mode == "ocr_canvas":
                summary = ", ".join("%s:%.3f" % (label, confidence) for _rank, label, confidence in tops) or "-"
                self.rospy.loginfo("ocr candidates received top=%s", summary)

            for i, text in enumerate(format_rank_rows(data, count=3)):
                self.root.after(0, lambda i=i, t=text: self.rank_vars[i].set(t))
            self.root.after(0, self._update_ocr_toolbar_visuals)
            if self.demo_input_mode == "ocr_canvas":
                self._refresh_dwell_status()
            self._schedule_ui_refresh()

    def _sync_external_candidate_display(self, data, backend_text):
        tops = summarize_candidates(data)
        self.trajectory_candidates = [
            {"rank": rank, "label": label, "confidence": confidence}
            for rank, label, confidence in tops
        ]
        display_rows = format_candidate_rows_for_display(data, count=3)
        has_display_candidates = any(not row.endswith("-") for row in display_rows)
        accepted = data.get("accepted")
        if tops:
            self.candidate_status = "ready"
            self.preview_interaction_state = "external_candidates_ready"
        elif data.get("uncertain") or accepted is False:
            self.candidate_status = "uncertain"
            self.preview_interaction_state = "external_candidate_uncertain"
        else:
            self.candidate_status = "none"
            self.preview_interaction_state = "external_no_candidate"
        self.root.after(0, lambda: self.traj_preview_mode_var.set(
            "Mode: external DTW trajectory recognition"))
        self.root.after(0, lambda t=backend_text: self.traj_backend_var.set(t))
        self.root.after(0, lambda: self.traj_recognizer_detail_var.set(
            "Feature: trajectory_dtw | Source: /colmag/symbol_candidates"))
        status_text = "Candidate status: external candidates ready"
        if self.candidate_status == "uncertain":
            status_text = "Candidate status: uncertain / redraw"
        elif not tops:
            status_text = "Candidate status: no accepted candidate"
        progress_text = "Progress: external candidates received"
        if self.candidate_status == "uncertain":
            progress_text = "Progress: rejected by DTW gate"
        elif not has_display_candidates:
            progress_text = "Progress: no candidate"
        self.root.after(0, lambda t=status_text: self.traj_candidate_status_var.set(t))
        self.root.after(0, lambda t=format_candidate_result_summary(data): self.traj_result_var.set(t))
        self.root.after(
            0,
            lambda t=format_candidate_debug_details(data): self.traj_candidate_debug_var.set(t),
        )
        self.root.after(0, lambda t=progress_text: self.traj_recognition_progress_var.set(t))
        for i, text in enumerate(display_rows):
            self.root.after(0, lambda i=i, t=text: self.traj_candidate_vars[i].set(t))
        self._refresh_operator_status(display_rows)

    def _ocr_confirm_enabled(self):
        return self.ocr_panel_state == "CONFIRM_PENDING" and bool(summarize_candidates(self.current_candidate_payload))

    def _ocr_state_hint_text(self):
        return format_interaction_instruction(self.interaction_profile)

    def _refresh_mouse_operator_presentation(self):
        if not hasattr(self, "mouse_mode_var"):
            return
        hover_button = str(getattr(self, "ocr_hover_button", "") or "").upper()
        if hover_button in ("B", "C", "A", "X"):
            mode = "HOVER — %s" % hover_button
        elif self.ocr_panel_state == "DRAWING":
            mode = "DRAW"
        else:
            mode = "IDLE"
        tops = summarize_candidates(getattr(self, "current_candidate_payload", None))
        if getattr(self, "last_confirmed_label", ""):
            stage = "act"
        elif self.ocr_panel_state == "WAITING":
            stage = "start"
        else:
            stage = derive_operator_workflow_stage(
                interaction_state=self.ocr_panel_state,
                has_sample=bool(self._collect_ocr_points()),
                has_candidate=bool(tops),
            )
        if self.ocr_panel_state == "BLOCKED":
            status = "Stopped"
        elif self.ocr_panel_state == "CONFIRM_PENDING" and tops:
            status = "Review the result, then choose C / A / X"
        elif self.ocr_panel_state == "RECOGNIZING":
            status = "Wait for recognition"
        elif self.ocr_panel_state == "DRAWING":
            status = "Draw a character in the center."
        elif getattr(self, "last_confirmed_label", ""):
            status = "Confirmed: %s" % self.last_confirmed_label
        else:
            status = "Hover B to start drawing"
        self.root.after(0, lambda t=mode: self.mouse_mode_var.set("Mode: %s" % t))
        self.root.after(
            0,
            lambda t=format_operator_workflow(stage, "mouse"): self.mouse_workflow_var.set(t),
        )
        self.root.after(0, lambda t=status: self.mouse_operator_status_var.set(t))
        self._set_operator_recognition_view(
            getattr(self, "current_candidate_payload", None),
            self.ocr_panel_state,
            False,
            self.mouse_operator_result_var,
            self.mouse_operator_mapping_var,
            self.mouse_operator_candidate_vars,
        )
        debug_text = format_candidate_debug_details(
            getattr(self, "current_candidate_payload", None))
        self.root.after(
            0, lambda t=debug_text: self.mouse_candidate_debug_var.set(t))

    def _preview_pointer_tracking_enabled(self):
        return (
            self.is_trajectory_mode
            and self.candidate_preview_enabled
            and self.hover_buttons_enabled
        )

    def _preview_dwell_active(self):
        """Preview dwell loop should run when either the live board cursor or the
        mouse can drive dwell activation."""
        return (
            self._preview_pointer_tracking_enabled()
            and (self.hover_dwell_enabled or self.board_cursor_dwell_enabled)
        )

    def _preview_button_at_event(self, event):
        if not self._preview_pointer_tracking_enabled():
            return ""
        try:
            x_root = float(event.x_root)
            y_root = float(event.y_root)
            stage_x = x_root - float(self.ocr_stage_card.winfo_rootx())
            stage_y = y_root - float(self.ocr_stage_card.winfo_rooty())
        except (AttributeError, TypeError, ValueError, self.tk.TclError):
            return ""
        return self._resolve_preview_button_at_stage_xy(stage_x, stage_y)

    def _handle_preview_pointer_motion(self, event):
        button_id = self._preview_button_at_event(event)
        if button_id:
            if button_id != self.ocr_hover_button:
                self._set_ocr_hover_button(button_id)
        elif self.ocr_hover_button:
            self._clear_ocr_hover_button()

    def _handle_preview_pointer_leave(self, event):
        if self._preview_pointer_tracking_enabled():
            self._clear_ocr_hover_button()

    def _handle_preview_pointer_click(self, event):
        if not (self._preview_pointer_tracking_enabled() and self.hover_click_enabled):
            return None
        if not getattr(self, "latest_input_active", False):
            return None
        button_id = self._preview_button_at_event(event)
        if not button_id or not self._preview_button_enabled(button_id):
            return None
        self._set_ocr_hover_button(button_id)
        return "break" if self._handle_preview_button_action(button_id) else None

    def _process_ocr_toolbar_dwell(self):
        if self.demo_input_mode == "ocr_canvas":
            self._process_ocr_canvas_dwell()
            self.root.after(50, self._process_ocr_toolbar_dwell)
            return
        if not self._preview_dwell_active():
            return
        self._process_preview_dwell()
        self.root.after(50, self._process_ocr_toolbar_dwell)

    def _process_ocr_canvas_dwell(self):
        """OCR-canvas hover-dwell (mouse only). Unchanged legacy behaviour."""
        now = time.time()
        button_id = self.ocr_hover_button
        dwell_sec = self.dwell_confirm_sec
        enabled = bool(button_id) and (button_id != "C" or self._ocr_confirm_enabled())
        self.hover_source = "mouse" if button_id else "none"
        if not enabled:
            self.ocr_hover_started_at = now if button_id else None
            self.ocr_dwell_progress = 0.0
            self.ocr_dwell_fired = False
        else:
            if self.ocr_hover_started_at is None:
                self.ocr_hover_started_at = now
            self.ocr_dwell_progress = max(0.0, min(1.0, (now - self.ocr_hover_started_at) / max(dwell_sec, 1e-9)))
            if self.ocr_dwell_progress >= 1.0 and not self.ocr_dwell_fired:
                self.ocr_dwell_fired = True
                self._handle_ocr_action(button_id)
        self.dwell_state = {
            "current_button_id": button_id if enabled else "",
            "started_at": self.ocr_hover_started_at,
            "dwell_progress": self.ocr_dwell_progress if enabled else 0.0,
            "activated_button_id": "",
        }
        self._refresh_dwell_status()
        self._update_ocr_toolbar_visuals()
        self._publish_dashboard_state()

    def _process_preview_dwell(self):
        """Preview dwell driven by the live board cursor first, mouse as fallback.

        The red current point is the primary interaction cursor: when it enters a
        button hit zone dwell starts, and after ``control_dwell_sec`` the same safe
        preview handler that a click uses is fired. Board hover is preferred over
        mouse hover so the demo works without a mouse; the choice is deterministic
        (board wins ties). One trigger per dwell entry: ``ocr_dwell_fired`` is only
        reset when the effective button changes (i.e. after leaving the zone).
        """
        now = time.time()
        if not getattr(self, "latest_input_active", True):
            self.board_hover_button = ""
            self.hover_source = "none"
            self.ocr_hover_button = ""
            self._dwell_active_button = ""
            self.ocr_hover_started_at = None
            self.ocr_dwell_progress = 0.0
            self.ocr_dwell_fired = False
            self.dwell_state = {
                "current_button_id": "",
                "started_at": None,
                "dwell_progress": 0.0,
                "activated_button_id": "",
            }
            self._refresh_dwell_status()
            self._update_ocr_toolbar_visuals()
            self._update_orientation_clutch_mode("")
            self._schedule_ui_refresh()
            self._publish_dashboard_state()
            return
        board_button = self._board_cursor_preview_button() if self.board_cursor_dwell_enabled else ""
        mouse_button = self.mouse_hover_button if self.hover_dwell_enabled else ""
        self.board_hover_button = board_button
        # Deterministic: board cursor is primary in the demo, mouse is the fallback.
        if board_button:
            button_id, source = board_button, "board"
        elif mouse_button:
            button_id, source = mouse_button, "mouse"
        else:
            button_id, source = "", "none"
        self.hover_source = source
        self.ocr_hover_button = button_id  # effective button drives visuals/status
        self._update_orientation_clutch_mode(button_id)

        enabled = bool(button_id) and self._preview_button_enabled(button_id)

        # Reset dwell timing only when the effective button changes, so staying on
        # one button does not re-trigger and leaving/re-entering re-arms it.
        if button_id != self._dwell_active_button:
            self._dwell_active_button = button_id
            self.ocr_hover_started_at = now if button_id else None
            self.ocr_dwell_progress = 0.0
            self.ocr_dwell_fired = False

        if not enabled:
            self.ocr_dwell_progress = 0.0
            if not button_id:
                self.ocr_hover_started_at = None
        else:
            if self.ocr_hover_started_at is None:
                self.ocr_hover_started_at = now
            self.ocr_dwell_progress = max(0.0, min(
                1.0, (now - self.ocr_hover_started_at) / max(self.control_dwell_sec, 1e-9)))
            if self.ocr_dwell_progress >= 1.0 and not self.ocr_dwell_fired:
                self.ocr_dwell_fired = True
                self._handle_preview_button_action(button_id)
        self.dwell_state = {
            "current_button_id": button_id if enabled else "",
            "started_at": self.ocr_hover_started_at,
            "dwell_progress": self.ocr_dwell_progress if enabled else 0.0,
            "activated_button_id": button_id if (enabled and self.ocr_dwell_fired) else "",
        }
        self._refresh_dwell_status()
        self._update_ocr_toolbar_visuals()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _set_ocr_hover_button(self, button_id):
        button_id = str(button_id or "").upper()
        if self.demo_input_mode != "ocr_canvas":
            # Preview mode: record the raw MOUSE hover only. The dwell loop decides
            # the effective button (board cursor first, this mouse hover as
            # fallback), so mouse dwell timing is owned by the loop, not here.
            self.mouse_hover_button = button_id
            return
        if button_id == self.ocr_hover_button:
            return
        self.ocr_hover_button = button_id
        self.ocr_hover_started_at = time.time()
        self.ocr_dwell_progress = 0.0
        self.ocr_dwell_fired = False
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()

    def _clear_ocr_hover_button(self, button_id=None):
        if self.demo_input_mode != "ocr_canvas":
            if button_id is not None and str(button_id or "").upper() != self.mouse_hover_button:
                return
            self.mouse_hover_button = ""
            return
        if button_id is not None and str(button_id or "").upper() != self.ocr_hover_button:
            return
        self.ocr_hover_button = ""
        self.ocr_hover_started_at = None
        self.ocr_dwell_progress = 0.0
        self.ocr_dwell_fired = False
        self.dwell_state = cancelled_preview_hover_state()
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()

    def _update_ocr_toolbar_visuals(self):
        for button_id, widgets in self.ocr_button_widgets.items():
            canvas = widgets["canvas"]
            base_text = widgets["base_text"]
            size = widgets.get("size", 82)
            pad = 2

            fill = "#F8FAFC"
            outline = "#94A3B8"
            text_fg = "#1F2937"

            if self.demo_input_mode == "ocr_canvas":
                enabled = button_id != "C" or self._ocr_confirm_enabled()
                active = button_id == self.ocr_hover_button
                style = preview_button_style(button_id, enabled, active=active)
                fill = style["fill"]
                outline = style["outline"]
                text_fg = style["text"]
            elif self.is_trajectory_mode and self.candidate_preview_enabled:
                enabled = self._preview_button_enabled(button_id)
                active = button_id == self.ocr_hover_button
                style = preview_button_style(button_id, enabled, active=active)
                fill = style["fill"]
                outline = style["outline"]
                text_fg = style["text"]
            else:
                if button_id == "X": fill = "#FFD700"
                elif button_id == "A": fill = "#FF6B6B"
                elif button_id == "C": fill = "#4CAF50"
                elif button_id == "B": fill = "#87CEEB"
                elif button_id == "CLEAR": fill = "#DDDDDD"
                outline = "black"

            canvas.delete("all")
            canvas.create_oval(pad, pad, size-pad, size-pad, fill=fill, outline=outline, width=2)
            canvas.create_text(size/2, size/2, text=base_text, font=("TkDefaultFont", max(8, int(size*0.12)), "bold"), justify=self.tk.CENTER, fill=text_fg)

            if (
                (self.demo_input_mode == "ocr_canvas" or (self.is_trajectory_mode and self.candidate_preview_enabled))
                and button_id == self.ocr_hover_button
                and self.ocr_dwell_progress > 0
                and self.hover_progress_enabled
            ):
                pct = self.ocr_dwell_progress
                arc_extent = -360 * pct
                canvas.create_arc(
                    pad+2, pad+2, size-pad-2, size-pad-2,
                    start=90, extent=arc_extent, style=self.tk.ARC,
                    outline=style["ring"], width=4)

    def _handle_ocr_action(self, button_id):
        button_id = str(button_id or "").upper()
        if button_id == "B":
            if should_publish_mouse_toolbar_stop(self.ocr_panel_state):
                self._start_ocr_drawing("Drawing mode started")
        elif button_id == "C":
            if not self._ocr_confirm_enabled():
                return
            self._cancel_ocr_auto_recognize()
            self._cancel_ocr_recognition_timeout()
            self._confirm_rank_by_dwell(1)
            self.ocr_panel_state = "WAITING"
            self.all_strokes = []
            self.drawing_stroke = []
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Confirmed)"))
        elif button_id == "X":
            self._cancel_ocr_auto_recognize()
            self._cancel_ocr_recognition_timeout()
            self.ocr_panel_state = "WAITING"
            self.captured_path = []
            self.all_strokes = []
            self.drawing_stroke = []
            self._clear_candidates("Cleared canvas")
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Cleared)"))
        elif button_id == "A":
            if self.ocr_panel_state != "BLOCKED":
                self._cancel_ocr_auto_recognize()
                self._cancel_ocr_recognition_timeout()
                self.ocr_panel_state = "BLOCKED"
                self._publish_ocr_stop_label()
            self.root.after(0, lambda: self.ui_state_var.set("State: BLOCKED (STOP)"))
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _start_ocr_drawing(self, message):
        self._cancel_ocr_auto_recognize()
        self._cancel_ocr_recognition_timeout()
        self.ocr_panel_state = "DRAWING"
        self.captured_path = []
        self.all_strokes = []
        self.drawing_stroke = []
        self.ocr_last_stroke_time = None
        self._clear_candidates(message)
        self.rospy.loginfo("ocr drawing started state=%s", self.ocr_panel_state)
        self.root.after(0, lambda: self.ui_state_var.set("State: DRAWING"))
        self.root.after(0, lambda: self.capture_var.set("Draw a symbol, then release"))

    def _collect_ocr_points(self):
        return _dashboard_points.flatten_strokes(self.all_strokes)

    def _cancel_ocr_auto_recognize(self):
        if self.ocr_auto_recognize_job is None:
            return
        try:
            self.root.after_cancel(self.ocr_auto_recognize_job)
        except self.tk.TclError:
            pass
        self.ocr_auto_recognize_job = None

    def _cancel_ocr_recognition_timeout(self):
        if self.ocr_recognition_timeout_job is None:
            return
        try:
            self.root.after_cancel(self.ocr_recognition_timeout_job)
        except self.tk.TclError:
            pass
        self.ocr_recognition_timeout_job = None

    def _schedule_ocr_auto_recognize(self):
        self._cancel_ocr_auto_recognize()
        points = self._collect_ocr_points()
        if len(points) < 2:
            self.root.after(0, lambda: self.capture_var.set("Draw a symbol before recognizing"))
            return
        self.ocr_last_stroke_time = time.time()
        self.ocr_auto_recognize_job = self.root.after(
            self.ocr_auto_recognize_after_ms,
            self._trigger_ocr_recognition_if_ready,
        )
        self.root.after(0, lambda: self.capture_var.set("Recognition will start automatically"))

    def _trigger_ocr_recognition_if_ready(self):
        self.ocr_auto_recognize_job = None
        if self.ocr_panel_state != "DRAWING":
            return
        points = self._collect_ocr_points()
        if len(points) < 2:
            self.root.after(0, lambda: self.capture_var.set("Draw a symbol before recognizing"))
            return
        self.ocr_panel_state = "RECOGNIZING"
        payload_points = [[x, y] for x, y in points]
        payload = {
            "points": payload_points,
            "sample_id": "ocr_canvas_%d" % int(time.time()),
            "sequence_id": 0,
        }
        self.symbol_capture_pub.publish(self.String(data=json.dumps(payload)))
        self.rospy.loginfo(
            "ocr capture published sample_id=%s point_count=%d state=%s",
            payload["sample_id"],
            len(payload_points),
            self.ocr_panel_state,
        )
        self._cancel_ocr_recognition_timeout()
        self.ocr_recognition_timeout_job = self.root.after(
            self.ocr_recognition_timeout_ms,
            self._handle_ocr_recognition_timeout,
        )
        self.root.after(0, lambda: self.ui_state_var.set("State: RECOGNIZING"))
        self.root.after(0, lambda: self.capture_var.set(
            f"Sent {len(points)} points for recognition"))
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _handle_ocr_recognition_timeout(self):
        self.ocr_recognition_timeout_job = None
        if self.ocr_panel_state != "RECOGNIZING":
            return
        self.ocr_panel_state = "DRAWING"
        self.rospy.logwarn("ocr recognition timeout: no candidates received")
        self.root.after(0, lambda: self.ui_state_var.set("State: DRAWING (No recognizer response)"))
        self.root.after(0, lambda: self.capture_var.set("No recognizer response"))
        self._update_ocr_toolbar_visuals()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _handle_gamepad_dwell_activation(self, button_id):
        button_id = str(button_id or "").upper()
        now = time.time()
        has_candidates = bool(summarize_candidates(self.current_candidate_payload))

        if button_id == "X":
            self.controller_mode = GAMEPAD_MODE_MOTION
            self.digit_mode_expires_at = None
            self._clear_candidates("Cleared/unblocked by X dwell")
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._publish_dashboard_state()
            return

        if button_id == "B":
            transition = update_controller_mode(self.controller_mode, "B", now, self.digit_mode_sec, has_candidates=has_candidates)
            self.controller_mode = transition["mode"]
            self.digit_mode_expires_at = transition["digit_expires_at"]
            self.root.after(0, lambda: self.confirm_var.set("Digit mode armed by B dwell"))
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._refresh_dwell_status()
            self._publish_dashboard_state()
            return

        if button_id == "C":
            if self.controller_mode != GAMEPAD_MODE_CONFIRM_PENDING:
                return
            self._confirm_rank_by_gamepad_c()
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            return

        if button_id in GAMEPAD_ARROW_BUTTONS:
            if not self.gamepad_publish_motion_buttons:
                return
            self._publish_gamepad_button_label(button_id)
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            return

        if button_id == "A":
            self._publish_gamepad_button_label(button_id)
            self.controller_mode = GAMEPAD_MODE_BLOCKED
            self.digit_mode_expires_at = None
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._publish_dashboard_state()

    def _clear_candidates(self, message):
        self.current_candidate_payload = None
        self.last_confirmed_label = ""
        self.last_selected_rank = None
        self.last_command_intent = ""
        self.root.after(0, lambda: self.confirm_var.set(message))
        for i in range(3):
            self.root.after(0, lambda i=i: self.rank_vars[i].set("Rank %d: -" % (i + 1)))
        self.root.after(0, self._update_ocr_toolbar_visuals)

    def _update_digit_mode_timeout(self, now):
        if self.controller_layout != CONTROLLER_LAYOUT_GAMEPAD:
            return
        if self.controller_mode == GAMEPAD_MODE_DIGIT and self.digit_mode_expires_at is not None and now >= self.digit_mode_expires_at:
            self.controller_mode = GAMEPAD_MODE_CONFIRM_PENDING
            self.digit_mode_expires_at = None
            self.root.after(0, lambda: self.confirm_var.set("Digit window ended; C confirms rank 1"))

    def _publish_gamepad_button_label(self, button_id):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        confirmed, reason = build_gamepad_confirmed_label_payload(button_id, self.controller_mode)
        if confirmed is None:
            self.rospy.logwarn("gamepad dwell confirm skipped: %s", reason)
            return

        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = None
        self.last_command_intent = str(confirmed.get("command_intent", ""))
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Gamepad: %s -> %s" % (button_id, self.last_command_intent)))
        self._publish_dashboard_state()
        self.rospy.loginfo("gamepad dwell confirmed_label=%s", msg.data)

    def _publish_ocr_stop_label(self):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        confirmed = build_mouse_toolbar_stop_payload(time.time())
        self.last_confirmed_label = "X"
        self.last_selected_rank = None
        self.last_command_intent = "STOP_OR_CANCEL"
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Mouse toolbar: A -> STOP_OR_CANCEL"))
        self._publish_dashboard_state()
        self.rospy.loginfo("mouse toolbar STOP_OR_CANCEL confirmed_label=%s", msg.data)

    def _confirm_rank_by_gamepad_c(self):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        payload = self.current_candidate_payload
        if should_suppress_repeated_confirm(self.last_confirm_key, payload, 1):
            return

        confirmed, reason = build_gamepad_confirmed_label_payload("C", self.controller_mode, payload)
        if confirmed is None:
            self.rospy.logwarn("gamepad C confirm skipped: %s", reason)
            return

        self.last_confirm_key = "%s:rank_1" % candidate_payload_key(payload)
        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = 1
        self.last_command_intent = str(confirmed.get("command_intent", ""))
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Confirmed rank 1 '%s' by C dwell" % self.last_confirmed_label))
        self._publish_dashboard_state()
        self.rospy.loginfo("gamepad C confirmed_label=%s", msg.data)

    def _confirm_rank_by_dwell(self, selected_rank):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        payload = self.current_candidate_payload
        if should_suppress_repeated_confirm(self.last_confirm_key, payload, selected_rank):
            return

        confirmed, reason = build_confirmed_label_payload(payload, selected_rank)
        if confirmed is None:
            self.rospy.logwarn("dashboard dwell confirm skipped: %s", reason)
            return

        self.last_confirm_key = "%s:rank_%d" % (candidate_payload_key(payload), int(selected_rank))
        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = int(selected_rank)
        self.last_command_intent = "CONFIRM_RANK_%d" % int(selected_rank)
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Confirmed: '%s' by dashboard dwell" % self.last_confirmed_label))
        self._publish_dashboard_state()
        self.rospy.loginfo("dashboard dwell confirmed_label=%s", msg.data)

    def _refresh_dwell_status(self):
        if self.demo_input_mode == "ocr_canvas":
            texts = format_ocr_canvas_dwell_texts(
                self.ocr_panel_state,
                self.ocr_hover_button,
                self.ocr_dwell_progress,
                self.dwell_confirm_sec,
            )
            self.root.after(0, lambda t=texts["dwell"]: self.dwell_var.set(t))
            self.root.after(0, lambda t=texts["controller"]: self.controller_mode_var.set(t))
            self.root.after(0, lambda t=texts["hover"]: self.hover_var.set(t))
            self.root.after(0, lambda: self.ocr_hint_var.set(self._ocr_state_hint_text()))
            self._refresh_mouse_operator_presentation()
            return

        if self.is_trajectory_mode and self.candidate_preview_enabled:
            text = format_trajectory_preview_interaction_text(
                self.preview_interaction_state,
                getattr(self, "hover_source", "none"),
                self.ocr_hover_button,
                self.ocr_dwell_progress,
                hover_progress_enabled=self.hover_progress_enabled,
            )
            self.root.after(0, lambda t=text: self.traj_interaction_var.set(t))
            if self.hover_buttons_enabled:
                hint = format_interaction_instruction(self.interaction_profile)
                self.root.after(0, lambda t=hint: self.ocr_hint_var.set(t))
            return

        texts = format_integrated_dwell_status_texts(
            self.integrated_confirm_enabled,
            self.controller_layout,
            self.dwell_state.get("current_button_id", ""),
            self.dwell_state.get("dwell_progress", 0.0),
            self.dwell_confirm_sec,
            self.controller_mode,
            self.last_command_intent,
            gamepad_layout=CONTROLLER_LAYOUT_GAMEPAD,
        )
        self.root.after(0, lambda t=texts["dwell"]: self.dwell_var.set(t))
        self.root.after(0, lambda t=texts["controller"]: self.controller_mode_var.set(t))
        self.root.after(0, lambda t=texts["hover"]: self.hover_var.set(t))

    def _publish_dashboard_state(self):
        if not self.integrated_confirm_enabled or self.dashboard_state_pub is None:
            return
        if self.demo_input_mode == "ocr_canvas":
            current_button = self.ocr_hover_button if self.ocr_hover_button != "C" or self._ocr_confirm_enabled() else ""
            payload = {
                "timestamp": time.time(),
                "mode": "ocr_canvas_dashboard",
                "ocr_panel_state": self.ocr_panel_state,
                "current_virtual_button": current_button,
                "dwell_progress": self.ocr_dwell_progress if current_button else 0.0,
                "integrated_confirm_enabled": True,
                "last_confirmed_label": self.last_confirmed_label,
                "last_selected_rank": self.last_selected_rank,
                "last_command_intent": self.last_command_intent,
                "controls_real_robot": False,
                "gazebo_only": True,
                "placeholder_only": True,
            }
            self.dashboard_state_pub.publish(self.String(data=json.dumps(payload, sort_keys=True)))
            return

        payload = {
            "timestamp": time.time(),
            "mode": "integrated_dashboard",
            "controller_layout": self.controller_layout,
            "controller_mode": self.controller_mode if self.controller_layout == CONTROLLER_LAYOUT_GAMEPAD else "",
            "current_virtual_button": self.dwell_state.get("current_button_id", ""),
            "dwell_progress": self.dwell_state.get("dwell_progress", 0.0),
            "integrated_confirm_enabled": True,
            "last_confirmed_label": self.last_confirmed_label,
            "last_selected_rank": self.last_selected_rank,
            "last_command_intent": self.last_command_intent,
            "controls_real_robot": False,
            "gazebo_only": True,
            "placeholder_only": True,
        }
        self.dashboard_state_pub.publish(self.String(data=json.dumps(payload, sort_keys=True)))

    def _handle_confirmed(self, msg):
        data = safe_json_loads(msg.data)
        if data:
            lbl = data.get("label", "-")
            src = data.get("confirmed_by", "-")
            text = f"Confirmed: '{lbl}' by {src}"
            self.root.after(0, lambda: self.confirm_var.set(text))

    def _request_control_mode_switch(self):
        if self.control_mode_request_pub is None:
            return
        if not self.control_mode_observed:
            requested = control_mode.TASK
        else:
            requested = (
                control_mode.TELEOP
                if self.current_control_mode == control_mode.TASK
                else control_mode.TASK
            )
        self.control_mode_request_pub.publish(self.String(data=requested))
        self.rospy.loginfo("requested control mode %s", requested)

    def _handle_control_mode(self, msg):
        selected = control_mode.normalize_mode(msg.data)
        if selected is None:
            self.current_control_mode = None
            self.control_mode_observed = False
            self.root.after(0, lambda: self.control_mode_var.set("Mode: INVALID — blocked"))
            self.root.after(0, lambda: self.control_mode_button_var.set("Request TASK"))
            return
        self.current_control_mode = selected
        self.control_mode_observed = True
        next_mode = (
            control_mode.TELEOP
            if selected == control_mode.TASK
            else control_mode.TASK
        )
        self.root.after(0, lambda m=selected: self.control_mode_var.set("Mode: %s" % m))
        self.root.after(
            0,
            lambda m=next_mode: self.control_mode_button_var.set("Switch to %s" % m),
        )

    def _handle_task(self, msg):
        data = safe_json_loads(msg.data)
        if data:
            task = data.get("task", "-")
            self.root.after(0, lambda: self.task_var.set(f"Command: {task}"))

    def _handle_gazebo(self, msg):
        data = safe_json_loads(msg.data)
        if data:
            phase = data.get("phase", "-")
            status = data.get("status", "-")
            text = f"Gazebo: {status} [{phase}]"
            self.root.after(0, lambda: self.gazebo_var.set(text))

    def _close(self):
        self.rospy.signal_shutdown("dashboard closed")
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass

    def run(self):
        self.rospy.on_shutdown(self._on_ros_shutdown)
        self.root.mainloop()

    def _on_ros_shutdown(self):
        try:
            self.root.after(0, self.root.quit)
        except self.tk.TclError:
            pass

    def _is_in_draw_area(self, x, y):
        if self.demo_input_mode != "ocr_canvas" or not self._ocr_draw_area:
            return True
        x0, y0, x1, y1 = self._ocr_draw_area
        return x0 <= x <= x1 and y0 <= y <= y1

    def _on_canvas_press(self, event):
        if self.ocr_panel_state != "DRAWING":
            return
        if not self._is_in_draw_area(event.x, event.y):
            return
        self._cancel_ocr_auto_recognize()
        self.is_drawing = True
        world_x, world_y = map_canvas_to_world(event.x, event.y, self.bounds, self.canvas_width, self.canvas_height, self.padding)
        self.drawing_stroke = [(world_x, world_y)]
        self.rospy.loginfo("ocr stroke started point_count=1")
        self._schedule_ui_refresh()

    def _on_canvas_motion(self, event):
        if not self.is_drawing:
            return
        if not self._is_in_draw_area(event.x, event.y):
            return
        world_x, world_y = map_canvas_to_world(event.x, event.y, self.bounds, self.canvas_width, self.canvas_height, self.padding)
        if self.drawing_stroke:
            last_x, last_y = self.drawing_stroke[-1]
            if math.hypot(world_x - last_x, world_y - last_y) > 0.002:
                self.drawing_stroke.append((world_x, world_y))
                if len(self.drawing_stroke) == 2 or len(self.drawing_stroke) % 10 == 0:
                    self.rospy.loginfo("ocr stroke point added point_count=%d", len(self.drawing_stroke))
                self._schedule_ui_refresh()

    def _on_canvas_release(self, event):
        if not self.is_drawing:
            return
        self.is_drawing = False
        if len(self.drawing_stroke) > 1:
            self.all_strokes.append(self.drawing_stroke)
            self.rospy.loginfo("ocr stroke released stroke_points=%d total_points=%d", len(self.drawing_stroke), len(self._collect_ocr_points()))
        self.drawing_stroke = []
        self._schedule_ocr_auto_recognize()
        self._schedule_ui_refresh()

    def _on_key_press(self, event):
        key = getattr(event, "char", "").lower()
        keysym = getattr(event, "keysym", "").lower()
        if self.is_trajectory_mode:
            # Preview-only Start: record one complete segment (never publishes).
            if getattr(self, "trajectory_recording_enabled", False) and key == "s":
                self._start_recording()
                return
            # Recognition triggers. External DTW route publishes the captured
            # trajectory sample to the existing /colmag/symbol_capture seam.
            if self.candidate_preview_enabled:
                if not getattr(self, "latest_input_active", False):
                    return
                if key in ("r", "b"):
                    self._handle_preview_button_action("B")
                elif key == "c":
                    self._handle_preview_button_action("C")
                elif key in ("a", "x"):
                    self._handle_preview_button_action(key)
                elif keysym in ("return", "kp_enter"):
                    self._handle_preview_button_action("C")
            return
        if self.demo_input_mode != "ocr_canvas":
            return
        if key in ("b", "c", "x", "a"):
            self._handle_ocr_action(key)

if __name__ == "__main__":
    import rospy
    rospy.init_node("magnetic_trajectory_dashboard_node")
    MagneticTrajectoryDashboardNode().run()
