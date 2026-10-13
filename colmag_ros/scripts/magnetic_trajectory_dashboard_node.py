#!/usr/bin/env python3
import json
import sys
import time
from collections import deque
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros.trajectory_segment_recorder import SegmentRecorder
from colmag_ros.dashboard_confirm_publisher import PreviewConfirmPublisher
from colmag_ros.dashboard_candidate_display import (
    backend_display_name,
    candidate_backend_status_text,
    format_operator_workflow,
    format_rank_rows,
    recognizer_status_texts,
    summarize_candidates,
)
from colmag_ros import control_mode
from colmag_ros.dashboard_controller_input import (
    CONTROLLER_LAYOUT_GAMEPAD,
    CONTROLLER_LAYOUT_RANK_CONFIRM,
    GAMEPAD_MODE_MOTION,
    build_confirmed_label_payload,
    build_controller_button_rects,
    candidate_payload_key,
    normalize_controller_layout,
    should_suppress_repeated_confirm,
    update_dwell_state,
)
from colmag_ros.dashboard_geometry import (
    BOARD_CONTROL_LAYOUT_EXTERNAL,
    dashboard_minimum_window_size,
    normalize_board_control_layout,
)
from colmag_ros.dashboard_points import (
    extract_single_point,
    extract_xy_points,
)
from colmag_ros.dashboard_sample_lifecycle import (
    DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR,
    real_board_sample_recording_dir,
)
from colmag_ros.dashboard_trajectory_processing import (
    normalize_scale_mode,
    normalize_stroke_gate_mode,
)
from colmag_ros.dashboard_dwell_status import (
    format_interaction_instruction,
    format_integrated_dwell_status_texts,
    format_ocr_canvas_dwell_texts,
    format_trajectory_preview_interaction_text,
    normalize_interaction_profile,
)
from colmag_ros.dashboard_texts import (
    TRAJECTORY_MODE,
    default_footer_text,
    default_safety_badge,
)
from colmag_ros.param_utils import bool_param, optional_float
from colmag_ros.preview_recognition import RECOGNIZER_BACKENDS, recognize_stroke
from colmag_ros.dashboard_layout import DashboardLayoutMixin
from colmag_ros.dashboard_canvas_view import DashboardCanvasViewMixin
from colmag_ros.dashboard_sample_capture import DashboardSampleCaptureMixin
from colmag_ros.dashboard_preview_controls import DashboardPreviewControlsMixin
from colmag_ros.dashboard_ocr_panel import DashboardOcrPanelMixin
from colmag_ros.dashboard_gamepad import DashboardGamepadMixin


def safe_json_loads(text):
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data
    except (ValueError, TypeError):
        pass
    return None


def coerce_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class MagneticTrajectoryDashboardNode(
    DashboardLayoutMixin,
    DashboardCanvasViewMixin,
    DashboardSampleCaptureMixin,
    DashboardPreviewControlsMixin,
    DashboardOcrPanelMixin,
    DashboardGamepadMixin,
):
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
        #     behavior unchanged.
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
        self.gate_z_min = optional_float(rospy.get_param("~trajectory_z_min", None))
        self.gate_z_max = optional_float(rospy.get_param("~trajectory_z_max", None))
        self.input_valid_gate_enabled = bool_param(rospy.get_param("~trajectory_input_valid_gate_enabled", True))
        raw_input_z_min = rospy.get_param("~trajectory_input_z_min", "inherit")
        raw_input_z_max = rospy.get_param("~trajectory_input_z_max", "inherit")
        self.input_z_min = self.gate_z_min if str(raw_input_z_min).strip().lower() == "inherit" else optional_float(raw_input_z_min)
        self.input_z_max = self.gate_z_max if str(raw_input_z_max).strip().lower() == "inherit" else optional_float(raw_input_z_max)
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
        # Four preview-only hover/click buttons.
        self.hover_buttons_enabled = bool_param(rospy.get_param("~hover_buttons_enabled", True))
        self.hover_click_enabled = bool_param(rospy.get_param("~hover_click_enabled", True))
        self.hover_dwell_enabled = bool_param(rospy.get_param("~hover_dwell_enabled", False))
        self.control_dwell_sec = float(rospy.get_param(
            "~dashboard_control_dwell_sec",
            rospy.get_param("~hover_dwell_sec", 1.0),
        ))
        self.hover_progress_enabled = bool_param(rospy.get_param("~hover_progress_enabled", True))
        # The live board's red current point can drive button dwell without
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

        # Preview-only segment recording. When enabled, Start records ONE
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
        # Preview-only semantic library (display only; no dispatch).
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
        self._redraw_pending = False
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

        # Candidate preview (display only).
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

    def _schedule_ui_refresh(self):
        # Samples can arrive faster than Tk draws; one pending redraw covers them all.
        if self._redraw_pending:
            return
        self._redraw_pending = True
        self.root.after(0, self._redraw_canvas)

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
        self._schedule_ui_refresh()

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

    def _clear_candidates(self, message):
        self.current_candidate_payload = None
        self.last_confirmed_label = ""
        self.last_selected_rank = None
        self.last_command_intent = ""
        self.root.after(0, lambda: self.confirm_var.set(message))
        for i in range(3):
            self.root.after(0, lambda i=i: self.rank_vars[i].set("Rank %d: -" % (i + 1)))
        self.root.after(0, self._update_ocr_toolbar_visuals)

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
