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
from colmag_ros.dashboard_config import DashboardConfig
from colmag_ros.dashboard_dwell_timer import DwellTimer
from colmag_ros.dashboard_controller_input import (
    build_confirmed_label_payload,
    candidate_payload_key,
    should_suppress_repeated_confirm,
    update_dwell_state,
)
from colmag_ros.dashboard_geometry import dashboard_minimum_window_size
from colmag_ros.dashboard_points import (
    extract_single_point,
    extract_xy_points,
)
from colmag_ros.dashboard_dwell_status import (
    format_interaction_instruction,
    format_integrated_dwell_status_texts,
    format_ocr_canvas_dwell_texts,
    format_trajectory_preview_interaction_text,
)
from colmag_ros.preview_interaction import PreviewInteraction
from colmag_ros.preview_recognition import recognize_stroke
from colmag_ros.dashboard_layout import DashboardLayoutMixin
from colmag_ros.dashboard_canvas_view import DashboardCanvasViewMixin
from colmag_ros.dashboard_sample_capture import DashboardSampleCaptureMixin
from colmag_ros.dashboard_preview_controls import DashboardPreviewControlsMixin
from colmag_ros.dashboard_ocr_panel import DashboardOcrPanelMixin


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
):
    def __init__(self):
        self._initialize_runtime_dependencies()
        self.config = DashboardConfig.from_params(self.rospy.get_param)
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

    def _initialize_dashboard_state(self):
        config = self.config
        # The layout resizes the canvas; it starts at the configured size.
        self.canvas_width = config.canvas_width
        self.canvas_height = config.canvas_height
        self._active_bounds = config.bounds
        self.latest_point = None
        self.latest_valid = True
        self.latest_writing = True
        self.latest_input_active = True
        self.latest_input_inactive_reason = ""
        self.latest_inside_drawing_zone = False
        self.latest_inside_control_zone = False
        self.trajectory_sample_count = 0
        self.latest_sample_index = None
        self._input_active_streak = 0
        self._prev_sample = None  # (x, y, timestamp) for speed gating
        self.current_control_mode = control_mode.TASK
        self.control_mode_observed = False

        trail_maxlen = config.traj_trace_window_points if config.is_trajectory_mode else config.trail_size
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
        if config.demo_input_mode == "ocr_canvas":
            self._ocr_draw_area = (10, 24, self.canvas_width - 10, self.canvas_height - 10)
        else:
            self._ocr_draw_area = None

        self.preview_interaction = PreviewInteraction()
        self.recognition_backend_label = backend_display_name(
            config.recognition_backend,
            config.recognition_model_path,
        )
        self.trajectory_candidates = []
        self._candidate_recognize_job = None
        self._recognizing = False
        self.segment_recorder = SegmentRecorder(
            duration_sec=config.record_duration_sec,
            stop_on_lift=config.record_stop_on_lift,
            lift_z_threshold=config.record_lift_z_threshold,
            min_points=config.record_min_points,
            max_points=config.record_max_points,
        )
        self.recorded_segment = []

        self.dwell_state = update_dwell_state({}, "", time.time(), config.dwell_confirm_sec)
        self.ocr_button_widgets = {}
        self.preview_button_hit_zones = []
        self.preview_canvas_button_zones = []
        self.preview_canvas_drawing_zone = None
        # ``dwell_timer.button`` is the EFFECTIVE hovered button that drives
        # visuals, status and dwell. In ocr_canvas mode it is the mouse hover
        # directly. In trajectory preview mode it is recomputed each dwell tick as
        # board-cursor first, then mouse fallback (``mouse_hover_button``).
        # ``hover_source`` names which one is active.
        self.dwell_timer = DwellTimer()
        self.mouse_hover_button = ""
        self.hover_source = "none"
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
        self.hover_var = tk.StringVar(value="Hover: -")
        if self.config.is_trajectory_mode and self.config.candidate_preview_enabled:
            initial_hint = format_interaction_instruction(self.config.interaction_profile)
        elif self.config.is_trajectory_mode:
            initial_hint = "Live preview — not affine-calibrated"
        else:
            initial_hint = "WAITING | Hover B to start drawing"
        self.ocr_hint_var = tk.StringVar(value=initial_hint)
        self.confirm_var = tk.StringVar(value="-")
        self.task_var = tk.StringVar(value="-")
        self.gazebo_var = tk.StringVar(value="-")

        # Trajectory-preview status (trajectory mode only).
        if self.config.traj_scale_mode == "auto":
            _scale_label = "auto-fit recent (diagnostic)"
        elif self.config.traj_scale_mode == "fixed_symmetric":
            _scale_label = "fixed symmetric, extent %.3g" % self.config.traj_projection_extent
        else:
            _scale_label = "fixed-range window"
        self.traj_mode_var = tk.StringVar(value="Scale: %s, uniform" % _scale_label)
        self.traj_window_var = tk.StringVar(value="Trace window: %d points" % self.config.traj_trace_window_points)
        self.traj_gate_var = tk.StringVar(value="Stroke gate: %s" % self.config.stroke_gate_mode)
        self.traj_topic_var = tk.StringVar(value="Topic: %s" % self.config.trajectory_topic)
        self.traj_valid_var = tk.StringVar(value="Valid: -")
        self.traj_xyz_var = tk.StringVar(value="x/y/z: - / - / -")
        self.traj_count_var = tk.StringVar(value="Samples: 0")
        self.traj_filter_var = tk.StringVar(value="Filter: -")
        self.traj_input_gate_var = tk.StringVar(
            value="Input: active - | drawing - | control - | reason -")
        self.traj_clutch_mode_var = tk.StringVar(value="Mode: NAVIGATE")
        self.traj_workflow_var = tk.StringVar(
            value=format_operator_workflow("draw", self.config.interaction_profile))
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
            "press S/Start to record a segment" if self.config.trajectory_recording_enabled else "disabled"))
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
        rospy.Subscriber(self.config.trajectory_topic, String, self._handle_trajectory, queue_size=10)
        rospy.Subscriber(self.config.ui_state_topic, String, self._handle_ui_state, queue_size=10)
        rospy.Subscriber(self.config.symbol_capture_topic, String, self._handle_capture, queue_size=10)
        rospy.Subscriber(self.config.symbol_candidates_topic, String, self._handle_candidates, queue_size=10)
        rospy.Subscriber(self.config.confirmed_label_topic, String, self._handle_confirmed, queue_size=10)
        rospy.Subscriber(self.config.task_command_topic, String, self._handle_task, queue_size=10)
        rospy.Subscriber(self.config.demo_task_state_topic, String, self._handle_gazebo, queue_size=10)
        self.control_mode_request_pub = None
        self.control_mode_subscriber = None
        if self.config.control_mode_enabled:
            self.control_mode_request_pub = rospy.Publisher(
                self.config.control_mode_request_topic, String, queue_size=1, latch=False
            )
            self.control_mode_subscriber = rospy.Subscriber(
                self.config.control_mode_topic,
                String,
                self._handle_control_mode,
                queue_size=1,
            )
        self.confirm_pub = None
        self.dashboard_state_pub = None
        self.symbol_capture_pub = rospy.Publisher(self.config.symbol_capture_topic, String, queue_size=10, latch=True)
        # Confirmed-label publishing needs BOTH the integrated confirm gate AND
        # the explicit publish_confirmed_label gate. Preview launches keep both
        # false, so no confirmed_label publisher is ever created in preview mode.
        if self.config.integrated_confirm_enabled:
            if self.config.publish_confirmed_label:
                self.confirm_pub = rospy.Publisher(self.config.confirmed_label_topic, String, queue_size=10, latch=True)
                self.dashboard_state_pub = rospy.Publisher(self.config.dashboard_state_topic, String, queue_size=10, latch=True)
            else:
                rospy.logwarn("integrated confirm enabled but publish_confirmed_label=false "
                              "-> confirmed_label publishing stays disabled (preview only)")
        if self.config.publish_task_command:
            # No task_command publisher exists in this preview dashboard by design.
            rospy.logwarn("publish_task_command=true has no effect: the preview "
                          "dashboard never publishes /colmag/task_command")

        rospy.loginfo("MagneticTrajectoryDashboardNode started")
        rospy.loginfo("integrated_dashboard_confirm_enabled=%s publish_confirmed_label=%s publish_task_command=%s",
                      self.config.integrated_confirm_enabled, self.config.publish_confirmed_label, self.config.publish_task_command)
        rospy.loginfo("dashboard_board_control_layout=%s", self.config.board_control_layout)

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
        if self.config.is_trajectory_mode:
            self._ingest_trajectory_sample(data, pt)
            return
        self.trail.append(pt)
        self._schedule_ui_refresh()

    def _recognize_current_stroke(self):
        self._candidate_recognize_job = None
        if not (self.config.is_trajectory_mode and self.config.candidate_preview_enabled):
            return
        self.preview_interaction.recognizing()
        self._update_candidate_ui()
        points = self._active_recognition_points()
        status, candidates, backend_label = recognize_stroke(
            points,
            backend=self.config.recognition_backend,
            min_points=self.config.candidate_min_points,
            labels=self.config.candidate_labels,
            recognition_model_path=self.config.recognition_model_path,
        )
        self.recognition_backend_label = backend_label
        self.trajectory_candidates = candidates
        self.preview_interaction.recognized(status)
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
            if self.config.demo_input_mode == "ocr_canvas" and self.ocr_panel_state == "RECOGNIZING":
                self._cancel_ocr_recognition_timeout()
                self.ocr_panel_state = "CONFIRM_PENDING"
                self.root.after(0, lambda: self.ui_state_var.set("State: CONFIRM_PENDING"))

            self.current_candidate_payload = data
            self._update_real_board_sample_record_candidate(data)
            backend_text = candidate_backend_status_text(data)
            self.root.after(0, lambda t=backend_text: self.backend_status_var.set(t))
            if self.config.is_trajectory_mode and self.config.candidate_preview_enabled:
                self._sync_external_candidate_display(data, backend_text)
            recognizer_texts = recognizer_status_texts(data)
            self.root.after(0, lambda t=recognizer_texts["mode"]: self.recognizer_mode_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["model"]: self.recognizer_model_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["status"]: self.recognizer_status_var.set(t))
            self.root.after(0, lambda t=recognizer_texts["labels"]: self.recognizer_labels_var.set(t))

            tops = summarize_candidates(data)
            if self.config.demo_input_mode == "ocr_canvas":
                summary = ", ".join("%s:%.3f" % (label, confidence) for _rank, label, confidence in tops) or "-"
                self.rospy.loginfo("ocr candidates received top=%s", summary)

            for i, text in enumerate(format_rank_rows(data, count=3)):
                self.root.after(0, lambda i=i, t=text: self.rank_vars[i].set(t))
            self.root.after(0, self._update_ocr_toolbar_visuals)
            if self.config.demo_input_mode == "ocr_canvas":
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
        if self.config.demo_input_mode == "ocr_canvas":
            texts = format_ocr_canvas_dwell_texts(
                self.ocr_panel_state,
                self.dwell_timer.button,
                self.dwell_timer.progress,
                self.config.dwell_confirm_sec,
            )
            self.root.after(0, lambda t=texts["dwell"]: self.dwell_var.set(t))
            self.root.after(0, lambda t=texts["hover"]: self.hover_var.set(t))
            self.root.after(0, lambda: self.ocr_hint_var.set(self._ocr_state_hint_text()))
            self._refresh_mouse_operator_presentation()
            return

        if self.config.is_trajectory_mode and self.config.candidate_preview_enabled:
            text = format_trajectory_preview_interaction_text(
                self.preview_interaction.state,
                self.hover_source,
                self.dwell_timer.button,
                self.dwell_timer.progress,
                hover_progress_enabled=self.config.hover_progress_enabled,
            )
            self.root.after(0, lambda t=text: self.traj_interaction_var.set(t))
            if self.config.hover_buttons_enabled:
                hint = format_interaction_instruction(self.config.interaction_profile)
                self.root.after(0, lambda t=hint: self.ocr_hint_var.set(t))
            return

        texts = format_integrated_dwell_status_texts(
            self.config.integrated_confirm_enabled,
            self.dwell_state.get("current_button_id", ""),
            self.dwell_state.get("dwell_progress", 0.0),
            self.config.dwell_confirm_sec,
            self.last_command_intent,
        )
        self.root.after(0, lambda t=texts["dwell"]: self.dwell_var.set(t))
        self.root.after(0, lambda t=texts["hover"]: self.hover_var.set(t))

    def _publish_dashboard_state(self):
        if not self.config.integrated_confirm_enabled or self.dashboard_state_pub is None:
            return
        if self.config.demo_input_mode == "ocr_canvas":
            current_button = self.dwell_timer.button if self.dwell_timer.button != "C" or self._ocr_confirm_enabled() else ""
            payload = {
                "timestamp": time.time(),
                "mode": "ocr_canvas_dashboard",
                "ocr_panel_state": self.ocr_panel_state,
                "current_virtual_button": current_button,
                "dwell_progress": self.dwell_timer.progress if current_button else 0.0,
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
        if self.config.is_trajectory_mode:
            # Preview-only Start: record one complete segment (never publishes).
            if self.config.trajectory_recording_enabled and key == "s":
                self._start_recording()
                return
            # Recognition triggers. External DTW route publishes the captured
            # trajectory sample to the existing /colmag/symbol_capture seam.
            if self.config.candidate_preview_enabled:
                if not self.latest_input_active:
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
        if self.config.demo_input_mode != "ocr_canvas":
            return
        if key in ("b", "c", "x", "a"):
            self._handle_ocr_action(key)

if __name__ == "__main__":
    import rospy
    rospy.init_node("magnetic_trajectory_dashboard_node")
    MagneticTrajectoryDashboardNode().run()
