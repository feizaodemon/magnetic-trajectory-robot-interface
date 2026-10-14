"""Read-once, immutable parameters of the trajectory dashboard node.

``DashboardConfig.from_params`` holds every ``~param`` read, default, and
fallback chain of the dashboard. It takes a ``get_param(name, default)``
callable (``rospy.get_param`` at runtime), so it can be tested without ROS.
Values that change while the node runs (canvas size, trail, candidates) are
node state, not config.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from colmag_ros.dashboard_dwell_status import normalize_interaction_profile
from colmag_ros.dashboard_geometry import (
    BOARD_CONTROL_LAYOUT_EXTERNAL,
    normalize_board_control_layout,
)
from colmag_ros.dashboard_sample_lifecycle import (
    DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR,
    real_board_sample_recording_dir,
)
from colmag_ros.dashboard_texts import (
    TRAJECTORY_MODE,
    default_footer_text,
    default_safety_badge,
)
from colmag_ros.dashboard_trajectory_processing import (
    normalize_scale_mode,
    normalize_stroke_gate_mode,
)
from colmag_ros.param_utils import bool_param, optional_float
from colmag_ros.preview_recognition import RECOGNIZER_BACKENDS

# Default magnetic-board extent (m) of the non-trajectory canvas.
REFERENCE_MAGNETIC_EXTENT_M = 0.05


@dataclass(frozen=True)
class DashboardConfig:
    # Topics.
    trajectory_topic: str
    ui_state_topic: str
    symbol_capture_topic: str
    symbol_candidates_topic: str
    confirmed_label_topic: str
    dashboard_state_topic: str
    task_command_topic: str
    demo_task_state_topic: str
    control_mode_enabled: bool
    control_mode_topic: str
    control_mode_request_topic: str

    # Confirm gates. The confirmed_label publisher needs both
    # integrated_confirm_enabled and publish_confirmed_label. The preview
    # dashboard has no task_command publisher; publish_task_command only
    # triggers a warning.
    integrated_confirm_enabled: bool
    publish_confirmed_label: bool
    publish_task_command: bool
    # "internal": confirm the in-process preview candidates.
    # "external_symbol_candidates": confirm the /colmag/symbol_candidates
    # payload, falling back to internal when it has no candidates.
    confirm_candidate_source: str
    dwell_confirm_sec: float

    # Mode and fixed texts.
    demo_input_mode: str
    is_trajectory_mode: bool
    interaction_profile: str
    subtitle_text: Optional[str]
    safety_badge: str
    footer_text: str
    route_scope_text: str
    confirm_policy_text: str

    # Canvas. canvas_width/height are the initial size; the node resizes its
    # canvas on layout.
    canvas_width: int
    canvas_height: int
    padding: int
    trail_size: int
    bounds: Tuple[float, float, float, float]

    # Real magnetic board preview scaling / orientation (trajectory mode).
    traj_flip_x: bool
    traj_flip_y: bool
    traj_swap_xy: bool
    # fixed | fixed_symmetric | auto. fixed_symmetric is the recommended demo
    # mode; auto stays diagnostic.
    traj_scale_mode: str
    traj_auto_center: bool
    # Drawn-trail length (recent points). 180 keeps a real-board symbol
    # visible long enough for a demo while still bounding stale history.
    traj_trace_window_points: int
    traj_auto_padding_ratio: float
    traj_auto_min_span: float
    # Fixed symmetric extent for fixed_symmetric mode (+/- this value, both axes).
    traj_projection_extent: float

    # Preview-only stroke gate (a display mask; never a robot/task/confirm
    # signal). Defaults pass every stroke through.
    stroke_gate_mode: str
    gate_z_min: Optional[float]
    gate_z_max: Optional[float]
    gate_min_speed: float
    gate_max_speed: float  # <=0 -> no upper limit
    gate_invalid_breaks_stroke: bool
    input_valid_gate_enabled: bool
    input_z_min: Optional[float]
    input_z_max: Optional[float]
    input_reactivate_samples: int

    # Drawing-sample cleanup and recording.
    board_sample_cleanup_enabled: bool
    board_sample_min_point_delta: float
    board_sample_max_points: int
    dashboard_debug_sample_panel: bool
    real_board_sample_recording_enabled: bool
    real_board_sample_recording_dir: str

    # Preview-only candidate display. It never publishes /colmag/task_command
    # or any robot signal.
    ocr_preview_enabled: bool
    candidate_preview_enabled: bool
    recognition_backend: str
    candidate_min_points: int
    candidate_labels: Optional[list]
    auto_recognize: bool
    ocr_debounce_sec: float
    ocr_recognition_timeout_ms: int

    # Preview buttons. The live board's red point can drive button dwell
    # without a mouse; mouse hover/click stays as fallback.
    hover_buttons_enabled: bool
    hover_click_enabled: bool
    hover_dwell_enabled: bool
    control_dwell_sec: float
    hover_progress_enabled: bool
    board_cursor_dwell_enabled: bool
    board_control_layout: str

    # Preview-only segment recording: Start records ONE complete segment and
    # recognition runs on it instead of the live stream. Off by default.
    trajectory_recording_enabled: bool
    record_duration_sec: float
    # z-lift stop is off by default: the real board's z is not yet validated
    # live. The time window is the primary stop.
    record_stop_on_lift: bool
    record_lift_z_threshold: float
    record_min_points: int
    record_max_points: int
    recognize_only_completed_segment: bool
    semantic_preview_enabled: bool

    @classmethod
    def from_params(cls, get_param):
        def get(name, default=None):
            return get_param("~" + name, default)

        def flag(name, default):
            return bool_param(get(name, default))

        demo_input_mode = get("demo_input_mode", "trajectory")
        is_trajectory_mode = demo_input_mode == TRAJECTORY_MODE
        if demo_input_mode == "ocr_canvas":
            canvas_width, canvas_height = 360, 300
        else:
            canvas_width = get("trajectory_dashboard_canvas_width", 560)
            canvas_height = get("trajectory_dashboard_canvas_height", 520)
        if is_trajectory_mode:
            bounds = (
                float(get("trajectory_x_min", -0.07)),
                float(get("trajectory_x_max", 0.07)),
                float(get("trajectory_y_min", -0.07)),
                float(get("trajectory_y_max", 0.09)),
            )
        else:
            bounds = (
                get("trajectory_dashboard_x_min", -REFERENCE_MAGNETIC_EXTENT_M),
                get("trajectory_dashboard_x_max", REFERENCE_MAGNETIC_EXTENT_M),
                get("trajectory_dashboard_y_min", -REFERENCE_MAGNETIC_EXTENT_M),
                get("trajectory_dashboard_y_max", REFERENCE_MAGNETIC_EXTENT_M),
            )

        gate_z_min = optional_float(get("trajectory_z_min", None))
        gate_z_max = optional_float(get("trajectory_z_max", None))
        raw_input_z_min = get("trajectory_input_z_min", "inherit")
        raw_input_z_max = get("trajectory_input_z_max", "inherit")

        recognition_backend = str(get("recognition_backend", "dtw")).strip().lower()
        if recognition_backend not in RECOGNIZER_BACKENDS:
            recognition_backend = "dtw"
        labels = get("candidate_labels", None)
        if isinstance(labels, str):
            labels = [token for token in labels.replace(",", " ").split() if token]

        ocr_preview_enabled = flag("ocr_preview_enabled", False)
        return cls(
            trajectory_topic=get("trajectory_topic", "/colmag/trajectory_2d"),
            ui_state_topic=get("ui_state_topic", "/colmag/ui_state"),
            symbol_capture_topic=get("symbol_capture_topic", "/colmag/symbol_capture"),
            symbol_candidates_topic=get("symbol_candidates_topic", "/colmag/symbol_candidates"),
            confirmed_label_topic=get("confirmed_label_topic", "/colmag/confirmed_label"),
            dashboard_state_topic=get("dashboard_state_topic", "/colmag/trajectory_dashboard_state"),
            task_command_topic=get("task_command_topic", "/colmag/task_command"),
            demo_task_state_topic=get("demo_task_state_topic", "/colmag/fr3_demo_task_state"),
            control_mode_enabled=flag("control_mode_enabled", False),
            control_mode_topic=get("control_mode_topic", "/colmag/control_mode"),
            control_mode_request_topic=get("control_mode_request_topic", "/colmag/control_mode/request"),
            integrated_confirm_enabled=flag("integrated_dashboard_confirm_enabled", False),
            publish_confirmed_label=flag("publish_confirmed_label", False),
            publish_task_command=flag("publish_task_command", False),
            confirm_candidate_source=str(get("confirm_candidate_source", "internal")),
            dwell_confirm_sec=float(get("dashboard_dwell_confirm_sec", 2.0)),
            demo_input_mode=demo_input_mode,
            is_trajectory_mode=is_trajectory_mode,
            interaction_profile=normalize_interaction_profile(
                get("interaction_profile", None), demo_input_mode),
            subtitle_text=get("subtitle_text", None),
            safety_badge=get("safety_badge", default_safety_badge(demo_input_mode)),
            footer_text=get("footer_text", default_footer_text(demo_input_mode)),
            route_scope_text=get("route_scope_text", "Gazebo only. No real robot command."),
            confirm_policy_text=get("confirm_policy_text", "Confirm publishes selected label."),
            canvas_width=canvas_width,
            canvas_height=canvas_height,
            padding=20,
            trail_size=get("trajectory_dashboard_trail_size", 300),
            bounds=bounds,
            traj_flip_x=flag("trajectory_flip_x", False),
            traj_flip_y=flag("trajectory_flip_y", False),
            traj_swap_xy=flag("trajectory_swap_xy", False),
            traj_scale_mode=normalize_scale_mode(get("trajectory_scale_mode", "fixed")),
            traj_auto_center=flag("trajectory_auto_center", False),
            traj_trace_window_points=max(2, int(get(
                "trajectory_trace_window_points", get("trajectory_auto_window_points", 180)))),
            traj_auto_padding_ratio=float(get("trajectory_auto_padding_ratio", 0.12)),
            traj_auto_min_span=float(get("trajectory_auto_min_span", 0.01)),
            traj_projection_extent=float(get("trajectory_projection_extent", 0.06)),
            stroke_gate_mode=normalize_stroke_gate_mode(get("trajectory_stroke_gate_mode", "off")),
            gate_z_min=gate_z_min,
            gate_z_max=gate_z_max,
            gate_min_speed=float(get("trajectory_min_speed", 0.0)),
            gate_max_speed=float(get("trajectory_max_speed", 0.0)),
            gate_invalid_breaks_stroke=flag("trajectory_gate_invalid_breaks_stroke", True),
            input_valid_gate_enabled=flag("trajectory_input_valid_gate_enabled", True),
            input_z_min=(gate_z_min if str(raw_input_z_min).strip().lower() == "inherit"
                         else optional_float(raw_input_z_min)),
            input_z_max=(gate_z_max if str(raw_input_z_max).strip().lower() == "inherit"
                         else optional_float(raw_input_z_max)),
            input_reactivate_samples=max(1, int(get("trajectory_input_reactivate_samples", 2))),
            board_sample_cleanup_enabled=flag("dashboard_board_sample_cleanup_enabled", True),
            board_sample_min_point_delta=max(0.0, float(get(
                "dashboard_board_sample_min_point_delta", 0.0015))),
            board_sample_max_points=max(0, int(get("dashboard_board_sample_max_points", 160))),
            dashboard_debug_sample_panel=flag("dashboard_debug_sample_panel", False),
            real_board_sample_recording_enabled=flag("real_board_sample_recording_enabled", False),
            real_board_sample_recording_dir=real_board_sample_recording_dir(
                get("real_board_sample_recording_dir", DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR)),
            ocr_preview_enabled=ocr_preview_enabled,
            candidate_preview_enabled=(
                flag("candidate_preview_enabled", False) or ocr_preview_enabled),
            recognition_backend=recognition_backend,
            candidate_min_points=int(get("candidate_min_points", get("ocr_min_points", 8))),
            candidate_labels=list(labels) if labels else None,
            auto_recognize=(
                flag("candidate_auto_recognize", False) or flag("ocr_auto_recognize", False)),
            ocr_debounce_sec=float(get("ocr_debounce_sec", get("candidate_auto_idle_sec", 0.8))),
            ocr_recognition_timeout_ms=int(get("ocr_recognition_timeout_ms", 12000)),
            hover_buttons_enabled=flag("hover_buttons_enabled", True),
            hover_click_enabled=flag("hover_click_enabled", True),
            hover_dwell_enabled=flag("hover_dwell_enabled", False),
            control_dwell_sec=float(get("dashboard_control_dwell_sec", get("hover_dwell_sec", 1.0))),
            hover_progress_enabled=flag("hover_progress_enabled", True),
            board_cursor_dwell_enabled=flag("board_cursor_dwell_enabled", True),
            board_control_layout=normalize_board_control_layout(
                get("dashboard_board_control_layout", BOARD_CONTROL_LAYOUT_EXTERNAL)),
            trajectory_recording_enabled=flag("trajectory_recording_enabled", False),
            record_duration_sec=float(get("record_duration_sec", 2.5)),
            record_stop_on_lift=flag("record_stop_on_lift", False),
            record_lift_z_threshold=float(get("record_lift_z_threshold", 0.05)),
            record_min_points=int(get("record_min_points", 8)),
            record_max_points=int(get("record_max_points", 600)),
            recognize_only_completed_segment=flag("recognize_only_completed_segment", True),
            semantic_preview_enabled=flag("semantic_preview_enabled", True),
        )
