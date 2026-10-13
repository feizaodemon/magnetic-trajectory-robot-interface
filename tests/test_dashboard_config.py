"""DashboardConfig.from_params: defaults, legacy fallback names, and normalization."""

import pytest

from colmag_ros.dashboard_config import DashboardConfig


def _config(**params):
    values = {"~" + name: value for name, value in params.items()}
    return DashboardConfig.from_params(lambda name, default=None: values.get(name, default))


def test_trajectory_mode_defaults():
    config = _config()
    assert config.is_trajectory_mode
    assert config.bounds == (-0.07, 0.07, -0.07, 0.09)
    assert (config.canvas_width, config.canvas_height) == (560, 520)
    assert config.recognition_backend == "dtw"
    assert config.candidate_min_points == 8
    assert config.control_dwell_sec == 1.0
    assert config.ocr_debounce_sec == 0.8
    assert config.traj_trace_window_points == 180
    assert config.symbol_capture_topic == "/colmag/symbol_capture"


def test_other_modes_use_dashboard_bounds_and_ocr_canvas_size():
    ocr = _config(demo_input_mode="ocr_canvas", trajectory_dashboard_x_min=-0.2)
    assert not ocr.is_trajectory_mode
    assert ocr.bounds == (-0.2, 0.05, -0.05, 0.05)
    assert (ocr.canvas_width, ocr.canvas_height) == (360, 300)


@pytest.mark.parametrize("field, legacy, value", [
    ("candidate_min_points", "ocr_min_points", 5),
    ("control_dwell_sec", "hover_dwell_sec", 0.6),
    ("ocr_debounce_sec", "candidate_auto_idle_sec", 1.5),
    ("traj_trace_window_points", "trajectory_auto_window_points", 50),
])
def test_legacy_param_names_are_fallbacks(field, legacy, value):
    assert getattr(_config(**{legacy: value}), field) == value


@pytest.mark.parametrize("field, primary, legacy", [
    ("candidate_min_points", "candidate_min_points", "ocr_min_points"),
    ("control_dwell_sec", "dashboard_control_dwell_sec", "hover_dwell_sec"),
    ("ocr_debounce_sec", "ocr_debounce_sec", "candidate_auto_idle_sec"),
    ("traj_trace_window_points", "trajectory_trace_window_points",
     "trajectory_auto_window_points"),
])
def test_primary_param_name_wins_over_legacy(field, primary, legacy):
    assert getattr(_config(**{primary: 12, legacy: 3}), field) == 12


def test_input_z_limits_inherit_the_stroke_gate_by_default():
    config = _config(trajectory_z_min=0.01, trajectory_z_max="0.2",
                     trajectory_input_z_max=0.15)
    assert (config.gate_z_min, config.gate_z_max) == (0.01, 0.2)
    assert (config.input_z_min, config.input_z_max) == (0.01, 0.15)


def test_flags_and_normalization():
    config = _config(
        recognition_backend=" BOGUS ", ocr_preview_enabled="true",
        candidate_preview_enabled="false", candidate_labels="1, 2 3",
        trajectory_trace_window_points=1, trajectory_input_reactivate_samples=0)
    assert config.recognition_backend == "dtw"
    # OCR preview implies candidate preview.
    assert config.candidate_preview_enabled is True
    assert config.candidate_labels == ["1", "2", "3"]
    assert config.traj_trace_window_points == 2
    assert config.input_reactivate_samples == 1


def test_config_is_frozen():
    config = _config()
    with pytest.raises(AttributeError):
        config.publish_confirmed_label = True
