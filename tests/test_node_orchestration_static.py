"""Structural gates for the behavior-preserving node orchestration refactor."""

import ast
from pathlib import Path

from colmag_ros import dashboard_preview_controls
from colmag_ros.dashboard_candidate_display import PreviewCandidateDisplayState
from dashboard_harness import DashboardHarness


REPO = Path(__file__).resolve().parents[1]
DASHBOARD = REPO / "colmag_ros" / "scripts" / "magnetic_trajectory_dashboard_node.py"
DISPLAY = REPO / "colmag_ros" / "src" / "colmag_ros" / "dashboard_candidate_display.py"
RECOGNIZER = REPO / "colmag_ros" / "scripts" / "trajectory_symbol_top3_recognizer_node.py"


def _dashboard_methods():
    tree = ast.parse(DASHBOARD.read_text())
    dashboard = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "MagneticTrajectoryDashboardNode"
    )
    return {
        node.name: node
        for node in dashboard.body
        if isinstance(node, ast.FunctionDef)
    }


def _self_call_names(function):
    names = []
    for statement in function.body:
        expression = statement.value if isinstance(statement, ast.Expr) else None
        call = expression if isinstance(expression, ast.Call) else None
        target = call.func if call is not None else None
        if (
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "self"
        ):
            names.append(target.attr)
    return names


def test_dashboard_init_is_readable_ordered_orchestration():
    methods = _dashboard_methods()
    assert _self_call_names(methods["__init__"]) == [
        "_initialize_runtime_dependencies",
        "_read_route_and_trajectory_parameters",
        "_read_preview_and_recording_parameters",
        "_initialize_dashboard_state",
        "_initialize_window_and_variables",
        "_build_ui",
        "_refresh_dwell_status",
        "_schedule_ui_refresh",
        "_initialize_ros_interfaces",
    ]
    assert methods["__init__"].end_lineno - methods["__init__"].lineno + 1 < 120


def test_setup_methods_remain_cohesive_instead_of_fragmented_wrappers():
    methods = _dashboard_methods()
    setup_names = [
        "_initialize_runtime_dependencies",
        "_read_route_and_trajectory_parameters",
        "_read_preview_and_recording_parameters",
        "_initialize_dashboard_state",
        "_initialize_window_and_variables",
        "_initialize_ros_interfaces",
    ]
    assert all(name in methods for name in setup_names)
    assert len(setup_names) == 6


def test_ros_interface_setup_preserves_subscriber_then_publisher_order(monkeypatch):
    harness = DashboardHarness(
        monkeypatch, control_mode_enabled=True,
        integrated_dashboard_confirm_enabled=True, publish_confirmed_label=True)
    registrations = harness.rospy.registrations
    subscribed = [topic for kind, topic in registrations if kind == "sub"]
    published = [topic for kind, topic in registrations if kind == "pub"]

    assert registrations[0][0] == "sub"
    assert sorted(subscribed) == sorted([
        "/colmag/trajectory_2d", "/colmag/ui_state", "/colmag/symbol_capture",
        "/colmag/symbol_candidates", "/colmag/confirmed_label", "/colmag/task_command",
        "/colmag/fr3_demo_task_state", "/colmag/control_mode",
    ])
    assert sorted(published) == sorted([
        "/colmag/control_mode/request", "/colmag/symbol_capture",
        "/colmag/confirmed_label", "/colmag/trajectory_dashboard_state",
    ])


def test_display_state_and_formatter_live_only_in_canonical_package(monkeypatch):
    canonical = DISPLAY.read_text()
    assert "class PreviewCandidateDisplayState" in canonical
    assert "def preview_candidate_ui_texts(state):" in canonical
    assert not (REPO / "colmag_ros" / "scripts" / "dashboard_candidate_display.py").exists()

    # The dashboard builds the canonical state and shows the canonical formatter's text.
    formatter = dashboard_preview_controls.preview_candidate_ui_texts
    states = []

    def recording_formatter(state):
        states.append(state)
        return formatter(state)

    monkeypatch.setattr(
        dashboard_preview_controls, "preview_candidate_ui_texts", recording_formatter)
    harness = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    harness.send_sample_and_candidates()
    harness.press_key("x")

    state = states[-1]
    assert isinstance(state, PreviewCandidateDisplayState)
    assert state.preview_interaction_state == "cleared"
    assert harness.node.traj_candidate_status_var.get() == formatter(state)["status"]


def test_recognizer_callback_delegates_without_changing_public_helpers():
    tree = ast.parse(RECOGNIZER.read_text())
    recognizer = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TrajectorySymbolTop3RecognizerNode"
    )
    methods = {
        node.name: node
        for node in recognizer.body
        if isinstance(node, ast.FunctionDef)
    }
    assert {"_parse_capture_message", "_recognize_capture", "_build_recognition_payload"} <= set(methods)
    callback = methods["_handle_capture"]
    assert callback.end_lineno - callback.lineno + 1 < 80
