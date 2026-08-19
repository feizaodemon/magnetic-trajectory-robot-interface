"""Structural gates for the behavior-preserving M104-MAINT3 refactor."""

import ast
from pathlib import Path


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


def test_ros_interface_setup_preserves_subscriber_then_publisher_order():
    method = _dashboard_methods()["_initialize_ros_interfaces"]
    source_lines = DASHBOARD.read_text().splitlines()
    body = "\n".join(source_lines[method.lineno - 1:method.end_lineno])
    assert body.index("rospy.Subscriber") < body.index("rospy.Publisher")
    assert body.count("rospy.Subscriber") == 8
    assert "self.control_mode_subscriber = rospy.Subscriber" in body
    assert body.count("rospy.Publisher") == 4
    assert "self.control_mode_request_pub = rospy.Publisher" in body


def test_display_state_and_formatter_live_only_in_canonical_package():
    canonical = DISPLAY.read_text()
    dashboard = DASHBOARD.read_text()
    wrapper = (REPO / "colmag_ros" / "scripts" / "dashboard_candidate_display.py").read_text()
    assert "class PreviewCandidateDisplayState" in canonical
    assert "def preview_candidate_ui_texts(state):" in canonical
    assert "PreviewCandidateDisplayState(" in dashboard
    assert "preview_candidate_ui_texts(display_state)" in dashboard
    assert "class PreviewCandidateDisplayState" not in wrapper
    assert "def preview_candidate_ui_texts" not in wrapper


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
    source = RECOGNIZER.read_text()
    for public_helper in ("def dtw_distance", "def normalize_points", "def resample_by_arclength"):
        assert public_helper in source
