"""Focused package/install seam checks for M104-PKG-INSTALL1."""

import ast
import importlib
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


REPO = Path(__file__).resolve().parents[1]
ROS_SCRIPTS = REPO / "colmag_ros" / "scripts"
ROS_PACKAGE = REPO / "colmag_ros" / "src" / "colmag_ros"
DISPATCHER = ROS_SCRIPTS / "task_dispatcher_node.py"
BRIDGE = REPO / "colmag_gazebo_stub" / "scripts" / "fr3_gazebo_visible_task_bridge_node.py"
CMAKE = REPO / "colmag_ros" / "CMakeLists.txt"

CANONICAL_HELPERS = (
    "dashboard_candidate_display",
    "dashboard_confirm_publisher",
    "dashboard_dwell_status",
    "dashboard_geometry",
    "dashboard_points",
    "dtw_template_bank_tools",
    "m104c2c3_display_semantics",
    "m104c4_execution_semantics",
    "recognition_uncertainty",
    "semantic_library",
    "symbol_semantics",
    "trajectory_segment_recorder",
    "trajectory_strokes",
)


def test_canonical_helpers_import_from_package():
    for name in CANONICAL_HELPERS:
        module = importlib.import_module("colmag_ros.%s" % name)
        assert Path(module.__file__).resolve() == (ROS_PACKAGE / (name + ".py")).resolve()


def test_legacy_helper_wrappers_have_no_business_logic():
    for name in CANONICAL_HELPERS:
        tree = ast.parse((ROS_SCRIPTS / (name + ".py")).read_text())
        assert not [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]


def test_c4_contract_rejects_missing_attribute_and_empty_mapping():
    semantics = importlib.import_module("colmag_ros.m104c4_execution_semantics")
    valid = semantics.validated_execution_contract()
    assert valid["label_to_task"] == semantics.C4_LABEL_TO_TASK

    missing = SimpleNamespace(
        **{name: value for name, value in vars(semantics).items() if name != "C4_NO_MOTION_TASKS"}
    )
    with pytest.raises(RuntimeError, match="missing attributes"):
        semantics.validated_execution_contract(missing)

    empty = SimpleNamespace(**vars(semantics))
    empty.C4_LABEL_TO_TASK = {}
    with pytest.raises(RuntimeError, match="non-empty mapping"):
        semantics.validated_execution_contract(empty)


def test_dispatcher_and_bridge_use_canonical_fail_loud_contract():
    dispatcher = DISPATCHER.read_text()
    bridge = BRIDGE.read_text()
    for text in (dispatcher, bridge):
        assert "from colmag_ros import m104c4_execution_semantics" in text
        assert "validated_execution_contract()" in text
        assert "spec_from_file_location" not in text
        assert "getattr(_C4" not in text


def test_dispatcher_import_fails_when_canonical_semantics_module_is_missing(monkeypatch):
    empty_package = ModuleType("colmag_ros")
    empty_package.__path__ = []
    monkeypatch.setitem(sys.modules, "colmag_ros", empty_package)
    monkeypatch.delitem(sys.modules, "colmag_ros.m104c4_execution_semantics", raising=False)
    spec = importlib.util.spec_from_file_location("missing_c4_dispatcher", DISPATCHER)
    module = importlib.util.module_from_spec(spec)
    with pytest.raises(ImportError):
        spec.loader.exec_module(module)


def test_mouse_toolbar_stop_uses_existing_c4_cancel_contract():
    confirms = importlib.import_module("colmag_ros.dashboard_confirm_publisher")
    semantics = importlib.import_module("colmag_ros.m104c4_execution_semantics")
    payload = confirms.build_mouse_toolbar_stop_payload(123.0)

    assert payload == {
        "timestamp": 123.0,
        "label": "X",
        "confidence": 1.0,
        "confirmed": True,
        "confirmed_by": "ocr_canvas_toolbar_dwell",
        "selected_button": "A",
        "command_intent": "STOP_OR_CANCEL",
        "controls_real_robot": False,
        "gazebo_only": True,
        "test_only": False,
        "source_topic": "/colmag/confirmed_label",
    }
    assert semantics.C4_LABEL_TO_TASK[payload["label"]] == "STOP_OR_CANCEL"
    spec = importlib.util.spec_from_file_location("pkg_install_dispatcher", DISPATCHER)
    dispatcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dispatcher)
    command = dispatcher.build_task_command(
        payload,
        dispatcher_mapping_name="m104c4_8symbol_gazebo",
    )
    assert command["task"] == "STOP_OR_CANCEL"
    assert command["accepted"] is True
    assert command["controls_real_robot"] is False
    assert semantics.C4_LABEL_TO_TASK["A"] == "HOME_OR_READY"
    assert semantics.C4_LABEL_TO_TASK["C"] == "SOFT_DESCEND_PREVIEW"
    assert confirms.should_publish_mouse_toolbar_stop("WAITING") is True
    assert confirms.should_publish_mouse_toolbar_stop("BLOCKED") is False


def test_mouse_toolbar_wording_and_legacy_gamepad_mapping_scope():
    dashboard = (ROS_SCRIPTS / "magnetic_trajectory_dashboard_node.py").read_text()
    controller_input = (ROS_PACKAGE / "dashboard_controller_input.py").read_text()
    assert "OCR toolbar" not in dashboard
    assert '"A": "S"' in controller_input
    assert '"A": "STOP"' in controller_input
    assert "build_mouse_toolbar_stop_payload" in dashboard


def test_real_board_active_state_node_is_installed_with_other_entrypoints():
    cmake = CMAKE.read_text()
    assert "scripts/magnetic_ui_state_node.py" in cmake
    assert cmake.index("catkin_install_python(PROGRAMS") < cmake.index(
        "scripts/magnetic_ui_state_node.py"
    ) < cmake.index("DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}")
