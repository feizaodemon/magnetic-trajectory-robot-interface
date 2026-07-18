"""Static contracts for the two supported DTW demo systems."""

import importlib.util
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
LAUNCH = REPO / "colmag_ros" / "launch"
GAZEBO = LAUNCH / "colmag_8symbol_gazebo_robot_body_demo.launch"
BOARD_GAZEBO = LAUNCH / "colmag_real_board_ready_gazebo_robot_body_demo.launch"
MOUSE_HARDWARE = LAUNCH / "colmag_mouse_fr3_hardware_demo.launch"
BOARD_HARDWARE = LAUNCH / "colmag_real_board_fr3_hardware_demo.launch"
MOUSE_FRONTEND = LAUNCH / "dtw_mouse_demo_frontend.launch"
BOARD_FRONTEND = LAUNCH / "dtw_real_board_demo_frontend.launch"
ADAPTER = REPO / "colmag_ros" / "src" / "colmag_ros" / "adapter.py"
SERIAL_NODE = REPO / "colmag_ros" / "scripts" / "serial_packet_publisher_node.py"
FR3_DOCKERFILE = REPO / "docker" / "Dockerfile.noetic-fr3"


def load_adapter():
    spec = importlib.util.spec_from_file_location("colmag_portability_adapter", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def directives(path):
    return re.sub(r"<!--.*?-->", "", path.read_text(encoding="utf-8"), flags=re.S)


def test_supported_entrypoints_and_frontends_are_valid_xml():
    for path in (GAZEBO, BOARD_GAZEBO, MOUSE_HARDWARE, BOARD_HARDWARE,
                 MOUSE_FRONTEND, BOARD_FRONTEND):
        ET.parse(path)


def test_gazebo_routes_own_all_gazebo_dependencies():
    mouse = directives(LAUNCH / "final_gazebo_robot_demo.launch")
    board = directives(BOARD_GAZEBO)
    for text in (mouse, board):
        assert "franka_gazebo" in text
        assert "fr3_gazebo_visible_task_bridge_node.py" in text
        assert "follow_joint_trajectory" in text


def test_real_fr3_routes_and_shared_frontends_have_no_gazebo_dependency():
    paths = (
        MOUSE_HARDWARE,
        BOARD_HARDWARE,
        LAUNCH / "colmag_mouse_fr3_adapter_dry_run.launch",
        LAUNCH / "colmag_real_board_fr3_adapter_dry_run.launch",
        MOUSE_FRONTEND,
        BOARD_FRONTEND,
        LAUNCH / "colmag_fr3_hardware_stack.launch",
    )
    for path in paths:
        text = directives(path).lower()
        assert "franka_gazebo" not in text, path
        assert "colmag_gazebo_stub" not in text, path
        assert "gazebo_visible" not in text, path


def test_frontends_are_dtw_only():
    for path in (MOUSE_FRONTEND, BOARD_FRONTEND):
        text = directives(path).lower()
        assert "dtw_template_bank" in text
        for stale in ("easyocr", "model_inference", "joblib", "candidate_panel", "auto_confirm"):
            assert stale not in text, (path, stale)


def test_real_board_frontend_disables_legacy_controls_and_owns_one_visible_dwell():
    root = ET.parse(BOARD_FRONTEND).getroot()
    args = {
        node.attrib["name"]: node.attrib.get("default", "")
        for node in root.findall("arg")
    }
    nodes = {node.attrib["name"]: node for node in root.findall("node")}
    legacy_params = {
        node.attrib["name"]: node.attrib.get("value", "")
        for node in nodes["magnetic_ui_state_node"].findall("param")
    }
    dashboard_params = {
        node.attrib["name"]: node.attrib.get("value", "")
        for node in nodes["magnetic_trajectory_dashboard_node"].findall("param")
    }

    assert args["enable_legacy_control_zones"] == "false"
    assert legacy_params["enable_legacy_control_zones"] == "$(arg enable_legacy_control_zones)"
    assert args["dashboard_control_dwell_sec"] == "1.0"
    assert dashboard_params["dashboard_control_dwell_sec"] == "$(arg dashboard_control_dwell_sec)"
    assert "dashboard_dwell_confirm_sec" not in args
    assert "dashboard_dwell_confirm_sec" not in dashboard_params
    assert "candidate_auto_recognize" not in dashboard_params


def test_frontends_select_source_specific_presentation_profiles_on_shared_dashboard():
    profiles = {}
    modes = {}
    for name, path in (("mouse", MOUSE_FRONTEND), ("real_board", BOARD_FRONTEND)):
        root = ET.parse(path).getroot()
        dashboard = next(
            node for node in root.findall("node")
            if node.attrib.get("type") == "magnetic_trajectory_dashboard_node.py"
        )
        params = {
            param.attrib["name"]: param.attrib.get("value", "")
            for param in dashboard.findall("param")
        }
        args = {
            arg.attrib["name"]: arg.attrib.get("default", "")
            for arg in root.findall("arg")
        }
        profiles[name] = args["interaction_profile"]
        modes[name] = params["demo_input_mode"]
        assert params["interaction_profile"] == "$(arg interaction_profile)"

    assert profiles == {"mouse": "mouse", "real_board": "real_board"}
    assert modes == {"mouse": "ocr_canvas", "real_board": "trajectory"}
    assert "tracking_mz" not in directives(MOUSE_FRONTEND)


def test_frontend_audit_decorations_are_optional_and_clean_by_default():
    for path in (MOUSE_FRONTEND, BOARD_FRONTEND):
        root = ET.parse(path).getroot()
        args = {
            arg.attrib["name"]: arg.attrib.get("default", "")
            for arg in root.findall("arg")
        }
        assert args["safety_badge"] == ""
        assert args["footer_text"] == ""
        dashboard = next(
            node for node in root.findall("node")
            if node.attrib.get("type") == "magnetic_trajectory_dashboard_node.py"
        )
        params = {
            param.attrib["name"]: param.attrib.get("value", "")
            for param in dashboard.findall("param")
        }
        assert params["safety_badge"] == "$(arg safety_badge)"
        assert params["footer_text"] == "$(arg footer_text)"


@pytest.mark.parametrize("value", [None, "", "   "])
def test_missing_direct_project_src_path_is_rejected(value):
    with pytest.raises(ValueError, match="missing_project_src_path"):
        load_adapter().ensure_project_src_path(value)


def test_invalid_direct_project_src_path_is_rejected(tmp_path):
    missing = tmp_path / "missing"
    with pytest.raises(ValueError, match="invalid_project_src_path"):
        load_adapter().ensure_project_src_path(str(missing))


def test_explicit_repository_or_src_path_is_accepted(tmp_path, monkeypatch):
    repository = tmp_path / "checkout"
    project_src = repository / "src"
    (project_src / "serial_mvp").mkdir(parents=True)
    monkeypatch.setattr(sys, "path", list(sys.path))

    load_adapter().ensure_project_src_path(str(repository))

    assert sys.path[0] == str(project_src)


def test_serial_path_owner_has_no_home_or_cwd_guessing():
    adapter = ADAPTER.read_text(encoding="utf-8")
    node = SERIAL_NODE.read_text(encoding="utf-8")
    machine_home = "/home/" + "example-user/"
    assert machine_home not in adapter
    assert not re.search(r"/home/[^/]+/.+COLMAG.+/src", adapter)
    assert "Path.cwd" not in adapter
    assert "Path.home" not in adapter
    assert "DEFAULT_PROJECT_SRC_PATH" not in adapter + node
    assert 'get_param("~project_src_path", None)' in node
    assert "except ValueError as exc" in node
    assert "rospy.logfatal" in node
    assert node.index("ensure_project_src_path(self.project_src_path)") < node.index(
        "from serial_mvp.reader import SerialPacketReader"
    )


def test_canonical_board_path_is_explicit_and_mouse_stays_serial_free():
    board = directives(BOARD_FRONTEND)
    mouse = directives(MOUSE_FRONTEND)
    assert '<arg name="project_src_path" default="$(find colmag_ros)/../src"' in board
    assert '<param name="project_src_path" value="$(arg project_src_path)"' in board
    assert "serial_packet_publisher_node.py" in board
    assert "serial_packet_publisher_node.py" not in mouse
    assert "project_src_path" not in mouse


def test_fr3_docker_keeps_installed_serial_mvp_import_contract():
    dockerfile = FR3_DOCKERFILE.read_text(encoding="utf-8")
    assert "COPY src/serial_mvp /opt/colmag_python/lib/python3/dist-packages/serial_mvp" in dockerfile
    assert "PYTHONPATH=/opt/colmag_python/lib/python3/dist-packages" in dockerfile
    assert "from serial_mvp.reader import SerialPacketReader" in dockerfile
    assert "from serial_mvp.sources import SerialPortSource" in dockerfile


def test_real_hardware_defaults_remain_no_motion():
    for path in (MOUSE_HARDWARE, BOARD_HARDWARE):
        text = directives(path)
        assert '<arg name="send_goals" default="false"' in text
        assert '<arg name="hardware_execution_enabled" default="false"' in text
        assert '<arg name="hardware_profile" default="dry-run"' in text
