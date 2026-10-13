"""Static install-list checks for M104-RELEASE3 + MAINT1."""

import xml.etree.ElementTree as ET
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
COLMAG_ROS_CMAKE = REPO / "colmag_ros" / "CMakeLists.txt"
GAZEBO_STUB_CMAKE = REPO / "colmag_gazebo_stub" / "CMakeLists.txt"
ACTIVE_LAUNCHES = (
    REPO / "colmag_ros" / "launch" / "final_gazebo_robot_demo.launch",
    REPO / "colmag_ros" / "launch" / "dtw_mouse_demo_frontend.launch",
)


def _active_node_types():
    return {
        node.attrib.get("type", "")
        for path in ACTIVE_LAUNCHES
        for node in ET.parse(path).getroot().iter("node")
    }


def test_active_dashboard_and_gazebo_bridge_are_install_listed():
    active = _active_node_types()
    colmag_ros_cmake = COLMAG_ROS_CMAKE.read_text(encoding="utf-8")
    gazebo_stub_cmake = GAZEBO_STUB_CMAKE.read_text(encoding="utf-8")

    assert "magnetic_trajectory_dashboard_node.py" in active
    assert "scripts/magnetic_trajectory_dashboard_node.py" in colmag_ros_cmake
    assert "fr3_gazebo_visible_task_bridge_node.py" in active
    assert "scripts/fr3_gazebo_visible_task_bridge_node.py" in gazebo_stub_cmake


def test_new_shared_helpers_are_install_listed_with_their_callers():
    colmag_ros_cmake = COLMAG_ROS_CMAKE.read_text(encoding="utf-8")
    gazebo_stub_cmake = GAZEBO_STUB_CMAKE.read_text(encoding="utf-8")

    assert "catkin_python_setup()" in colmag_ros_cmake
    assert (
        REPO / "colmag_ros" / "src" / "colmag_ros" / "dtw_template_bank_tools.py"
    ).exists()
    assert "catkin_python_setup()" in gazebo_stub_cmake
    assert (REPO / "colmag_gazebo_stub" / "setup.py").exists()
    assert (
        REPO
        / "colmag_gazebo_stub"
        / "src"
        / "colmag_gazebo_stub"
        / "follow_joint_trajectory_goal_validator.py"
    ).exists()
