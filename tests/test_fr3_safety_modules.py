from pathlib import Path

from cpp_harness import compile_and_run


REPO = Path(__file__).resolve().parents[1]
INCLUDE_DIR = REPO / "colmag_ros" / "include"


def test_ros_free_joint_nudge_trajectory_and_limit_contract(tmp_path):
    compile_and_run(tmp_path, "fr3_trajectory_safety_test", "fr3_trajectory_safety.cpp")


def test_ros_free_command_replay_one_shot_and_lifecycle_contract(tmp_path):
    compile_and_run(tmp_path, "fr3_hardware_safety_test", "fr3_hardware_safety.cpp")


def test_ros_free_cartesian_teleop_core_contract(tmp_path):
    compile_and_run(tmp_path, "fr3_cartesian_teleop_core_test", "fr3_cartesian_teleop_core.cpp")


def test_json_escape_header(tmp_path):
    compile_and_run(tmp_path, "json_escape_test")


def test_safety_modules_remain_ros_free():
    paths = [
        INCLUDE_DIR / "colmag_ros" / "fr3_trajectory_safety.h",
        INCLUDE_DIR / "colmag_ros" / "fr3_hardware_safety.h",
        REPO / "colmag_ros" / "src" / "fr3_trajectory_safety.cpp",
        REPO / "colmag_ros" / "src" / "fr3_hardware_safety.cpp",
    ]
    combined = "\n".join(path.read_text() for path in paths)
    for forbidden in ("ros::", "NodeHandle", "Publisher", "SimpleActionClient"):
        assert forbidden not in combined
