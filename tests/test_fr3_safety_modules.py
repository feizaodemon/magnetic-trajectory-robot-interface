import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
INCLUDE_DIR = REPO / "colmag_ros" / "include"
BUILD_DIR = REPO / "outputs" / "agent_tmp" / "m104r2_f3a_cpp_tests"


def _compile_and_run(name, harness, implementation):
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    executable = BUILD_DIR / name
    compile_result = subprocess.run(
        [
            "g++", "-std=c++11", "-Wall", "-Wextra", "-pedantic", "-pthread",
            "-I", str(INCLUDE_DIR), str(harness), str(implementation),
            "-o", str(executable),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr
    run_result = subprocess.run(
        [str(executable)], cwd=REPO, text=True, capture_output=True, check=False,
        timeout=20,
    )
    assert run_result.returncode == 0, run_result.stderr


def test_ros_free_joint_nudge_trajectory_and_limit_contract():
    _compile_and_run(
        "fr3_trajectory_safety_test",
        REPO / "tests" / "cpp" / "fr3_trajectory_safety_test.cpp",
        REPO / "colmag_ros" / "src" / "fr3_trajectory_safety.cpp",
    )


def test_ros_free_command_replay_one_shot_and_lifecycle_contract():
    _compile_and_run(
        "fr3_hardware_safety_test",
        REPO / "tests" / "cpp" / "fr3_hardware_safety_test.cpp",
        REPO / "colmag_ros" / "src" / "fr3_hardware_safety.cpp",
    )


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
