"""Compile and run the ROS-free C++ tests under ``tests/cpp/`` with g++.

Each test is ``tests/cpp/<test_name>.cpp`` linked with the named sources from
``colmag_ros/src/``. The program must exit with status 0.
"""

import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
INCLUDE_DIR = REPO / "colmag_ros" / "include"
CPP_TESTS = REPO / "tests" / "cpp"
SRC = REPO / "colmag_ros" / "src"


def compile_and_run(build_dir, test_name, *sources, std="c++11"):
    executable = Path(build_dir) / test_name
    compile_result = subprocess.run(
        [
            "g++", "-std=" + std, "-Wall", "-Wextra", "-pedantic", "-pthread",
            "-I", str(INCLUDE_DIR), str(CPP_TESTS / (test_name + ".cpp")),
            *(str(SRC / source) for source in sources),
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
