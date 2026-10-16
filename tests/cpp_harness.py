"""Compile and run the ROS-free C++ tests under ``tests/cpp/`` with g++.

Each test is ``tests/cpp/<test_name>.cpp`` linked with the named sources from
``colmag_ros/src/``. The program must exit with status 0.

A test skips instead of failing when g++ is missing, or when one of its files
includes a Boost header and the Boost headers are not installed. CI sets
``COLMAG_REQUIRE_CPP_DEPS=1`` so a missing toolchain fails there instead.
"""

import functools
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
INCLUDE_DIR = REPO / "colmag_ros" / "include"
CPP_TESTS = REPO / "tests" / "cpp"
SRC = REPO / "colmag_ros" / "src"

_BOOST_INCLUDE = re.compile(r"^\s*#\s*include\s*<boost/", re.MULTILINE)


def _skip_or_fail(reason):
    if os.environ.get("COLMAG_REQUIRE_CPP_DEPS") == "1":
        pytest.fail(reason + " (COLMAG_REQUIRE_CPP_DEPS=1)")
    pytest.skip(reason)


@functools.lru_cache(maxsize=None)
def _boost_headers_available():
    with tempfile.TemporaryDirectory() as tmp:
        probe = Path(tmp) / "probe.cpp"
        probe.write_text("#include <boost/uuid/uuid.hpp>\nint main() { return 0; }\n")
        result = subprocess.run(
            ["g++", "-fsyntax-only", str(probe)],
            capture_output=True,
            check=False,
        )
    return result.returncode == 0


def _needs_boost(paths):
    return any(_BOOST_INCLUDE.search(path.read_text()) for path in paths)


def compile_and_run(build_dir, test_name, *sources, std="c++11"):
    if shutil.which("g++") is None:
        _skip_or_fail("g++ is not installed")
    files = [CPP_TESTS / (test_name + ".cpp"), *(SRC / source for source in sources)]
    if _needs_boost(files) and not _boost_headers_available():
        _skip_or_fail("Boost headers are not installed (apt install libboost-dev)")

    executable = Path(build_dir) / test_name
    compile_result = subprocess.run(
        [
            "g++", "-std=" + std, "-Wall", "-Wextra", "-pedantic", "-pthread",
            "-I", str(INCLUDE_DIR), *(str(path) for path in files),
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
