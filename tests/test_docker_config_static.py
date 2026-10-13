"""Static packaging checks for M104-DOCKER2; starts no container."""

import os
import shutil
import subprocess
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[1]
DOCKERFILE = (REPO / "docker" / "Dockerfile.noetic-gazebo").read_text()
COMPOSE = yaml.safe_load((REPO / "docker" / "compose.yaml").read_text())
FR3_DOCKERFILE = (REPO / "docker" / "Dockerfile.noetic-fr3").read_text()
FR3_COMPOSE = yaml.safe_load((REPO / "docker" / "compose.fr3.yaml").read_text())
DOCKERIGNORE = (REPO / ".dockerignore").read_text()
ENTRYPOINT = (REPO / "docker" / "entrypoint.sh").read_text()
LAUNCHER = REPO / "docker" / "run_gazebo_demo.sh"
FR3_LAUNCHER = REPO / "docker" / "run_fr3_demo.sh"


def _head_or_empty_history_placeholder():
    result = subprocess.run(
        ["git", "rev-parse", "--verify", "HEAD"],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "0" * 40


HEAD = _head_or_empty_history_placeholder()


def _write_executable(path, text):
    path.write_text(text)
    path.chmod(0o755)


def _write_fake_git(bin_dir):
    _write_executable(
        bin_dir / "git",
        """#!/usr/bin/env bash
case "$*" in
  *"branch --show-current"*) echo main ;;
  *"rev-parse HEAD"*) echo "$FAKE_GIT_HEAD" ;;
  *"status --porcelain"*) printf '%s' "$FAKE_GIT_DIRTY" ;;
  *) exit 2 ;;
esac
""",
    )


def _write_fake_docker(bin_dir):
    _write_executable(
        bin_dir / "docker",
        """#!/usr/bin/env bash
printf '%s\n' "$*" >> "$FAKE_DOCKER_LOG"
if [[ "$1" == info ]]; then exit "${FAKE_DOCKER_INFO:-0}"; fi
if [[ "$1 $2" == "image inspect" ]]; then
  [[ -n "${FAKE_IMAGE_REVISION:-}" ]] || exit 1
  echo "$FAKE_IMAGE_REVISION"
  exit 0
fi
if [[ "$1" == compose && " $* " == *" ps "* ]]; then
  printf '%s' "$FAKE_RUNNING_SERVICES"
  exit 0
fi
if [[ "$1" == compose ]]; then exit 0; fi
exit 2
""",
    )


def _write_fake_getent(bin_dir):
    _write_executable(
        bin_dir / "getent",
        """#!/usr/bin/env bash
[[ "$1 $2" == "group dialout" && -n "${FAKE_DIALOUT_GID:-}" ]] || exit 2
echo "dialout:x:${FAKE_DIALOUT_GID}:"
""",
    )


def _copy_fr3_fixture(tmp_path):
    docker_dir = tmp_path / "repository" / "docker"
    docker_dir.mkdir(parents=True)
    launcher = shutil.copy2(FR3_LAUNCHER, docker_dir / "run_fr3_demo.sh")
    shutil.copy2(REPO / "docker" / "compose.fr3.yaml", docker_dir / "compose.fr3.yaml")
    return Path(launcher), docker_dir / "fr3-hardware.env"


def _file_identity(path):
    metadata = path.stat()
    return path.read_bytes(), metadata.st_ino, metadata.st_mtime_ns, metadata.st_mode


def _fake_cli(
    tmp_path, monkeypatch, *, revision=HEAD, running="", dirty="", dialout="20",
    docker_info="0",
):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "docker.log"
    _write_fake_git(bin_dir)
    _write_fake_docker(bin_dir)
    _write_fake_getent(bin_dir)
    monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ['PATH']}")
    monkeypatch.setenv("FAKE_DOCKER_LOG", str(log))
    monkeypatch.setenv("FAKE_GIT_HEAD", HEAD)
    monkeypatch.setenv("FAKE_GIT_DIRTY", dirty)
    monkeypatch.setenv("FAKE_IMAGE_REVISION", revision)
    monkeypatch.setenv("FAKE_RUNNING_SERVICES", running)
    monkeypatch.setenv("FAKE_DIALOUT_GID", dialout)
    monkeypatch.setenv("FAKE_DOCKER_INFO", docker_info)
    return log


def _run_fr3(command, tmp_path, monkeypatch, env_text=None, **fake_options):
    tmp_path.mkdir(parents=True, exist_ok=True)
    log = _fake_cli(tmp_path, monkeypatch, **fake_options)
    launcher, fixture_env = _copy_fr3_fixture(tmp_path)
    if env_text is not None:
        fixture_env.write_text(env_text)
    result = subprocess.run(
        [str(launcher), command], cwd=tmp_path, text=True,
        capture_output=True, check=False,
    )
    return result, log.read_text() if log.exists() else ""


VALID_ENV = """COLMAG_FR3_DOCKER_IMAGE=magnetic-trajectory-interface:fr3-noetic
FR3_ROBOT_IP=192.0.2.10
COLMAG_HARDWARE_EXECUTION_ENABLED=true
COLMAG_SEND_GOALS=true
COLMAG_MAX_MOTION_GOALS_PER_PROCESS=0
COLMAG_BOARD_DEVICE=/dev/null
"""


def test_image_is_multi_stage_install_space_only():
    assert (
        "osrf/ros:noetic-desktop-full@sha256:"
        "7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797"
    ) in DOCKERFILE
    assert " AS builder" in DOCKERFILE
    assert " AS runtime" in DOCKERFILE
    assert "catkin_make install" in DOCKERFILE
    assert "COPY --from=builder /workspace/catkin_ws/install" in DOCKERFILE
    assert "source /workspace/catkin_ws/install/setup.bash" in DOCKERFILE
    assert "devel/setup.bash" not in DOCKERFILE


def test_gazebo_image_records_exact_source_revision_contract():
    common = COMPOSE["x-gazebo-common"]
    assert COMPOSE["name"] == "magnetic-trajectory-interface-gazebo"
    assert common["image"] == "${COLMAG_DOCKER_IMAGE:-magnetic-trajectory-interface:gazebo-noetic}"
    assert common["build"]["args"]["COLMAG_GIT_SHA"] == "${COLMAG_GIT_SHA:-uncommitted}"
    assert "ARG COLMAG_GIT_SHA=uncommitted" in DOCKERFILE
    assert 'org.opencontainers.image.revision="${COLMAG_GIT_SHA}"' in DOCKERFILE

    assert FR3_COMPOSE["x-fr3-common"]["build"]["args"]["COLMAG_GIT_SHA"] == (
        "${COLMAG_GIT_SHA:-uncommitted}"
    )
    assert 'org.opencontainers.image.revision="${COLMAG_GIT_SHA}"' in FR3_DOCKERFILE


def test_fr3_runtime_keeps_upstream_license_and_notice_files():
    for expected in (
        "COPY --from=builder /opt/src/libfranka/LICENSE /usr/share/doc/libfranka/LICENSE",
        "COPY --from=builder /opt/src/libfranka/NOTICE /usr/share/doc/libfranka/NOTICE",
        "COPY --from=builder /opt/franka_ros_ws/src/franka_ros/LICENSE /usr/share/doc/franka_ros/LICENSE",
        "COPY --from=builder /opt/franka_ros_ws/src/franka_ros/NOTICE /usr/share/doc/franka_ros/NOTICE",
    ):
        assert expected in FR3_DOCKERFILE


def test_compose_profiles_match_official_commands():
    services = COMPOSE["services"]
    assert set(services) == {"mouse-gazebo", "real-board-gazebo"}
    assert services["mouse-gazebo"]["command"] == [
        "roslaunch", "colmag_ros", "colmag_8symbol_gazebo_robot_body_demo.launch"
    ]
    assert services["real-board-gazebo"]["command"] == [
        "roslaunch", "colmag_ros",
        "colmag_real_board_ready_gazebo_robot_body_demo.launch",
        "board_source:=serial", "port:=/dev/ttyACM0", "baudrate:=921600",
    ]


def test_serial_profile_is_narrow_and_unprivileged():
    board = COMPOSE["services"]["real-board-gazebo"]
    assert board["devices"] == ["/dev/ttyACM0:/dev/ttyACM0"]
    assert "group_add" in board
    assert all("privileged" not in service for service in COMPOSE["services"].values())


def test_no_profile_owns_real_fr3_control():
    text = (REPO / "docker" / "compose.yaml").read_text()
    for forbidden in (
        "send_goals=true", "send_goals:=true", "franka_control",
        "controller_manager", "robot_ip", "privileged: true",
    ):
        assert forbidden not in text


def test_build_context_excludes_generated_and_classifier_artifacts():
    for pattern in ("outputs/", "*.joblib", "tests/", "**/logs/"):
        assert pattern in DOCKERIGNORE


def test_entrypoint_defaults_ros_master_uri_and_limits_nounset_relaxation():
    lines = [line.strip() for line in ENTRYPOINT.splitlines() if line.strip()]
    default = 'export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"'
    ros_setup = "source /opt/ros/noetic/setup.bash"
    colmag_setup = "source /workspace/catkin_ws/install/setup.bash"

    assert default in lines
    assert lines.count("set +u") == 1
    assert lines.count("set -u") == 1
    assert lines.index(default) < lines.index("set +u")
    assert lines.index("set +u") < lines.index(ros_setup)
    assert lines.index(ros_setup) < lines.index(colmag_setup)
    assert lines.index(colmag_setup) < lines.index("set -u")
    assert lines.index("set -u") < lines.index('exec "$@"')


def test_entrypoint_preserves_explicit_uri_and_exec_contract():
    assert '${ROS_MASTER_URI:-http://localhost:11311}' in ENTRYPOINT
    assert '${ROS_MASTER_URI:=http://localhost:11311}' not in ENTRYPOINT
    assert 'exec "$@"' in ENTRYPOINT


def test_launcher_is_directly_executable_and_tracked_as_executable():
    assert os.access(LAUNCHER, os.X_OK)
    assert LAUNCHER.stat().st_mode & 0o111
    index_entry = subprocess.check_output(
        ["git", "ls-files", "-s", "--", str(LAUNCHER.relative_to(REPO))],
        cwd=REPO,
        text=True,
    )
    if index_entry.strip():
        assert index_entry.split()[0] == "100755"
    else:
        assert _head_or_empty_history_placeholder() == "0" * 40
    assert "RUN chmod +x /ros_entrypoint_colmag.sh" in DOCKERFILE


def test_ros_log_directory_exists_before_setup_and_compose_launch():
    entrypoint_lines = [line.strip() for line in ENTRYPOINT.splitlines() if line.strip()]
    launcher_lines = [line.strip() for line in LAUNCHER.read_text().splitlines() if line.strip()]
    entrypoint_mkdir = 'mkdir -p "${ROS_LOG_DIR}"'
    launcher_mkdir = 'mkdir -p "${COLMAG_RUNTIME_LOG_DIR}/ros"'

    assert entrypoint_mkdir in entrypoint_lines
    assert entrypoint_lines.index(entrypoint_mkdir) < entrypoint_lines.index("set +u")
    assert launcher_mkdir in launcher_lines
    assert launcher_lines.index(launcher_mkdir) < next(
        index for index, line in enumerate(launcher_lines) if line.startswith("exec docker compose")
    )
    assert "/tmp" not in launcher_mkdir


def test_fr3_launcher_exists_is_executable_and_has_valid_shell_syntax():
    assert FR3_LAUNCHER.is_file()
    assert os.access(FR3_LAUNCHER, os.X_OK)
    assert FR3_LAUNCHER.stat().st_mode & 0o111
    subprocess.run(["bash", "-n", str(FR3_LAUNCHER)], check=True)


def test_fr3_launcher_keeps_the_canonical_script_relative_env_path():
    text = FR3_LAUNCHER.read_text()
    assert 'ENV_FILE="${SCRIPT_DIR}/fr3-hardware.env"' in text


def test_fr3_fixture_leaves_external_lab_env_untouched(tmp_path, monkeypatch):
    lab_env = tmp_path / "external-lab" / "docker" / "fr3-hardware.env"
    lab_env.parent.mkdir(parents=True)
    lab_env.write_text("FR3_ROBOT_IP=198.51.100.77\nLAB_SENTINEL=do-not-touch\n")
    lab_env.chmod(0o640)
    before = _file_identity(lab_env)

    case_dir = tmp_path / "wrapper-case"
    result, docker_log = _run_fr3("dry-run", case_dir, monkeypatch, VALID_ENV)

    assert result.returncode == 0, result.stderr
    assert _file_identity(lab_env) == before
    combined_output = result.stdout + result.stderr + docker_log
    assert "198.51.100.77" not in combined_output
    assert "192.0.2.10" not in combined_output
    fixture_env = case_dir / "repository" / "docker" / "fr3-hardware.env"
    fixture_env.relative_to(tmp_path)
    assert fixture_env.is_file()


def test_fr3_launcher_rejects_invalid_command_without_docker(tmp_path, monkeypatch):
    for help_command in ("help", "-h", "--help"):
        result, docker_log = _run_fr3(help_command, tmp_path / help_command, monkeypatch)
        assert result.returncode == 0
        assert "Usage:" in result.stdout
        assert docker_log == ""
    result, docker_log = _run_fr3("build", tmp_path, monkeypatch)
    assert result.returncode != 0
    assert "Usage:" in result.stderr
    assert docker_log == ""


def test_fr3_launcher_maps_start_commands_from_any_working_directory(tmp_path, monkeypatch):
    expected = {
        "dry-run": "--profile fr3-hardware-dry-run up fr3-hardware-dry-run",
        "mouse": "--profile fr3-hardware-active up fr3-hardware-active",
        "board": "--profile fr3-hardware-active-board up fr3-hardware-active-board",
    }
    for index, (command, compose_tail) in enumerate(expected.items()):
        case_dir = tmp_path / str(index)
        case_dir.mkdir()
        result, docker_log = _run_fr3(command, case_dir, monkeypatch, VALID_ENV)
        assert result.returncode == 0, result.stderr
        fixture_docker = case_dir / "repository" / "docker"
        assert f"--env-file {fixture_docker / 'fr3-hardware.env'}" in docker_log
        assert f"-f {fixture_docker / 'compose.fr3.yaml'}" in docker_log
        assert compose_tail in docker_log


def test_fr3_launcher_has_no_runtime_or_broad_cleanup_commands():
    text = FR3_LAUNCHER.read_text()
    for forbidden in (
        "docker build", " compose build", "roslaunch", "rostopic", "rosservice",
        "FollowJointTrajectory", "privileged", "system prune", "image prune",
        "volume prune", "docker stop", "docker rm", "GATE_B_PASS", "GATE_C_PASS",
    ):
        assert forbidden not in text


def test_fr3_launcher_rejects_missing_env_placeholder_ip_and_dirty_tree(tmp_path, monkeypatch):
    missing, _ = _run_fr3("dry-run", tmp_path / "missing", monkeypatch)
    assert missing.returncode != 0
    placeholder_env = VALID_ENV.replace("192.0.2.10", "__REQUIRED_FR3_ROBOT_IP__")
    placeholder, _ = _run_fr3("dry-run", tmp_path / "placeholder", monkeypatch, placeholder_env)
    assert placeholder.returncode != 0
    dirty, _ = _run_fr3(
        "dry-run", tmp_path / "dirty", monkeypatch, VALID_ENV, dirty=" M README.md\n"
    )
    assert dirty.returncode != 0


def test_fr3_launcher_rejects_missing_or_mismatched_image(tmp_path, monkeypatch):
    missing, missing_log = _run_fr3(
        "dry-run", tmp_path / "missing", monkeypatch, VALID_ENV, revision=""
    )
    assert missing.returncode != 0
    assert " compose " not in f" {missing_log} "
    mismatch, mismatch_log = _run_fr3(
        "dry-run", tmp_path / "mismatch", monkeypatch, VALID_ENV, revision="deadbeef"
    )
    assert mismatch.returncode != 0
    assert " up " not in f" {mismatch_log} "
    uncommitted, uncommitted_log = _run_fr3(
        "dry-run", tmp_path / "uncommitted", monkeypatch, VALID_ENV,
        revision="uncommitted",
    )
    assert uncommitted.returncode != 0
    assert " up " not in f" {uncommitted_log} "


def test_fr3_launcher_requires_both_active_gates(tmp_path, monkeypatch):
    for index, env_text in enumerate((
        VALID_ENV.replace("COLMAG_SEND_GOALS=true", "COLMAG_SEND_GOALS=false"),
        VALID_ENV.replace(
            "COLMAG_HARDWARE_EXECUTION_ENABLED=true",
            "COLMAG_HARDWARE_EXECUTION_ENABLED=TRUE",
        ),
    )):
        result, docker_log = _run_fr3("mouse", tmp_path / str(index), monkeypatch, env_text)
        assert result.returncode != 0
        assert " up " not in f" {docker_log} "


def test_fr3_launcher_validates_board_device_and_dialout(tmp_path, monkeypatch):
    no_device = VALID_ENV.replace("COLMAG_BOARD_DEVICE=/dev/null\n", "")
    result, docker_log = _run_fr3("board", tmp_path / "device", monkeypatch, no_device)
    assert result.returncode != 0
    assert " up " not in f" {docker_log} "
    result, docker_log = _run_fr3(
        "board", tmp_path / "dialout", monkeypatch, VALID_ENV, dialout=""
    )
    assert result.returncode != 0
    assert " up " not in f" {docker_log} "


def test_fr3_launcher_rejects_when_owned_service_is_running(tmp_path, monkeypatch):
    running = "fr3-hardware-dry-run magnetic-trajectory-interface-fr3-fr3-hardware-dry-run-1\n"
    result, docker_log = _run_fr3(
        "mouse", tmp_path, monkeypatch, VALID_ENV, running=running
    )
    assert result.returncode != 0
    assert "fr3-hardware-dry-run" in result.stderr
    assert "run_fr3_demo.sh stop" in result.stderr
    assert " up " not in f" {docker_log} "


def test_fr3_status_is_observational_and_tolerates_missing_dependencies(tmp_path, monkeypatch):
    result, docker_log = _run_fr3(
        "status", tmp_path, monkeypatch, revision="", dialout="", docker_info="1"
    )
    assert result.returncode == 0
    assert "Environment file: no" in result.stdout
    assert "Motion goal quota (0=unlimited): 1" in result.stdout
    assert "Image revision: missing" in result.stdout
    assert " up " not in f" {docker_log} "
    assert " stop " not in f" {docker_log} "


def test_fr3_stop_targets_only_owned_services_and_handles_empty_project(tmp_path, monkeypatch):
    result, docker_log = _run_fr3("stop", tmp_path, monkeypatch)
    assert result.returncode == 0, result.stderr
    assert "stop fr3-hardware-dry-run fr3-hardware-active fr3-hardware-active-board" in docker_log
    assert "down" not in docker_log
    assert "prune" not in docker_log
