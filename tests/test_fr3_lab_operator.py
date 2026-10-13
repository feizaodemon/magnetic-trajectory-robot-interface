import json
import os
import signal
import stat
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fr3_lab_operator as operator
import fr3_lab_runtime as runtime


class RecordingRunner:
    def __init__(self, handler=None):
        self.handler = handler or (lambda _args: runtime.CommandResult(1, "", ""))
        self.calls = []

    def run(self, args, timeout=None):
        self.calls.append((tuple(args), timeout))
        return self.handler(tuple(args))


def passing_evidence(now, revision="a" * 40, robot_ip="192.0.2.1"):
    joints = tuple("fr3_joint{}".format(index) for index in range(1, 8))
    return {
        "schema_version": 1,
        "result": "PASS",
        "completed_at_utc": now.isoformat().replace("+00:00", "Z"),
        "git_revision": revision,
        "repository_clean": True,
        "host_fingerprint": "host-hash",
        "robot_ip_sha256": runtime.sha256_text(robot_ip),
        "ros_version": "noetic",
        "franka_control_version": "0.10.1",
        "libfranka_version": "0.13.3",
        "controller_name": runtime.CONTROLLER_NAME,
        "controller_joint_names": list(joints),
        "live_joint_names": list(joints),
        "adapter_joint_names": list(joints),
        "joint_contract_sha256": runtime.joint_contract_fingerprint(
            runtime.CONTROLLER_NAME, runtime.ACTION_NAME, joints
        ),
        "action_name": runtime.ACTION_NAME,
        "adapter_session_present": True,
        "send_goals": False,
        "hardware_execution_enabled": False,
        "hardware_profile": "dry-run",
        "one_shot": True,
        "max_goals": 1,
        "no_task_publisher": True,
        "no_goal_observed": True,
        "no_motion_confirmed": True,
        "max_observed_joint_delta": 0.0,
        "cleanup_complete": True,
    }


def validate(evidence, now, revision="a" * 40, robot_ip="192.0.2.1"):
    repository = runtime.RepositoryState(revision, True, True)
    return runtime.validate_gate_b_evidence(
        evidence=evidence,
        now=now,
        repository=repository,
        host="host-hash",
        robot_ip=robot_ip,
        expected_controller=runtime.CONTROLLER_NAME,
        expected_action=runtime.ACTION_NAME,
        expected_joint_fingerprint=evidence.get("joint_contract_sha256", ""),
        expected_versions=("noetic", "0.10.1", "0.13.3"),
    )


def test_typed_gate_plans_fix_every_internal_gate():
    gate_b = runtime.gate_b_plan()
    assert gate_b.hardware_profile == "dry-run"
    assert gate_b.hardware_execution_enabled is False
    assert gate_b.send_goals is False
    assert gate_b.realtime_config == "enforce"
    assert gate_b.one_shot is True
    assert gate_b.max_goals == 1

    source = operator.gate_c_input_plan(operator.InputSource.MOUSE)
    gate_c = runtime.gate_c_plan(
        source.launch_file, "HEXAGON_TRAJECTORY", "2", source.launch_arguments
    )
    assert gate_c.hardware_profile == "fr3-hardware-active"
    assert gate_c.hardware_execution_enabled is True
    assert gate_c.send_goals is True
    assert gate_c.realtime_config == "enforce"
    assert gate_c.one_shot is True
    assert gate_c.max_goals == 1
    assert gate_c.primitive == "HEXAGON_TRAJECTORY"
    assert gate_c.expected_label == "2"


def test_gate_commands_use_only_reviewed_launch_arguments():
    gate_b = runtime.gate_b_plan().launch_command("secret-ip")
    assert "colmag_fr3_hardware_stack.launch" in gate_b
    assert "hardware_profile:=dry-run" in gate_b
    assert "hardware_execution_enabled:=false" in gate_b
    assert "send_goals:=false" in gate_b
    assert "one_shot_hardware_mode:=true" in gate_b
    assert "max_motion_goals_per_process:=1" in gate_b

    source = operator.gate_c_input_plan(operator.InputSource.MOUSE)
    gate_c = runtime.gate_c_plan(
        source.launch_file, "HEXAGON_TRAJECTORY", "2", source.launch_arguments
    ).launch_command("secret-ip")
    assert "colmag_mouse_fr3_hardware_demo.launch" in gate_c
    assert "hardware_profile:=fr3-hardware-active" in gate_c
    assert "hardware_execution_enabled:=true" in gate_c
    assert "send_goals:=true" in gate_c
    assert not any("joint_nudge_delta_rad" in argument for argument in gate_c)
    assert "one_shot_hardware_mode:=true" in gate_c
    assert "max_motion_goals_per_process:=1" in gate_c
    assert all("home" not in arg.lower() and "recovery" not in arg.lower() for arg in gate_c)


@pytest.mark.parametrize(
    "choice,expected",
    [("1", operator.InputSource.MOUSE), ("2", operator.InputSource.REAL_BOARD)],
)
def test_gate_c_input_selection_has_two_explicit_choices(choice, expected):
    lab = operator.Fr3LabOperator(
        REPO, input_func=lambda _prompt: choice, interactive=lambda: True, environ={}
    )
    assert lab._select_input_source() is expected


@pytest.mark.parametrize("choice", ["", "3", "mouse"])
def test_gate_c_input_selection_has_no_default_or_free_form_value(choice):
    lab = operator.Fr3LabOperator(
        REPO, input_func=lambda _prompt: choice, interactive=lambda: True, environ={}
    )
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        lab._select_input_source()
    assert blocked.value.reason == "input_source_invalid"


def test_gate_c_input_selection_cannot_come_from_environment():
    lab = operator.Fr3LabOperator(
        REPO,
        input_func=lambda _prompt: "1",
        interactive=lambda: True,
        environ={"COLMAG_GATE_C_INPUT_SOURCE": "real-board"},
    )
    assert lab._select_input_source() is operator.InputSource.MOUSE


def test_mouse_and_real_board_plans_share_the_same_typed_action_contract():
    mouse = operator.gate_c_input_plan(operator.InputSource.MOUSE)
    board = operator.gate_c_input_plan(operator.InputSource.REAL_BOARD)
    assert mouse.launch_file == "colmag_mouse_fr3_hardware_demo.launch"
    assert mouse.launch_arguments == ()
    assert mouse.display_name == "Mouse"
    assert mouse.requires_serial_preflight is False
    assert board.launch_file == "colmag_real_board_fr3_hardware_demo.launch"
    assert board.launch_arguments == (
        "board_source:=serial",
        "baudrate:=921600",
    )
    assert board.display_name == "Real magnetic board"
    assert board.requires_serial_preflight is True
    action = operator.gate_c_action_plan(operator.GateCAction.HEXAGON)
    assert action.expected_label == "2"
    assert action.expected_task == "HEXAGON_TRAJECTORY"


def test_real_board_gate_plan_keeps_existing_active_one_shot_gates():
    source = operator.gate_c_input_plan(operator.InputSource.REAL_BOARD)
    plan = runtime.gate_c_plan(
        source.launch_file, "HEXAGON_TRAJECTORY", "2", source.launch_arguments
    )
    command = plan.launch_command("secret-ip")
    assert "colmag_real_board_fr3_hardware_demo.launch" in command
    assert "board_source:=serial" in command
    assert "baudrate:=921600" in command
    assert not any(argument.startswith("port:=") for argument in command)
    assert "hardware_profile:=fr3-hardware-active" in command
    assert "hardware_execution_enabled:=true" in command
    assert "send_goals:=true" in command
    assert "realtime_config:=enforce" in command
    assert "one_shot_hardware_mode:=true" in command
    assert "max_motion_goals_per_process:=1" in command


def _fake_device_stat(mode=stat.S_IFCHR):
    return os.stat_result((mode, 0, 0, 0, 0, 0, 0, 0, 0, 0))


def test_unique_by_id_serial_device_is_resolved_without_hardcoded_tty_path():
    resolved = operator.resolve_serial_device(
        input_func=lambda _prompt: pytest.fail("unexpected serial selection prompt"),
        interactive=lambda: True,
        glob_func=lambda pattern: (
            ["/dev/serial/by-id/usb-COLMAG-board"] if "by-id" in pattern else []
        ),
    )
    assert resolved == "/dev/serial/by-id/usb-COLMAG-board"
    runner = RecordingRunner(lambda _args: runtime.CommandResult(1, "", ""))
    assert operator.serial_preflight(
        resolved,
        runner,
        stat_func=lambda _path: _fake_device_stat(),
        access_func=lambda _path, _mode: True,
        which_func=lambda _name: "/usr/bin/fuser",
    ) == "passed"
    assert runner.calls[0][0] == ("/usr/bin/fuser", resolved)


def test_unique_acm_or_usb_fallback_is_resolved():
    resolved = operator.resolve_serial_device(
        input_func=lambda _prompt: pytest.fail("unexpected serial selection prompt"),
        interactive=lambda: True,
        glob_func=lambda pattern: ["/dev/ttyUSB7"] if pattern == "/dev/ttyUSB*" else [],
    )
    assert resolved == "/dev/ttyUSB7"


def test_serial_resolution_blocks_when_no_candidate_exists():
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        operator.resolve_serial_device(
            input_func=lambda _prompt: "1",
            interactive=lambda: True,
            glob_func=lambda _pattern: [],
        )
    assert blocked.value.reason == "serial_device_missing"


def test_multiple_serial_candidates_require_one_explicit_selection():
    prompts = []
    resolved = operator.resolve_serial_device(
        input_func=lambda prompt: prompts.append(prompt) or "2",
        interactive=lambda: True,
        glob_func=lambda pattern: (
            ["/dev/ttyACM0", "/dev/ttyACM2"] if pattern == "/dev/ttyACM*" else []
        ),
    )
    assert resolved == "/dev/ttyACM2"
    assert prompts == ["Select serial device number: "]


def test_selected_serial_device_permission_denied_is_blocked():
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        operator.serial_preflight(
            "/dev/ttyUSB7",
            RecordingRunner(),
            stat_func=lambda _path: _fake_device_stat(),
            access_func=lambda _path, _mode: False,
        )
    assert blocked.value.reason == "serial_device_permission_denied"


def test_selected_serial_device_busy_is_blocked_without_killing_owner():
    runner = RecordingRunner(lambda _args: runtime.CommandResult(0, "1234", ""))
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        operator.serial_preflight(
            "/dev/ttyACM2",
            runner,
            stat_func=lambda _path: _fake_device_stat(),
            access_func=lambda _path, _mode: True,
            which_func=lambda _name: "/usr/bin/fuser",
        )
    assert blocked.value.reason == "serial_device_busy"
    assert all("kill" not in " ".join(call[0]) for call in runner.calls)


class PlanningOperator(operator.Fr3LabOperator):
    def _run_gate_c(
        self,
        plan,
        source_plan,
        action_plan,
        serial_preflight_status,
        robot_ip,
        expected_joint_contract,
    ):
        self.captured_gate_c = (
            plan,
            source_plan,
            action_plan,
            serial_preflight_status,
            robot_ip,
            expected_joint_contract,
        )
        return runtime.EXIT_SUCCESS


def _ready_gate_c_operator(monkeypatch, tmp_path, choices, **kwargs):
    now = datetime(2026, 7, 14, 12, 0, tzinfo=timezone.utc)
    repository = runtime.RepositoryState("a" * 40, True, True)
    monkeypatch.setattr(operator, "is_native_linux", lambda: True)
    monkeypatch.setattr(operator, "repository_state", lambda _runner, _repo: repository)
    monkeypatch.setattr(operator, "ros_noetic_available", lambda _runner: True)
    monkeypatch.setattr(operator, "franka_stack_available", lambda _runner, _repo: True)
    monkeypatch.setattr(operator, "detect_control_processes", lambda _runner: ())
    monkeypatch.setattr(operator, "ros_master_available", lambda _runner: False)
    monkeypatch.setattr(operator, "host_fingerprint", lambda: "host-hash")
    monkeypatch.setattr(
        operator, "installed_versions", lambda _runner: ("noetic", "0.10.1", "0.13.3")
    )
    choice_iter = iter(choices)
    lab = PlanningOperator(
        REPO,
        input_func=lambda _prompt: next(choice_iter),
        now=lambda: now,
        interactive=lambda: True,
        network_probe=lambda _robot_ip: True,
        environ={"FR3_ROBOT_IP": "192.0.2.1"},
        **kwargs,
    )
    lab.evidence_path = tmp_path / "gate_b.json"
    runtime.write_private_json(lab.evidence_path, passing_evidence(now))
    return lab


def test_gate_b_evidence_is_checked_before_input_source_selection(tmp_path):
    input_calls = []
    lab = operator.Fr3LabOperator(
        REPO,
        input_func=lambda prompt: input_calls.append(prompt) or "1",
        interactive=lambda: True,
        environ={},
    )
    lab.evidence_path = tmp_path / "missing_gate_b.json"
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        lab.gate_c()
    assert blocked.value.reason == "gate_b_evidence_missing"
    assert input_calls == []


@pytest.mark.parametrize(
    "choice,expected_launch,expected_source",
    [
        ("1", "colmag_mouse_fr3_hardware_demo.launch", operator.InputSource.MOUSE),
        (
            "2",
            "colmag_real_board_fr3_hardware_demo.launch",
            operator.InputSource.REAL_BOARD,
        ),
    ],
)
def test_gate_c_launches_only_the_selected_input_route(
    monkeypatch, tmp_path, capsys, choice, expected_launch, expected_source
):
    lab = _ready_gate_c_operator(
        monkeypatch,
        tmp_path,
        [choice, "2", "ARM ONE SHOT"],
        serial_glob=lambda pattern: (
            ["/dev/serial/by-id/usb-COLMAG-board"]
            if choice == "2" and "by-id" in pattern
            else []
        ),
        serial_stat=lambda _path: _fake_device_stat(),
        serial_access=lambda _path, _mode: True,
        serial_which=lambda _name: None,
    )
    assert lab.gate_c() == runtime.EXIT_SUCCESS
    plan, source, action, preflight, _robot_ip, _contract = lab.captured_gate_c
    assert plan.launch_file == expected_launch
    assert source.input_source is expected_source
    assert action.expected_task == "HEXAGON_TRAJECTORY"
    assert preflight == ("passed" if choice == "2" else "not_applicable")
    assert sum(argument.startswith("port:=") for argument in plan.launch_arguments) == (
        1 if choice == "2" else 0
    )
    output = capsys.readouterr().out
    assert 'Draw "2"' not in output  # printed only after launch succeeds


def test_board_preflight_failure_never_launches_or_falls_back(
    monkeypatch, tmp_path
):
    lab = _ready_gate_c_operator(
        monkeypatch,
        tmp_path,
        ["2", "2"],
        serial_glob=lambda pattern: (
            ["/dev/ttyUSB7"] if pattern == "/dev/ttyUSB*" else []
        ),
        serial_stat=lambda _path: _fake_device_stat(),
        serial_access=lambda _path, _mode: False,
        serial_which=lambda _name: None,
    )
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        lab.gate_c()
    assert blocked.value.reason == "serial_device_permission_denied"
    assert not hasattr(lab, "captured_gate_c")


@pytest.mark.parametrize("command,expected", [("gate-b", 20), ("gate-c", 24), ("status", 0)])
def test_cli_dispatches_only_three_commands(monkeypatch, command, expected):
    monkeypatch.setattr(operator.Fr3LabOperator, command.replace("-", "_"), lambda _self: expected)
    assert operator.main([command]) == expected


@pytest.mark.parametrize(
    "argv",
    [
        [],
        ["unknown"],
        ["gate-b", "--send-goals"],
        ["gate-c", "--hardware-execution-enabled"],
        ["gate-c", "--hardware-profile", "active"],
        ["gate-c", "--delta", "0.01"],
        ["gate-c", "--source", "mouse"],
        ["gate-c", "--board-source", "serial"],
        ["gate-c", "--port", "/dev/ttyACM0"],
        ["gate-c", "--baud", "921600"],
        ["gate-b", "--robot-ip", "192.0.2.1"],
        ["gate-c", "--force"],
    ],
)
def test_cli_rejects_missing_unknown_and_internal_overrides(argv):
    assert operator.main(argv) == runtime.EXIT_USAGE


def test_shell_wrapper_preserves_usage_exit_code():
    result = subprocess.run(
        [str(SCRIPTS / "fr3_lab.sh"), "unknown"],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == runtime.EXIT_USAGE


def test_evidence_is_private_and_contains_hash_not_robot_or_session(tmp_path):
    path = tmp_path / "latest_gate_b.json"
    value = passing_evidence(datetime.now(timezone.utc), robot_ip="198.51.100.44")
    runtime.write_private_json(path, value)
    stored = path.read_text()
    assert "198.51.100.44" not in stored
    assert "adapter_session_id" not in stored
    assert "robot_ip_sha256" in stored
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert set(json.loads(stored)) == {
        "schema_version", "result", "completed_at_utc", "git_revision",
        "repository_clean", "host_fingerprint", "robot_ip_sha256", "ros_version",
        "franka_control_version", "libfranka_version", "controller_name",
        "controller_joint_names", "live_joint_names", "adapter_joint_names",
        "joint_contract_sha256", "action_name", "adapter_session_present",
        "send_goals", "hardware_execution_enabled", "hardware_profile", "one_shot",
        "max_goals", "no_task_publisher", "no_goal_observed", "no_motion_confirmed",
        "max_observed_joint_delta", "cleanup_complete",
    }


def test_valid_evidence_passes():
    now = datetime(2026, 7, 14, 12, 0, tzinfo=timezone.utc)
    evidence = passing_evidence(now)
    assert validate(evidence, now + timedelta(minutes=10)) == ""


@pytest.mark.parametrize(
    "field,value",
    [
        ("git_revision", "b" * 40),
        ("host_fingerprint", "different-host"),
        ("robot_ip_sha256", "different-robot"),
        ("joint_contract_sha256", "different-joints"),
        ("controller_name", "different_controller"),
        ("action_name", "/different/action"),
        ("cleanup_complete", False),
        ("no_task_publisher", False),
        ("no_goal_observed", False),
        ("ros_version", "melodic"),
        ("franka_control_version", "different"),
        ("libfranka_version", "different"),
    ],
)
def test_evidence_environment_mismatch_is_fail_closed(field, value):
    now = datetime(2026, 7, 14, 12, 0, tzinfo=timezone.utc)
    evidence = passing_evidence(now)
    expected_fingerprint = evidence["joint_contract_sha256"]
    evidence[field] = value
    repository = runtime.RepositoryState("a" * 40, True, True)
    reason = runtime.validate_gate_b_evidence(
        evidence,
        now + timedelta(minutes=1),
        repository,
        "host-hash",
        "192.0.2.1",
        runtime.CONTROLLER_NAME,
        runtime.ACTION_NAME,
        expected_fingerprint,
        ("noetic", "0.10.1", "0.13.3"),
    )
    assert reason == "gate_b_environment_changed"


def test_expired_evidence_is_distinct():
    now = datetime(2026, 7, 14, 12, 0, tzinfo=timezone.utc)
    evidence = passing_evidence(now)
    assert validate(evidence, now + timedelta(minutes=31)) == "gate_b_evidence_expired"


def test_dirty_repository_invalidates_evidence():
    now = datetime(2026, 7, 14, 12, 0, tzinfo=timezone.utc)
    evidence = passing_evidence(now)
    reason = runtime.validate_gate_b_evidence(
        evidence,
        now,
        runtime.RepositoryState("a" * 40, False, True),
        "host-hash",
        "192.0.2.1",
        runtime.CONTROLLER_NAME,
        runtime.ACTION_NAME,
        evidence["joint_contract_sha256"],
        ("noetic", "0.10.1", "0.13.3"),
    )
    assert reason == "gate_b_environment_changed"


def test_missing_and_invalid_evidence_are_blocked(tmp_path):
    with pytest.raises(runtime.OperatorBlocked) as missing:
        runtime.load_gate_b_evidence(tmp_path / "missing.json")
    assert missing.value.reason == "gate_b_evidence_missing"
    invalid = tmp_path / "invalid.json"
    invalid.write_text("not-json")
    with pytest.raises(runtime.OperatorBlocked):
        runtime.load_gate_b_evidence(invalid)


def test_adapter_config_fixes_hexagon_offsets_below_operator_limit():
    config = runtime.read_adapter_config(REPO)
    assert runtime.operator_hexagon_offsets(config) == pytest.approx(
        (0.08, 0.10)
    )


@pytest.mark.parametrize("value", [0.0, -0.01, 0.120001, float("nan"), "invalid"])
def test_operator_rejects_hexagon_offset_outside_reviewed_limit(value):
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        runtime.operator_hexagon_offsets(
            {"hexagon_primary_offset_rad": value, "hexagon_secondary_offset_rad": 0.01}
        )
    assert blocked.value.reason == "hexagon_offset_invalid"


@pytest.mark.parametrize(
    "terminal,goals,attempts,reason",
    [
        ("FINAL_STATE_VERIFIED", 1, 1, "pass"),
        ("FINAL_STATE_VERIFIED", 2, 1, "multiple_goals_observed"),
        ("FINAL_STATE_VERIFIED", 1, 2, "multiple_goals_observed"),
        ("FINAL_STATE_VERIFIED", 0, 1, "goal_count_mismatch"),
        ("GOAL_ABORTED", 1, 1, "goal_aborted"),
        ("GOAL_TIMEOUT", 1, 1, "goal_timeout"),
        ("FINAL_STATE_VERIFY_FAILED", 1, 1, "final_state_verify_failed"),
        ("SHUTDOWN_CANCELLED", 1, 1, "shutdown_cancelled"),
    ],
)
def test_gate_c_requires_exactly_one_verified_goal(terminal, goals, attempts, reason):
    assert runtime.gate_c_outcome_reason(
        terminal, goals, attempts, "HEXAGON_TRAJECTORY", "HEXAGON_TRAJECTORY"
    ) == reason


class FakeProcess:
    def __init__(self, wait_results):
        self.pid = 4321
        self.stdout = iter(())
        self.wait_results = list(wait_results)
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        result = self.wait_results.pop(0)
        if result == "timeout":
            raise subprocess.TimeoutExpired("fake", timeout)
        self.returncode = 0
        return 0


def make_owned_process(tmp_path, wait_results):
    process = FakeProcess(wait_results)
    signals = []
    log = runtime.DiagnosticLog(tmp_path / "owned.log", ("secret-ip",))
    owner = runtime.OwnedProcess(
        ["fake", "robot_ip:=secret-ip"],
        log,
        ("secret-ip",),
        popen_factory=lambda *args, **kwargs: process,
        killpg=lambda pgid, sig: signals.append((pgid, sig)),
        getpgid=lambda _pid: 4321,
    )
    return owner, process, signals, log


def test_owned_cleanup_uses_sigint_only_when_it_succeeds(tmp_path):
    owner, _process, signals, log = make_owned_process(tmp_path, ["done"])
    assert owner.cleanup()
    log.close()
    assert signals == [(4321, signal.SIGINT)]


def test_owned_cleanup_has_bounded_sigterm_fallback(tmp_path):
    owner, _process, signals, log = make_owned_process(tmp_path, ["timeout", "done"])
    assert owner.cleanup(interrupt_timeout=0.01, term_timeout=0.01)
    log.close()
    assert signals == [(4321, signal.SIGINT), (4321, signal.SIGTERM)]


def test_cleanup_failure_prevents_success(tmp_path):
    owner, _process, signals, log = make_owned_process(tmp_path, ["timeout", "timeout"])
    assert not owner.cleanup(interrupt_timeout=0.01, term_timeout=0.01)
    log.close()
    assert signals == [(4321, signal.SIGINT), (4321, signal.SIGTERM)]


def test_goal_observer_cleanup_targets_only_its_process_group(tmp_path, monkeypatch):
    process = FakeProcess(["done"])
    signals = []
    monkeypatch.setattr(runtime.subprocess, "Popen", lambda *args, **kwargs: process)
    monkeypatch.setattr(runtime.os, "getpgid", lambda _pid: 4321)
    monkeypatch.setattr(runtime.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    log = runtime.DiagnosticLog(tmp_path / "observer.log", ())
    observer = runtime.GoalObserver(runtime.ACTION_NAME + "/goal", log)
    assert observer.cleanup()
    log.close()
    assert signals == [(4321, signal.SIGINT)]


def test_diagnostic_log_redacts_robot_ip_and_command_fingerprint(tmp_path):
    log = runtime.DiagnosticLog(tmp_path / "redacted.log", ("198.51.100.8",))
    log.write(
        'robot=198.51.100.8 FR3_ROBOT_IP=198.51.100.8 '
        '"adapter_session_id":"full-session-value"'
    )
    log.close()
    text = (tmp_path / "redacted.log").read_text()
    assert "198.51.100.8" not in text
    assert "full-session-value" not in text
    assert "<redacted>" in text
    first = runtime.command_fingerprint(["launch", "robot_ip:=198.51.100.8"], ("198.51.100.8",))
    second = runtime.command_fingerprint(["launch", "robot_ip:=203.0.113.8"], ("203.0.113.8",))
    assert first == second


def test_status_is_read_only_and_tolerates_unavailable_ros(tmp_path, capsys):
    revision = "c" * 40

    def handler(args):
        joined = " ".join(args)
        if "rev-parse HEAD" in joined:
            return runtime.CommandResult(0, revision + "\n", "")
        if "status --porcelain" in joined or "diff --cached --name-only" in joined:
            return runtime.CommandResult(0, "", "")
        return runtime.CommandResult(1, "", "unavailable")

    runner = RecordingRunner(handler)
    lab = operator.Fr3LabOperator(tmp_path, runner=runner, environ={})
    assert lab.status() == 0
    output = capsys.readouterr().out
    assert "FR3_LAB_STATUS" in output
    assert "ROS master: unavailable" in output
    assert "Gate C input source: not-running" in output
    assert "Gate C actions: 1=Left, 2=Hexagon, 3=Right" in output
    assert "Real board device: unavailable" in output
    assert len(output.strip().splitlines()) <= 20
    invoked = "\n".join(" ".join(call[0]) for call in runner.calls)
    for forbidden in ("roslaunch", "rostopic pub", "rosservice call", "rosparam set", "recovery"):
        assert forbidden not in invoked
    assert not (tmp_path / "outputs").exists()


def test_status_reports_available_board_without_opening_serial(tmp_path, capsys):
    revision = "c" * 40

    def handler(args):
        joined = " ".join(args)
        if "rev-parse HEAD" in joined:
            return runtime.CommandResult(0, revision + "\n", "")
        if "status --porcelain" in joined or "diff --cached --name-only" in joined:
            return runtime.CommandResult(0, "", "")
        return runtime.CommandResult(1, "", "unavailable")

    lab = operator.Fr3LabOperator(
        tmp_path,
        runner=RecordingRunner(handler),
        environ={},
        serial_glob=lambda pattern: ["/dev/ttyUSB7"] if pattern == "/dev/ttyUSB*" else [],
        serial_stat=lambda _path: _fake_device_stat(),
        serial_access=lambda _path, _mode: True,
        serial_which=lambda _name: pytest.fail("status must not run fuser"),
    )
    assert lab.status() == 0
    assert "Real board device: available" in capsys.readouterr().out


def test_joint_and_controller_parsers_accept_ros_cli_shapes():
    controller_text = '''
controller:
- name: "franka_state_controller"
  state: "running"
- name: "position_joint_trajectory_controller"
  state: "running"
'''
    assert runtime.parse_controller_states(controller_text) == {
        "franka_state_controller": "running",
        "position_joint_trajectory_controller": "running",
    }
    message = '''
name:
  - fr3_joint1
  - fr3_joint2
position: [0.1, -0.2]
'''
    assert runtime.parse_named_sequence(message, "name") == ["fr3_joint1", "fr3_joint2"]
    assert runtime.parse_named_sequence(message, "position") == [0.1, -0.2]


def test_task_publisher_parser_distinguishes_none_and_named_nodes():
    assert runtime.parse_ros_topic_publishers("Publishers: None\nSubscribers: None\n") == ()
    text = "Publishers:\n * /task_dispatcher_node (http://host:123/)\nSubscribers: None\n"
    assert runtime.parse_ros_topic_publishers(text) == ("/task_dispatcher_node",)


def test_active_goal_detection_is_fail_closed_for_pending_and_active_codes():
    pending = RecordingRunner(lambda _args: runtime.CommandResult(0, "status: 0\n", ""))
    succeeded = RecordingRunner(lambda _args: runtime.CommandResult(0, "status: 3\n", ""))
    unavailable = RecordingRunner()
    assert runtime.active_goal_present(pending)
    assert not runtime.active_goal_present(succeeded)
    assert not runtime.active_goal_present(unavailable)


def test_confirmation_phrases_are_exact(tmp_path):
    lab = operator.Fr3LabOperator(tmp_path, input_func=lambda _prompt: "almost")
    with pytest.raises(runtime.OperatorBlocked) as blocked:
        lab._require_confirmation("", "CHECK ONLY", "physical_confirmation_missing")
    assert blocked.value.exit_code == runtime.EXIT_CONFIRMATION


def test_signal_guard_restores_handlers_and_reports_interrupt():
    previous = signal.getsignal(signal.SIGTERM)
    with operator.signal_guard():
        handler = signal.getsignal(signal.SIGTERM)
        with pytest.raises(runtime.OperatorInterrupted):
            handler(signal.SIGTERM, None)
    assert signal.getsignal(signal.SIGTERM) == previous
