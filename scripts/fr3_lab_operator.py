#!/usr/bin/env python3
"""Native-lab operator facade for the bounded COLMAG real-FR3 route."""

from __future__ import annotations

import argparse
import getpass
import glob
import os
import shutil
import signal
import stat
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

from fr3_lab_runtime import (
    ACTION_NAME,
    CONTROLLER_NAME,
    EXIT_CLEANUP,
    EXIT_CONFIRMATION,
    EXIT_GATE_B_EVIDENCE,
    EXIT_INTERNAL,
    EXIT_PREREQUISITE,
    EXIT_RUNTIME_EVIDENCE,
    EXIT_SUCCESS,
    EXIT_USAGE,
    OPERATOR_SESSION_TIMEOUT_SECONDS,
    TERMINAL_STATES,
    CommandRunner,
    DiagnosticLog,
    GatePlan,
    GoalObserver,
    OperatorBlocked,
    OperatorInterrupted,
    OwnedProcess,
    _enabled_value,
    _goal_count_value,
    _int_value,
    _yes_no,
    active_goal_present,
    build_gate_b_evidence,
    collect_live_contract,
    configured_joint_contract,
    detect_control_processes,
    docker_image_revision,
    evidence_summary,
    franka_stack_available,
    gate_b_plan,
    gate_c_outcome_reason,
    gate_c_plan,
    host_fingerprint,
    installed_versions,
    is_native_linux,
    load_gate_b_evidence,
    maximum_joint_delta,
    observe_one_message,
    operator_hexagon_offsets,
    probe_franka_port,
    read_adapter_config,
    read_adapter_state,
    read_controllers,
    read_joint_sample,
    repository_state,
    ros_master_available,
    ros_nodes,
    ros_noetic_available,
    task_publishers,
    terminal_state,
    validate_gate_b_evidence,
    validate_graph_contract,
    validate_live_contract,
    write_private_json,
    joint_contract_fingerprint,
)


class InputSource(Enum):
    MOUSE = "mouse"
    REAL_BOARD = "real-board"


class GateCAction(Enum):
    LEFT = "left"
    HEXAGON = "hexagon"
    RIGHT = "right"


@dataclass(frozen=True)
class GateCInputPlan:
    input_source: InputSource
    launch_file: str
    launch_arguments: Tuple[str, ...]
    display_name: str
    requires_serial_preflight: bool


@dataclass(frozen=True)
class GateCActionPlan:
    action: GateCAction
    expected_label: str
    expected_task: str
    display_name: str
    motion_type: str


def gate_c_input_plan(input_source: InputSource) -> GateCInputPlan:
    if input_source is InputSource.MOUSE:
        return GateCInputPlan(
            input_source=input_source,
            launch_file="colmag_mouse_fr3_hardware_demo.launch",
            launch_arguments=(),
            display_name="Mouse",
            requires_serial_preflight=False,
        )
    return GateCInputPlan(
        input_source=input_source,
        launch_file="colmag_real_board_fr3_hardware_demo.launch",
        launch_arguments=(
            "board_source:=serial",
            "baudrate:=921600",
        ),
        display_name="Real magnetic board",
        requires_serial_preflight=True,
    )


def gate_c_action_plan(action: GateCAction) -> GateCActionPlan:
    values = {
        GateCAction.LEFT: ("1", "MOVE_LEFT", "Left and return",
                           "positive joint excursion and return"),
        GateCAction.HEXAGON: ("2", "HEXAGON_TRAJECTORY", "Hexagon",
                              "closed joint-space hexagon and return"),
        GateCAction.RIGHT: ("3", "MOVE_RIGHT", "Right and return",
                            "negative joint excursion and return"),
    }
    label, task, display_name, motion_type = values[action]
    return GateCActionPlan(action, label, task, display_name, motion_type)


def resolve_serial_device(
    input_func: Callable[[str], str],
    interactive: Callable[[], bool],
    glob_func: Callable[[str], Sequence[str]] = glob.glob,
) -> str:
    by_id = sorted(set(glob_func("/dev/serial/by-id/*")))
    if len(by_id) == 1:
        return by_id[0]

    fallback = sorted(set(
        list(glob_func("/dev/ttyACM*")) + list(glob_func("/dev/ttyUSB*"))
    ))
    if len(fallback) == 1:
        return fallback[0]
    candidates = fallback if fallback else by_id
    if not candidates:
        raise OperatorBlocked("serial_device_missing", EXIT_PREREQUISITE)
    if not interactive():
        raise OperatorBlocked("serial_device_ambiguous", EXIT_CONFIRMATION)
    print("Available serial devices:")
    for index, candidate in enumerate(candidates, 1):
        print("  {}) {}".format(index, candidate))
    choice = input_func("Select serial device number: ")
    try:
        selected = int(choice)
    except (TypeError, ValueError):
        selected = 0
    if selected < 1 or selected > len(candidates):
        raise OperatorBlocked("serial_device_ambiguous", EXIT_CONFIRMATION)
    return candidates[selected - 1]


def serial_preflight(
    device: str,
    runner: CommandRunner,
    stat_func: Callable[[str], os.stat_result] = os.stat,
    access_func: Callable[[str, int], bool] = os.access,
    which_func: Callable[[str], Optional[str]] = shutil.which,
) -> str:
    try:
        device_stat = stat_func(device)
    except (FileNotFoundError, OSError):
        raise OperatorBlocked("serial_device_missing", EXIT_PREREQUISITE)
    if not stat.S_ISCHR(device_stat.st_mode):
        raise OperatorBlocked("serial_device_not_character", EXIT_PREREQUISITE)
    if not access_func(device, os.R_OK | os.W_OK):
        raise OperatorBlocked("serial_device_permission_denied", EXIT_PREREQUISITE)
    fuser = which_func("fuser")
    if fuser:
        result = runner.run([fuser, device], timeout=3.0)
        if result.returncode != 1:
            raise OperatorBlocked("serial_device_busy", EXIT_PREREQUISITE)
    return "passed"


def real_board_device_available(
    glob_func: Callable[[str], Sequence[str]] = glob.glob,
    stat_func: Callable[[str], os.stat_result] = os.stat,
    access_func: Callable[[str, int], bool] = os.access,
) -> bool:
    candidates = set(glob_func("/dev/serial/by-id/*"))
    candidates.update(glob_func("/dev/ttyACM*"))
    candidates.update(glob_func("/dev/ttyUSB*"))
    for candidate in candidates:
        try:
            if stat.S_ISCHR(stat_func(candidate).st_mode) and access_func(
                candidate, os.R_OK | os.W_OK
            ):
                return True
        except (FileNotFoundError, OSError):
            continue
    return False


class Fr3LabOperator:
    def __init__(
        self,
        repo: Path,
        runner: Optional[CommandRunner] = None,
        input_func: Callable[[str], str] = input,
        secret_input: Callable[[str], str] = getpass.getpass,
        now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        interactive: Callable[[], bool] = lambda: sys.stdin.isatty() and sys.stdout.isatty(),
        network_probe: Optional[Callable[[str], bool]] = None,
        environ: Optional[Mapping[str, str]] = None,
        serial_glob: Callable[[str], Sequence[str]] = glob.glob,
        serial_stat: Callable[[str], os.stat_result] = os.stat,
        serial_access: Callable[[str, int], bool] = os.access,
        serial_which: Callable[[str], Optional[str]] = shutil.which,
    ):
        self.repo = repo.resolve()
        self.runner = runner or CommandRunner()
        self.input = input_func
        self.secret_input = secret_input
        self.now = now
        self.monotonic = monotonic
        self.sleep = sleep
        self.interactive = interactive
        self.network_probe = network_probe or probe_franka_port
        self.environ = dict(os.environ if environ is None else environ)
        self.serial_glob = serial_glob
        self.serial_stat = serial_stat
        self.serial_access = serial_access
        self.serial_which = serial_which
        self.verbose = self.environ.get("COLMAG_FR3_LAB_VERBOSE") == "1"
        self.evidence_path = self.repo / "outputs/runtime_logs/fr3_lab/latest_gate_b.json"
        self.log_dir = self.repo / "outputs/runtime_logs/m104r2_f3e"

    def _select_input_source(self) -> InputSource:
        if not self.interactive():
            raise OperatorBlocked("input_source_invalid", EXIT_CONFIRMATION)
        print("Select input source:\n")
        print("  1) Mouse")
        print("  2) Real magnetic board\n")
        choice = self.input("Enter 1 or 2: ")
        if choice == "1":
            return InputSource.MOUSE
        if choice == "2":
            return InputSource.REAL_BOARD
        raise OperatorBlocked("input_source_invalid", EXIT_CONFIRMATION)

    def _select_action(self) -> GateCAction:
        if not self.interactive():
            raise OperatorBlocked("action_selection_invalid", EXIT_CONFIRMATION)
        print("Select action:\n")
        print("  1) Left and return")
        print("  2) Hexagon")
        print("  3) Right and return\n")
        choice = self.input("Enter 1, 2 or 3: ")
        actions = {
            "1": GateCAction.LEFT,
            "2": GateCAction.HEXAGON,
            "3": GateCAction.RIGHT,
        }
        if choice not in actions:
            raise OperatorBlocked("action_selection_invalid", EXIT_CONFIRMATION)
        return actions[choice]

    def gate_b(self) -> int:
        plan = gate_b_plan()
        robot_ip = self._gate_prerequisites("gate-b")
        self._require_confirmation(
            "Gate B only:\n"
            "- robot connection check\n"
            "- controllers and joints inspection\n"
            "- no motion command\n"
            "- no recovery\n"
            "- hardware E-stop must remain reachable\n",
            "CHECK ONLY",
            "physical_confirmation_missing",
        )
        log = self._new_log("gate_b", robot_ip)
        owner: Optional[OwnedProcess] = None
        cleanup_complete = False
        evidence: Optional[Dict[str, object]] = None
        blocked: Optional[OperatorBlocked] = None
        try:
            owner = OwnedProcess(plan.launch_command(robot_ip), log, (robot_ip,))
            self._wait_for_ros_contract(owner)
            start = read_joint_sample(self.runner)
            live = collect_live_contract(self.runner)
            validate_live_contract(live, plan)
            validate_graph_contract(self.runner, expect_task_publisher=False)
            if active_goal_present(self.runner):
                raise OperatorBlocked("active_goal_present", EXIT_RUNTIME_EVIDENCE)
            if observe_one_message(self.runner, ACTION_NAME + "/goal", 15.0):
                raise OperatorBlocked("fjt_goal_observed", EXIT_RUNTIME_EVIDENCE)
            end = read_joint_sample(self.runner)
            max_delta = maximum_joint_delta(start, end)
            self._require_confirmation(
                "Gate B checks complete. Confirm the robot did not move.",
                "NO MOTION OBSERVED",
                "no_motion_confirmation_missing",
            )
            evidence = build_gate_b_evidence(
                now=self.now(),
                repository=repository_state(self.runner, self.repo),
                robot_ip=robot_ip,
                live=live,
                max_observed_joint_delta=max_delta,
                cleanup_complete=True,
            )
        except OperatorBlocked as exc:
            blocked = exc
            log.write("gate_b_blocked reason=" + exc.reason)
        finally:
            cleanup_complete = owner is not None and owner.cleanup()
            log.close()
        if not cleanup_complete:
            return self._blocked("gate-b", "cleanup_incomplete", EXIT_CLEANUP)
        if blocked is not None:
            return self._blocked("gate-b", blocked.reason, blocked.exit_code)
        if evidence is None:
            return self._blocked("gate-b", "runtime_evidence_failed", EXIT_RUNTIME_EVIDENCE)
        evidence["cleanup_complete"] = True
        write_private_json(self.evidence_path, evidence)
        print("GATE_B_PASS")
        return EXIT_SUCCESS

    def gate_c(self) -> int:
        evidence = load_gate_b_evidence(self.evidence_path)
        robot_ip = self._gate_prerequisites("gate-c")
        repository = repository_state(self.runner, self.repo)
        config = read_adapter_config(self.repo)
        operator_hexagon_offsets(config)
        current_contract = configured_joint_contract(self.repo)
        reason = validate_gate_b_evidence(
            evidence=evidence,
            now=self.now(),
            repository=repository,
            host=host_fingerprint(),
            robot_ip=robot_ip,
            expected_controller=CONTROLLER_NAME,
            expected_action=ACTION_NAME,
            expected_joint_fingerprint=current_contract,
            expected_versions=installed_versions(self.runner),
        )
        if reason:
            return self._blocked("gate-c", reason, EXIT_GATE_B_EVIDENCE)
        if detect_control_processes(self.runner):
            return self._blocked("gate-c", "gate_b_environment_changed", EXIT_GATE_B_EVIDENCE)
        if ros_master_available(self.runner):
            if task_publishers(self.runner):
                return self._blocked("gate-c", "gate_b_environment_changed", EXIT_GATE_B_EVIDENCE)
            if active_goal_present(self.runner):
                return self._blocked("gate-c", "active_goal_present", EXIT_GATE_B_EVIDENCE)
        input_source = self._select_input_source()
        action_plan = gate_c_action_plan(self._select_action())
        source_plan = gate_c_input_plan(input_source)
        serial_device = ""
        serial_preflight_status = "not_applicable"
        launch_arguments = source_plan.launch_arguments
        if source_plan.requires_serial_preflight:
            serial_device = resolve_serial_device(
                self.input, self.interactive, self.serial_glob
            )
            serial_preflight_status = serial_preflight(
                serial_device,
                self.runner,
                self.serial_stat,
                self.serial_access,
                self.serial_which,
            )
            launch_arguments += ("port:=" + serial_device,)
        plan = gate_c_plan(
            source_plan.launch_file,
            action_plan.expected_task,
            action_plan.expected_label,
            launch_arguments,
        )
        arm_id = str(config.get("arm_id", "fr3"))
        print("Gate C — one supervised motion\n")
        print("Input source: {}".format(source_plan.display_name))
        if serial_device:
            print("Serial device: {}".format(serial_device))
            print("Baudrate: 921600")
        print("Expected recognition label: {}".format(action_plan.expected_label))
        print("Robot: configured")
        print("Git revision: {}".format(repository.revision[:12]))
        print("Controller: {}".format(CONTROLLER_NAME))
        print("Primitive: {}".format(action_plan.expected_task))
        print("Type: {}".format(action_plan.motion_type))
        if action_plan.action is GateCAction.HEXAGON:
            print("Joints: {}_joint6 / {}_joint7".format(arm_id, arm_id))
        else:
            print("Joint: {}".format(config.get("target_joint_name", arm_id + "_joint1")))
        print("Maximum per-joint offset: 0.12 rad")
        print("Expected FJT goals: 1")
        print("Automatic homing: disabled")
        print("Automatic recovery: disabled")
        print("Hardware E-stop: required")
        self._require_confirmation("", "ARM ONE SHOT", "physical_confirmation_missing")
        return self._run_gate_c(
            plan,
            source_plan,
            action_plan,
            serial_preflight_status,
            robot_ip,
            str(evidence["joint_contract_sha256"]),
        )

    def _run_gate_c(
        self,
        plan: GatePlan,
        source_plan: GateCInputPlan,
        action_plan: GateCActionPlan,
        serial_preflight_status: str,
        robot_ip: str,
        expected_joint_contract: str,
    ) -> int:
        log = self._new_log("gate_c", robot_ip)
        log.write(
            "gate_c_plan input_source={} launch_file={} expected_label={} "
            "expected_task={} serial_preflight={}".format(
                source_plan.input_source.value,
                source_plan.launch_file,
                action_plan.expected_label,
                action_plan.expected_task,
                serial_preflight_status,
            )
        )
        owner: Optional[OwnedProcess] = None
        observer: Optional[GoalObserver] = None
        cleanup_complete = False
        observer_cleanup = True
        outcome_reason = "runtime_evidence_failed"
        success = False
        try:
            owner = OwnedProcess(plan.launch_command(robot_ip), log, (robot_ip,))
            self._wait_for_ros_contract(owner)
            live = collect_live_contract(self.runner)
            validate_live_contract(live, plan)
            validate_graph_contract(self.runner, expect_task_publisher=True)
            if joint_contract_fingerprint(CONTROLLER_NAME, ACTION_NAME, live.controller_joints) != expected_joint_contract:
                raise OperatorBlocked("gate_b_environment_changed", EXIT_GATE_B_EVIDENCE)
            observer = GoalObserver(ACTION_NAME + "/goal", log)
            input_name = "the mouse" if source_plan.input_source is InputSource.MOUSE \
                else "the real magnetic board"
            print('Draw "{}" once with {}.'.format(action_plan.expected_label, input_name))
            print("Confirm {} once in the dashboard.".format(action_plan.expected_task))
            print("No second motion will be accepted.")
            terminal, goal_count, send_attempts, observed_task = \
                self._wait_for_gate_c_terminal(
                    observer, owner, action_plan.expected_task
                )
            log.write("gate_c_observed observed_task=" + observed_task)
            outcome_reason = gate_c_outcome_reason(
                terminal, goal_count, send_attempts, observed_task,
                action_plan.expected_task,
            )
            if outcome_reason == "pass":
                self._require_confirmation(
                    "Confirm the selected motion completed and returned to its action start.",
                    "ONE SHOT COMPLETE",
                    "physical_confirmation_missing",
                )
                latest = read_adapter_state(self.runner, timeout=2.0)
                latest_attempts = _int_value(latest.get("motion_goal_send_attempts"), 0)
                outcome_reason = gate_c_outcome_reason(
                    terminal_state(latest) or terminal,
                    observer.count,
                    latest_attempts,
                    str(latest.get("task", observed_task)),
                    action_plan.expected_task,
                )
                success = outcome_reason == "pass"
        except OperatorBlocked as exc:
            outcome_reason = exc.reason
            log.write("gate_c_blocked reason=" + exc.reason)
        finally:
            cleanup_complete = owner is not None and owner.cleanup()
            if observer is not None:
                observer_cleanup = observer.cleanup()
            log.close()
        if not cleanup_complete or not observer_cleanup:
            return self._blocked("gate-c", "cleanup_incomplete", EXIT_CLEANUP)
        if success and observer is not None and observer.count != 1:
            success = False
            outcome_reason = "multiple_goals_observed" if observer.count > 1 else "goal_count_mismatch"
        if not success:
            code = EXIT_CONFIRMATION if outcome_reason == "physical_confirmation_missing" else EXIT_RUNTIME_EVIDENCE
            return self._blocked("gate-c", outcome_reason, code)
        print("GATE_C_PASS")
        return EXIT_SUCCESS

    def _wait_for_gate_c_terminal(
        self, observer: GoalObserver, owner: OwnedProcess, expected_task: str
    ) -> Tuple[str, int, int, str]:
        deadline = self.monotonic() + OPERATOR_SESSION_TIMEOUT_SECONDS
        last_state: Mapping[str, object] = {}
        observed_task = ""
        while self.monotonic() < deadline:
            if owner.poll() is not None:
                raise OperatorBlocked("runtime_process_exited", EXIT_RUNTIME_EVIDENCE)
            state = read_adapter_state(self.runner, timeout=2.0)
            if state:
                last_state = state
                task = str(state.get("task", "")).strip()
                if task not in ("", "NO_OP", "STOP_OR_CANCEL"):
                    observed_task = task
            send_attempts = _int_value(last_state.get("motion_goal_send_attempts"), 0)
            observed_goals = observer.count
            if observed_goals > 1 or send_attempts > 1:
                print("Hardware E-stop may be required.")
                return "", max(observed_goals, 2), send_attempts, observed_task
            if observed_task and observed_task != expected_task:
                return "", observed_goals, send_attempts, observed_task
            terminal = terminal_state(last_state)
            if terminal in TERMINAL_STATES:
                return terminal, observed_goals, send_attempts, observed_task
            self.sleep(0.5)
        return (
            "GOAL_TIMEOUT",
            observer.count,
            _int_value(last_state.get("motion_goal_send_attempts"), 0),
            observed_task,
        )

    def status(self) -> int:
        try:
            repository = repository_state(self.runner, self.repo)
        except OperatorBlocked:
            print("STATUS_UNAVAILABLE: repository_unavailable")
            return EXIT_PREREQUISITE
        evidence = load_gate_b_evidence(self.evidence_path, required=False)
        evidence_status = evidence_summary(evidence, self.now())
        state: Mapping[str, object] = {}
        master = ros_master_available(self.runner)
        controller = "unknown"
        joint_contract = "unknown"
        active_goal = "unknown"
        input_source = "not-running"
        if master:
            state = read_adapter_state(self.runner, timeout=1.0)
            nodes = ros_nodes(self.runner)
            controllers = read_controllers(self.runner)
            controller = "running" if controllers.get(CONTROLLER_NAME) == "running" else "stopped"
            active_goal = "yes" if active_goal_present(self.runner) else "no"
            if state:
                expected = _string_tuple(state.get("expected_joint_names"))
                observed = _string_tuple(state.get("observed_joint_names"))
                joint_contract = "matched" if expected and expected == observed else "mismatch"
                if state.get("hardware_profile") == "fr3-hardware-active":
                    input_source = (
                        "real-board"
                        if "/serial_packet_publisher_node" in nodes
                        else "mouse"
                    )
        docker_revision = docker_image_revision(self.runner)
        board_device = "available" if real_board_device_available(
            self.serial_glob, self.serial_stat, self.serial_access
        ) else "unavailable"
        print("FR3_LAB_STATUS")
        print("Backend: native")
        print("Git revision: {}".format(repository.revision[:12]))
        print("Repository: {}".format("clean" if repository.clean else "dirty"))
        print("ROS master: {}".format("available" if master else "unavailable"))
        print("Last Gate B: {}".format(evidence_status["result"]))
        print("Gate C actions: 1=Left, 2=Hexagon, 3=Right")
        print("Gate C input source: {}".format(input_source))
        print("Real board device: {}".format(board_device))
        print("Gate B revision match: {}".format(_yes_no(evidence and evidence.get("git_revision") == repository.revision)))
        print("Gate B host match: {}".format(_yes_no(evidence and evidence.get("host_fingerprint") == host_fingerprint())))
        print("Current profile: {}".format(state.get("hardware_profile", "not-running") if state else "not-running"))
        print("Hardware execution: {}".format(_enabled_value(state.get("hardware_execution_enabled"))))
        print("Goal sending: {}".format(_enabled_value(state.get("send_goals"))))
        print("Controller: {}".format(controller))
        print("Joint contract: {}".format(joint_contract))
        print("Adapter session: {}".format("present" if state.get("adapter_session_id") else ("absent" if state else "unknown")))
        print("Motion goals used: {}".format(_goal_count_value(state)))
        print("Active goal: {}".format(active_goal))
        print("Docker image revision: {}".format(docker_revision))
        return EXIT_SUCCESS

    def _gate_prerequisites(self, command: str) -> str:
        if not is_native_linux():
            raise OperatorBlocked("not_native_linux", EXIT_PREREQUISITE)
        repository = repository_state(self.runner, self.repo)
        if not repository.clean or not repository.staged_empty:
            raise OperatorBlocked("repository_dirty", EXIT_PREREQUISITE)
        if not self.interactive():
            reason = "input_source_invalid" if command == "gate-c" else "interactive_terminal_required"
            raise OperatorBlocked(reason, EXIT_PREREQUISITE)
        if not ros_noetic_available(self.runner):
            raise OperatorBlocked("ros_noetic_missing", EXIT_PREREQUISITE)
        if not franka_stack_available(self.runner, self.repo):
            raise OperatorBlocked("franka_stack_missing", EXIT_PREREQUISITE)
        if detect_control_processes(self.runner):
            raise OperatorBlocked("unknown_control_process", EXIT_PREREQUISITE)
        if ros_master_available(self.runner) and task_publishers(self.runner):
            raise OperatorBlocked("task_publisher_present", EXIT_PREREQUISITE)
        robot_ip = self.environ.get("FR3_ROBOT_IP", "").strip()
        if not robot_ip:
            robot_ip = self.secret_input("FR3 robot IP (input hidden): ").strip()
        if not robot_ip:
            raise OperatorBlocked("robot_ip_missing", EXIT_PREREQUISITE)
        if not self.network_probe(robot_ip):
            raise OperatorBlocked("robot_unreachable", EXIT_PREREQUISITE)
        return robot_ip

    def _require_confirmation(self, text: str, phrase: str, reason: str) -> None:
        if text:
            print(text)
        response = self.input("Type {}: ".format(phrase))
        if response != phrase:
            raise OperatorBlocked(reason, EXIT_CONFIRMATION)

    def _wait_for_ros_contract(self, owner: OwnedProcess) -> None:
        deadline = self.monotonic() + 30.0
        while self.monotonic() < deadline:
            if owner.poll() is not None:
                raise OperatorBlocked("runtime_process_exited", EXIT_RUNTIME_EVIDENCE)
            if ros_master_available(self.runner):
                nodes = ros_nodes(self.runner)
                controllers = read_controllers(self.runner)
                if (
                    "/colmag_fr3_task_trajectory_adapter" in nodes
                    and controllers.get("franka_state_controller") == "running"
                    and controllers.get(CONTROLLER_NAME) == "running"
                ):
                    return
            self.sleep(0.5)
        raise OperatorBlocked("controller_not_running", EXIT_RUNTIME_EVIDENCE)

    def _new_log(self, command: str, robot_ip: str) -> DiagnosticLog:
        stamp = self.now().strftime("%Y%m%dT%H%M%SZ")
        log = DiagnosticLog(self.log_dir / "{}_{}.log".format(command, stamp), (robot_ip,))
        log.write("operator_command={} robot=configured".format(command))
        return log

    def _blocked(self, command: str, reason: str, exit_code: int) -> int:
        prefix = "GATE_B_BLOCKED" if command == "gate-b" else "GATE_C_BLOCKED"
        print("{}: {}".format(prefix, reason))
        return exit_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fr3_lab.sh",
        description="Native COLMAG FR3 lab operator facade.",
    )
    parser.add_argument("command", choices=("gate-b", "gate-c", "status"))
    return parser


def signal_guard():
    class Guard:
        def __enter__(self):
            self.previous = {
                sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)
            }

            def interrupted(signum, _frame):
                raise OperatorInterrupted("signal {}".format(signum))

            for sig in self.previous:
                signal.signal(sig, interrupted)
            return self

        def __exit__(self, _type, _value, _traceback):
            for sig, handler in self.previous.items():
                signal.signal(sig, handler)
            return False

    return Guard()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return EXIT_SUCCESS if exc.code == 0 else EXIT_USAGE
    repo = Path(__file__).resolve().parents[1]
    operator = Fr3LabOperator(repo)
    try:
        if args.command == "status":
            return operator.status()
        with signal_guard():
            return operator.gate_b() if args.command == "gate-b" else operator.gate_c()
    except OperatorBlocked as exc:
        prefix = "GATE_B_BLOCKED" if args.command == "gate-b" else "GATE_C_BLOCKED"
        print("{}: {}".format(prefix, exc.reason))
        return exc.exit_code
    except OperatorInterrupted:
        prefix = "GATE_B_BLOCKED" if args.command == "gate-b" else "GATE_C_BLOCKED"
        print("{}: operator_interrupted".format(prefix))
        return EXIT_RUNTIME_EVIDENCE
    except Exception as exc:
        if os.environ.get("COLMAG_FR3_LAB_VERBOSE") == "1":
            print("internal diagnostic: {}".format(exc), file=sys.stderr)
        prefix = "GATE_B_BLOCKED" if args.command == "gate-b" else "GATE_C_BLOCKED"
        print("{}: internal_error".format(prefix))
        return EXIT_INTERNAL


if __name__ == "__main__":
    sys.exit(main())
