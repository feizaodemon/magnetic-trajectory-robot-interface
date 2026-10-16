# Design decisions

This page answers "why is it built this way" for the main choices in the repository. Each entry gives the decision, the alternatives, the trade-off, and where the evidence lives. Components and topics are described in [Architecture](ARCHITECTURE.md); this page does not repeat them. Where the repository does not record a reason, the entry says "not documented" instead of guessing.

## 1. DTW template bank as the runtime recognizer

- **Decision.** Strokes are normalized (centroid, max-abs scale, 64-point arc-length resampling) and ranked by DTW distance against a tracked JSON bank of 124 templates over 8 symbols (`colmag_ros/config/dtw_banks/`).
- **Alternatives.** Derivative DTW, windowed DTW, Protractor ($1), $P and feature k-NN were implemented behind one interface in [`gesture_recognizers.py`](../colmag_ros/src/colmag_ros/gesture_recognizers.py) and benchmarked offline. Only DTW runs in the ROS pipeline. An older 4-label bank and the OCR/"lightweight" backend display paths were removed as unused (commits `ebee2c1`, `0b11c43`).
- **Trade-off.** On clean leave-one-out queries every method scores 100%, so the bank alone cannot rank them. Under synthetic perturbations, Protractor matches or beats DTW everywhere except truncated strokes, at about 1/200 of the cost; DTW is described as "a solid second". Early truncation is the weak point of every shape matcher.
- **Why DTW was chosen originally:** not documented. The comparison was added later (commit `4f13ef3`) and explicitly leaves the runtime unchanged.
- **Evidence.** [Recognizer comparison](RECOGNIZER_COMPARISON.md), [README Results](../README.md#results), `scripts/eval_dtw_bank_loo.py`, CI job `dtw-eval` (fails below 95% Top-1).

## 2. Exact DTW by default, Sakoe-Chiba window as opt-in

- **Decision.** `dtw_template_window` defaults to 0, the full DTW search (commit `8e3cfee`).
- **Alternative.** A 4-point Sakoe-Chiba window.
- **Trade-off.** The window is about 5.7x faster with Top-1 still 100% on the leave-one-out run, but it reorders the 2nd and 3rd candidates on about half of the queries, and it is weaker on truncated strokes (8.9% instead of 16.9% at 40% truncation).
- **Evidence.** [README](../README.md#results), `scripts/bench_dtw_ranking.py`, [Recognizer comparison](RECOGNIZER_COMPARISON.md#what-the-numbers-say).

## 3. Confidence and margin gates on the ranking

- **Decision.** A candidate list is accepted only if the top-1 confidence and the top-1/top-2 margin clear thresholds; otherwise it is flagged `low_confidence` or `low_margin` ([`recognition_uncertainty.py`](../colmag_ros/src/colmag_ros/recognition_uncertainty.py)). The recognizer launch sets `dtw_template_min_confidence` 0.30 and `dtw_template_min_margin` 0.01 ([`dtw_template_bank_recognizer.launch`](../colmag_ros/launch/dtw_template_bank_recognizer.launch)).
- **Trade-off.** The stated purpose is to reject unclear samples ([README](../README.md#highlights)) rather than show a weak guess. On the leave-one-out run, 100% of queries pass the gates and all accepted ones are correct, so the bank data does not exercise the rejection path.
- **How the threshold values were chosen:** not documented.
- **Evidence.** [README Results](../README.md#results), `scripts/eval_dtw_bank_loo.py` (reports gate acceptance).

## 4. Confirmation-gated command path in separate ROS nodes

- **Decision.** Recognition, confirmation, dispatch and execution are separate nodes. The recognizer publishes `/colmag/symbol_candidates`; the dashboard publishes `/colmag/confirmed_label` only after `C Confirm`; the dispatcher is the only publisher of `/colmag/task_command`.
- **Alternative.** Not documented as a tried design; the rule is stated as a contract: "No recognizer component owns `/colmag/task_command`."
- **Trade-off.** Every task costs an explicit human step, and candidates from an earlier sample cannot be confirmed for the current one (sample/sequence identity). In exchange, a misrecognition cannot move the robot by itself.
- **Fail closed at startup.** The dispatcher used to fall back silently to the default mapping on an invalid `~label_to_task_json` or unknown `~dispatcher_mapping_name`; a typo could therefore send tasks from a mapping the operator did not configure. Both now raise and stop the node (commits `0c5953a`, `2863862`).
- **Evidence.** [Architecture](ARCHITECTURE.md#supervised-control-path), [Interaction Profiles](INTERACTION_PROFILES.md#shared-confirmation-contract), `tests/test_preview_confirm_seam_static.py` (no `task_command` publisher in the dashboard even with every publish gate on, commit `0282d16`).

## 5. TASK/TELEOP mode ownership, independent of backend authorization

- **Decision.** A latched `/colmag/control_mode` with exactly `TASK` and `TELEOP`. `TELEOP` blocks new task dispatch. Selecting a mode never selects a backend, sets `allow_real_robot`, or authorizes hardware. Invalid mode state fails closed.
- **Trade-off.** Mode switches are command-owner transitions only (discrete adapters relinquish their own goals; continuous control waits for fresh input and joint state). They are explicitly not an emergency stop.
- **Evidence.** [Dual-Mode Teleoperation](DUAL_MODE_TELEOPERATION.md#ownership-transitions), [Hardware Boundaries](HARDWARE_BOUNDARIES.md#default-state), CI `gazebo-smoke` (TASK at startup rejects gripper commands; TELEOP moves; TASK again rejects).

## 6. One C++ Cartesian core with damped-least-squares IK for Gazebo and Real FR3

- **Decision.** `Fr3CartesianTeleopCore` ([`fr3_cartesian_teleop_core.cpp`](../colmag_ros/src/fr3_cartesian_teleop_core.cpp)) owns destination filtering, bounded target progression, Franka forward kinematics, a numerical Jacobian, DLS inverse kinematics (`dls_damping: 0.03` in [`colmag_fr3_cartesian_teleop.yaml`](../colmag_ros/config/colmag_fr3_cartesian_teleop.yaml)), workspace and joint bounds, residual rejection and finite-output checks. One adapter drives both the Gazebo Panda and the Real-FR3 route.
- **Alternatives / why DLS and a shared core:** not documented beyond the statement that one core serves both backends. A consequence visible in the repo: the Gazebo runtime and the ROS-free C++ test (`tests/cpp/fr3_cartesian_teleop_core_test.cpp`) exercise the same code the Real-FR3 adapter links.
- **Iteration that is documented.** Keyboard input first went through the board's position filter and point-to-point S-curve. That made held keys cruise at about 0.03 m/s, overshoot about 11 cm on a direction change, and stutter on terminal auto-repeat. Keyboard input is now a velocity command with acceleration and jerk limits and braking before workspace bounds; the board path keeps its filter and S-curve (commit `b7232a0`).
- **Evidence.** [Dual-Mode Teleoperation](DUAL_MODE_TELEOPERATION.md#gazebo-teleop-route), CI `catkin-build` and `gazebo-smoke`.

## 7. Separate, defensive FR3 adapter instead of reusing the Gazebo bridge

- **Decision.** Gazebo tasks go through a simulation-only bridge; FR3 tasks go through a C++ adapter with allowlists, freshness and deduplication, joint-name and limit checks, bounded trajectories, goal ownership and final-state checks. Safety helpers are ROS-free C++ with their own tests.
- **Trade-off.** Two backends to maintain, but "the same task name does not imply the same safety qualification across simulation and hardware."
- **Why C++ for the hardware layer:** not documented.
- **Evidence.** [Architecture](ARCHITECTURE.md#execution-separation), [Hardware Boundaries](HARDWARE_BOUNDARIES.md#fr3-adapter), `tests/cpp/`, `tests/test_fr3_safety_modules.py`.

## 8. Recognition and dashboard logic kept ROS-free, plain Python

- **Decision.** Recognition, gates and most dashboard state live in ROS-free modules under `colmag_ros/src/colmag_ros/`; nodes in `colmag_ros/scripts/` are thin. The recognizers use no NumPy.
- **Documented reasons.** The recognizers are pure Python "so they run in the same Python 3.8 environment as the ROS nodes" ([Recognizer comparison](RECOGNIZER_COMPARISON.md#what-is-compared)). ROS-free code lets the bank evaluation and the comparison run without ROS, in CI jobs that have no ROS, and lets `mypy` check those modules ([`mypy.ini`](../mypy.ini)).
- **Iteration.** Dashboard tests that asserted on method source text broke on harmless refactors and missed behavior changes; they were replaced by a headless harness with fake `rospy`, `std_msgs` and `tkinter` (commits `0282d16`, `3380292`), and dwell and preview state moved into plain objects (commit `d8e4a95`).
- **Trade-off.** Pure-Python DTW costs about 83 ms median per query on one core, versus 0.4 ms for Protractor in the comparison.

## 9. ROS1 Noetic, packaged in Docker with revision labels

- **Decision.** The runtime is ROS1 Noetic (Python 3.8) with `franka_ros` and `libfranka`. Gazebo and FR3 have separate Dockerfiles and Compose profiles; images carry `org.opencontainers.image.revision`, and the FR3 wrapper gates on provenance.
- **Why ROS1 Noetic rather than ROS 2:** not documented. The repository is a portfolio edition of a collaborative project ([NOTICE](../NOTICE.md#provenance)).
- **Consequences that are documented.** Python 3.8 compatibility is enforced in CI (`ruff` rule FA102 with target py38, `mypy` against 3.8, a `pytest-noetic` job in `ros:noetic-ros-base`). Connected FR3 runs require a clean checkout and an image whose OCI revision matches `HEAD` ([Hardware Boundaries](HARDWARE_BOUNDARIES.md#required-connected-run-gates)). The Gazebo image build retries apt because `snapshots.ros.org` returned intermittent HTTP 500s (commits `60a5b0b`, `0aa10f6`).
- **Evidence.** [docker/README](../docker/README.md), [Validation](VALIDATION.md#continuous-integration).

## Known limitations and next steps

These come from the repository's own limitation sections; the "next" column names only the gap each one leaves.

| Documented limitation | What it leaves open |
| --- | --- |
| All 124 templates were mouse-drawn by one person; results are an upper bound, not accuracy for other writers or the board ([README](../README.md#limitations)). | Measuring accuracy on other writers and on the magnetic board. |
| Perturbations are synthetic; real board error "is not known to look like any of them" ([Recognizer comparison](RECOGNIZER_COMPARISON.md#limits)). | Evaluating recognizers on recorded board data. |
| Early truncation is the weak point of every recognizer ([Recognizer comparison](RECOGNIZER_COMPARISON.md#what-the-numbers-say)). | The doc points at the interaction design (clutch, explicit `B Recognize`) as much as the recognizer. |
| Seed JSONs are not published, so the bank cannot be rebuilt from this repository ([Running](RUNNING.md#dtw-template-bank)). | Publishing seeds or another rebuildable bank source. |
| Real-FR3 adapters compile and are in the launch graph, but physical Real-FR3 `TELEOP` was not run, and there is no Real-FR3 gripper route ([Dual-Mode Teleoperation](DUAL_MODE_TELEOPERATION.md#real-fr3-teleop-route)). | A supervised physical run under the [connected-run gates](HARDWARE_BOUNDARIES.md#required-connected-run-gates). |
| The Gazebo smoke test is one short scripted run; it does not cover the GUI, timing under load or grasp quality ([Validation](VALIDATION.md#evidence-boundaries)). | Broader runtime coverage. |
| The board clutch is an application convention, not a safety-rated sensor ([Interaction Profiles](INTERACTION_PROFILES.md#board-profile)). | It stays an interaction aid, not a safety function. |
