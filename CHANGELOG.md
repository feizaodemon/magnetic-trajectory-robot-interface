# Changelog

All notable changes to this repository are listed here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow [Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-10-16

First tagged release of the portfolio edition. It covers the work merged up to pull request #55. Earlier history is in `git log`.

### What this release contains

- **Gesture-to-task pipeline.** A DTW template bank with 124 mouse-drawn templates over 8 symbols ranks each stroke, and confidence and margin gates reject unclear input. Leave-one-out Top-1 accuracy on the bank is 100%, at 83 ms median per query in pure Python ([Results](README.md#results)).
- **Confirmation-gated command path.** Recognition, confirmation, dispatch and execution run as separate ROS nodes. Only the dispatcher publishes `/colmag/task_command`, and only after an explicit `C Confirm` ([Architecture](docs/ARCHITECTURE.md)).
- **Two modes, two backends.** `TASK` and `TELEOP` modes run against a Gazebo Panda or a Franka FR3. One C++ Cartesian core with damped-least-squares IK drives both robots ([Dual-Mode Teleoperation](docs/DUAL_MODE_TELEOPERATION.md)).
- **Browser demo.** The recognizer is ported to the browser and deployed to GitHub Pages: [feizaodemon.github.io/magnetic-trajectory-robot-interface](https://feizaodemon.github.io/magnetic-trajectory-robot-interface/). A pytest check keeps its rankings and gate decisions identical to the Python recognizer.
- **CI.** Eight jobs run on every pull request: ruff, mypy, pytest without ROS, pytest in ROS Noetic, a catkin build, a headless Gazebo smoke test, the DTW leave-one-out gate, and the recognizer comparison ([Validation](docs/VALIDATION.md)).

### Added

- Reproducible leave-one-out evaluation of the DTW bank, plus a CI job that fails below 95% Top-1 (#44).
- An optional Sakoe-Chiba window for DTW, with a benchmark of its speed and ranking changes (#46).
- A comparison of DTW with five alternatives under synthetic noise, rotation, truncation and stretching ([Recognizer comparison](docs/RECOGNIZER_COMPARISON.md), #47).
- A mypy CI job for the core ROS-free modules (#45).
- A Gazebo CI smoke test, scripted demo recordings and a Chinese README.
- A TELEOP-gated Gazebo gripper and pick objects.
- A headless dashboard test harness, which replaced tests that asserted on source text.
- A [Design decisions](docs/DESIGN_DECISIONS.md) page (#53) and the browser recognizer demo (#54).

### Changed

- Keyboard TELEOP is now a velocity command with acceleration and jerk limits. It used to go through the board's point-to-point filter, which cruised at about 0.03 m/s and overshot about 11 cm when the direction changed.
- The dashboard window was redesigned and gained a TELEOP view.
- Long dashboard, UI-state and lab-runtime functions and modules were split up, and duplicated DTW tooling was shared (#49 to #51).
- The README and reader docs were rewritten in plainer language (#41, #43).

### Fixed

- The dispatcher now refuses to start on an invalid `~label_to_task_json` or an unknown `~dispatcher_mapping_name`. It used to fall back silently to the default mapping.
- Boolean parameters of the keyboard teleop and serial publisher are parsed strictly (#52).
- The Gazebo image build retries apt when `snapshots.ros.org` returns intermittent errors.
- C++ tests skip cleanly when g++ or the Boost headers are missing (#42).
- The README now gives the correct counts for CI jobs, recognizer alternatives and pytest tests (#55).

### Known limitations

- All templates were drawn with a mouse by one person. The accuracy is an upper bound for that writer, not a measured accuracy for new users or for the magnetic board.
- The robustness perturbations are synthetic.
- The Real-FR3 adapters compile and are in the launch graph, but physical Real-FR3 `TELEOP` has not been run.

[1.0.0]: https://github.com/feizaodemon/magnetic-trajectory-robot-interface/releases/tag/v1.0.0
