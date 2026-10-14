# Running

## Safety first

Repository tests are offline/static. Do not start serial, GUI, Gazebo, or FR3 hardware merely to run the test suite. Runtime commands below require explicit authorization for the relevant surface.

## Prerequisites

- Linux or WSL-compatible Docker host for the container paths;
- Docker Engine with Compose;
- X11 access for GUI profiles;
- ROS1 Noetic only when using a native diagnostic path;
- a clean Git checkout for image revision checks.

Record the candidate revision:

```bash
git status --short
git rev-parse HEAD
```

## Offline validation

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -p no:cacheprovider -q
git diff --check
```

Without ROS Noetic, the tests that import `rospy` or ROS messages skip
themselves. See [Validation](VALIDATION.md).

## Mouse GUI only

After sourcing a ROS1 Noetic catkin workspace containing this package:

```bash
roslaunch colmag_ros dtw_mouse_demo_frontend.launch \
  publish_confirmed_label:=false
```

This starts no dispatcher or execution backend.

## Board GUI only

Use a synthetic or explicitly authorized serial source. The frontend can remain non-executing:

```bash
roslaunch colmag_ros dtw_real_board_demo_frontend.launch \
  publish_confirmed_label:=false
```

Do not open a real serial device without authorization and an explicit port review.

## Gazebo Docker profile

Build from the current revision:

```bash
COLMAG_GIT_SHA="$(git rev-parse HEAD)" \
docker compose -f docker/compose.yaml \
  --profile mouse-gazebo build mouse-gazebo
```

Inspect the resulting revision label:

```bash
docker inspect magnetic-trajectory-interface:gazebo-noetic \
  --format '{{ index .Config.Labels "org.opencontainers.image.revision" }}'
```

Then, only when Gazebo and GUI runtime are authorized:

```bash
docker/run_gazebo_demo.sh mouse-gazebo
```

The Board profile is `real-board-gazebo` and additionally requires an authorized serial device.

The bridge's placeholder motions are small by default. For a more visible motion, pass a launch command that sets `visible_bridge_motion_scale` (default `1.0`) and `visible_bridge_motion_duration_sec` (default `2.0`). The bridge still clamps every joint target to its limits:

```bash
docker/run_gazebo_demo.sh mouse-gazebo \
  roslaunch colmag_ros colmag_8symbol_gazebo_robot_body_demo.launch \
  visible_bridge_motion_scale:=3.0 visible_bridge_motion_duration_sec:=4.0
```

## Native Gazebo TASK/TELEOP profile

After a ROS1 Noetic build, start the combined Gazebo profile only when Gazebo
and GUI runtime are authorized:

```bash
roslaunch colmag_ros colmag_keyboard_gazebo_teleop.launch
```

`TASK` is the startup default. To use continuous Cartesian `TELEOP`, select it
in the dashboard and run the terminal input owner separately:

```bash
rosrun colmag_ros keyboard_cartesian_teleop_node.py
```

The GUI selection does not authorize hardware. The physical-board/Real-FR3
profile remains outside the quick path and requires every gate below.

## FR3 profiles

FR3 is intentionally not a quick-start path. Read [Hardware Boundaries](HARDWARE_BOUNDARIES.md), create the ignored `docker/fr3-hardware.env` from the example, and complete all provenance, network, controller, E-stop, workspace, and operator gates before any connected run.

The safe first command is status-only:

```bash
docker/run_fr3_demo.sh status
```

Do not run `dry-run`, `mouse`, or `board` against connected hardware without explicit authorization and local safety supervision.

## DTW template bank

The recognizer launches load the tracked bank
`colmag_ros/config/dtw_banks/m104c1_8symbol_mouse_seeded_yflip.json`. It holds
124 templates for the labels `1,2,3,V,O,X,A,C`, built from user-handwritten
mouse seeds with the `mouse_centroid_maxabs_resample64` profile and y-flip.
Each template records its source seed file.

The seed JSONs are not published, so this repository alone cannot rebuild the
bank. With seeds under `outputs/classifier_training/seeds/<label>/*.json`, the
offline tools are:

```bash
# Build a bank into ignored outputs/.
python3 scripts/build_dtw_template_bank_from_seeds.py \
  --labels 1,2,3,V,O,X,A,C \
  --out outputs/classifier_artifacts/dtw_bank/m104c1_8symbol_mouse_seeded_yflip.json

# Hold-out check on the seeds: the earlier seeds of each label become
# templates, the later ones become queries.
python3 scripts/eval_dtw_bank_seed_holdout.py

# Score a bank against recorded dashboard samples under
# outputs/classifier_training/runtime_samples (labels 1,2,3 by default).
python3 scripts/eval_dtw_bank_on_runtime_samples.py \
  --template-bank colmag_ros/config/dtw_banks/m104c1_8symbol_mouse_seeded_yflip.json
```

All three write only under `outputs/` and start no ROS, GUI, serial, or robot
process. When the generated bank exists at the path above,
`tests/test_curated_dtw_bank_static.py` checks that the tracked bank has the
same labels and template points.

## Generated outputs

Logs, screenshots, temporary audit files, generated banks, and local environment files belong under ignored `outputs/` or the ignored hardware env file. They are not tracked portfolio content.
