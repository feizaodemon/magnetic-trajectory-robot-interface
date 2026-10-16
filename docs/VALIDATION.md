# Validation

## Automated command

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -p no:cacheprovider -q
```

The suite covers ROS-free parsing and helpers, dashboard state, DTW ranking, dispatcher behavior, Gazebo bridge behavior, FR3 safety modules, Docker configuration, package/install references, and selected route-isolation contracts. It does not start ROS, Gazebo, serial, GUI, or physical hardware.

Tests that import `rospy` or ROS message packages skip themselves when ROS Noetic is absent; `pytest -rs` prints each skip reason.

## Continuous integration

`.github/workflows/pytest.yml` runs on every pull request and every push to `main`:

| Job | Environment | Runs |
| --- | --- | --- |
| `pytest` | Python 3.10, no ROS | the suite; ROS-only tests skip |
| `pytest-noetic` | `ros:noetic-ros-base` container, Python 3.8 | the full suite, including the ROS-only tests; then `roslaunch --dump-params` and `--nodes` on the eight launch files whose dependencies ros-base has (the Gazebo and FR3 hardware launches need `franka_gazebo` or `franka_control`) |
| `ruff` | Python 3.10 | `ruff check` with the settings in `ruff.toml`: undefined names, unused imports and variables, and annotations that would not run on Python 3.8 |
| `catkin-build` | `ros:noetic-ros-base` container | `catkin_make install` of `colmag_ros` and `colmag_gazebo_stub` in Release mode, then checks that both FR3 adapter executables and the Python packages are installed |
| `gazebo-smoke` | image built from `docker/Dockerfile.noetic-gazebo` | `scripts/ci/gazebo_smoke.sh`: Xvfb, then `colmag_keyboard_gazebo_teleop.launch headless:=true spawn_pick_objects:=true`, then `scripts/ci/gazebo_smoke.py`. The script publishes the keyboard terminal's payloads and checks: TASK at startup and a rejected gripper command; in TELEOP, at least 3 cm of flange motion while +X is held and a stop after release; gripper open (width > 7 cm) and close; TASK again with a rejected gripper command; both pick objects spawned |

Both pytest jobs also compile and run every ROS-free C++ test under `tests/cpp/`. Only `gazebo-smoke` starts ROS nodes: the Gazebo keyboard `TELEOP` profile with headless Gazebo and the dashboard on a virtual display. No job runs an FR3 launch file (`--dump-params` and `--nodes` only parse it), or touches serial, real controllers, or hardware. Each job's log holds the exact result for its commit.

## ROS1 compile/link build

From a catkin workspace whose `src/` contains `colmag_ros` and
`colmag_gazebo_stub` from this checkout:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg colmag_ros colmag_gazebo_stub
```

This build compiles and links the shared `Fr3CartesianTeleopCore`, the Cartesian
adapter used by both backends, and the discrete FR3 task adapter. On every
pull request, the `catkin-build` CI job builds the same targets with
`catkin_make install`. A successful
build does not authorize or execute Gazebo, serial, GUI, controller, or physical
hardware runtime.

## Export-time checks

These checks ran by hand when the public portfolio edition was exported. CI repeats only those that are also pytest tests: Markdown links, launch file XML, and the `outputs/` check. The checks were:

- `git diff --check` and staged diff checks;
- local Markdown links and images using case-sensitive repository paths;
- XML parsing for package and launch files;
- launch include targets and Docker `COPY` sources;
- absence of tracked references into `outputs/`;
- secret, credential, private-network, identity, and absolute-path patterns;
- filename review plus manual review of every retained non-text asset;
- repository visibility, default branch, remote commit, and clean worktree after push.

## Evidence boundaries

| Validation layer | Establishes | Does not establish |
| --- | --- | --- |
| Static/unit tests | source contracts, parsing, mapping, guards | live ROS timing or GUI rendering |
| Docker configuration checks | build context, profiles, labels, gates | a currently built/running image |
| Local browser review | documentation rendering and readability | backend runtime or robot safety |
| Gazebo smoke test | one scripted TELEOP run per commit: gating, motion, gripper | GUI rendering, timing under load, grasp quality |
| Gazebo evidence | only the exact recorded simulation run | real hardware behavior |
| FR3 evidence | only an explicitly authorized, bounded run | certification or arbitrary motion |

The README demo GIFs are real-time screen captures of headless Gazebo in Docker, with mouse and key input from `xdotool` scripts. The `TASK` clip was recorded at `cef908c` with `visible_bridge_motion_scale:=3.0 visible_bridge_motion_duration_sec:=4.0`. The `TELEOP` clip was recorded at `35df0ec` with `spawn_pick_objects:=true`, using [scripts/demo](../scripts/demo/README.md); the script releases each key based on the measured flange position.

## Tests removed at export

The source snapshot had twelve test files that only checked the wording of internal process documents. Those documents are not part of the public edition, so the tests were removed at export and never existed in this repository. Each one was removed only where focused implementation tests still cover the technical behavior. A repository-wide link and exact-case check replaced the old Markdown closure test.
