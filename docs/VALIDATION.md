# Validation

## Automated command

```bash
python3 -m pytest -p no:cacheprovider -q
```

The suite covers ROS-free parsing and helpers, dashboard state, DTW ranking, dispatcher behavior, Gazebo bridge behavior, FR3 safety modules, Docker configuration, package/install references, and selected route-isolation contracts. It does not start ROS, Gazebo, serial, GUI, or physical hardware.

## ROS1 compile/link build

From a catkin workspace whose `src/` contains `colmag_ros` and
`colmag_gazebo_stub` from this checkout:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg colmag_ros colmag_gazebo_stub
```

This build compiles and links the shared `Fr3CartesianTeleopCore`, the Cartesian
adapter used by both backends, and the discrete FR3 task adapter. A successful
build does not authorize or execute Gazebo, serial, GUI, controller, or physical
hardware runtime.

## Public portfolio publication gates

The public portfolio edition also checks:

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
| Gazebo evidence | only the exact recorded simulation run | real hardware behavior |
| FR3 evidence | only an explicitly authorized, bounded run | certification or arbitrary motion |

## Documentation-test consolidation

The source snapshot included release-gate tests whose only purpose was to assert internal archive, task-ledger, agent-context, or process-document wording. Those process documents are intentionally excluded from the public portfolio edition. Portfolio-irrelevant document-coupled tests were removed only where the underlying technical behavior remains covered by focused implementation tests; Markdown closure was replaced with a repository-wide portfolio link and exact-case check.

Removed document-coupled release tests:

- `tests/test_m104b2c_mouse_seeded_dtw_feasibility_static.py`
- `tests/test_m104b2d_dtw_template_bank_recognizer_static.py`
- `tests/test_m104b2e_dtw_template_bank_smoke_launch_static.py`
- `tests/test_m104c1_8symbol_dtw_recognition_only_static.py`
- `tests/test_m104c2c3_8symbol_recognition_display_static.py`
- `tests/test_m104c4_8symbol_gazebo_execution_static.py`
- `tests/test_m104_lab_fr3_controller_validation_guide_static.py`
- `tests/test_m104r1c_fr3_task_trajectory_adapter_static.py`
- `tests/test_m104r2_f3a_hardware_route_static.py`
- `tests/test_m104r2_f3b_docker_naming_static.py`
- `tests/test_m104r2_f3c_fr3_lab_operator_static.py`
- `tests/test_m104r2_param1_fr3_adapter_params_static.py`

Retained coverage includes DTW ranking, tracked-bank shape/provenance, dashboard behavior, confirmation, dispatcher semantics, Gazebo bridge behavior, FR3 safety modules and ROS-free C++ tests, FR3 operator logic, Docker configuration, packaging, and route isolation.

## Current result

The exact pytest count, link result, publication scan, browser result, commit SHA, and remote verification are recorded in the final creation report for this repository revision rather than hard-coded here.
