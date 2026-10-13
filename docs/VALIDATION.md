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
| `pytest-noetic` | `ros:noetic-ros-base` container, Python 3.8 | the full suite, including the ROS-only tests |
| `ruff` | Python 3.10 | `ruff check` with the settings in `ruff.toml`: undefined names, unused imports and variables, and annotations that would not run on Python 3.8 |
| `catkin-build` | `ros:noetic-ros-base` container | `catkin_make install` of `colmag_ros` and `colmag_gazebo_stub` in Release mode, then checks that both FR3 adapter executables and the Python packages are installed |

Both pytest jobs also compile and run the ROS-free C++ safety tests under `tests/cpp/`. The Cartesian core test there is not wired in yet; see [tests/README.md](../tests/README.md). No job starts ROS nodes or launch files, or touches Gazebo, GUI, serial, controllers, or hardware. Each job's log holds the exact result for its commit.

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
| Gazebo evidence | only the exact recorded simulation run | real hardware behavior |
| FR3 evidence | only an explicitly authorized, bounded run | certification or arbitrary motion |

## Tests removed at export

The source snapshot had twelve test files that only checked the wording of internal process documents. Those documents are not part of the public edition, so the tests were removed at export and never existed in this repository. Each one was removed only where focused implementation tests still cover the technical behavior. A repository-wide link and exact-case check replaced the old Markdown closure test.
