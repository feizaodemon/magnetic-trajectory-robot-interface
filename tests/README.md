# Test suite

Run the offline/static suite from the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -p no:cacheprovider -q
```

Without ROS Noetic, the tests that import ROS packages skip themselves.

The tests cover serial parsing, dashboard state, DTW ranking, confirmation,
dispatch, control-mode admission, keyboard Cartesian input, the shared
Cartesian core, Gazebo bridge behavior, FR3 safety helpers, Docker
configuration, package references, and route isolation. They do not launch ROS,
Gazebo, GUI, serial, or real hardware.

Layout:

- `test_*.py`: the pytest suite. Tests that need real ROS message packages skip
  themselves without ROS Noetic.
- `cpp/`: ROS-free C++ tests for the FR3 safety helpers, trajectory
  generators, and the Cartesian core. Tests build them with `g++` through
  `cpp_harness.py` into a pytest temporary directory and run them. A test
  skips when `g++` is missing, or when it needs Boost and `libboost-dev` is not
  installed. CI sets `COLMAG_REQUIRE_CPP_DEPS=1`, which turns those skips into
  failures.
- `dashboard_harness.py`: builds a real dashboard node with fake `rospy`,
  `std_msgs`, and `tkinter` modules, so dashboard tests need no display or ROS
  master.
- `launch_xml.py`: reads `<arg>` and `<param>` values from launch XML for the
  static launch tests.

The ROS1 build is a separate validation layer. It compiles and links the shared
core plus the Gazebo/Real-FR3 Cartesian and discrete adapters. CI runs it in the
`catkin-build` job.

See [Validation](../docs/VALIDATION.md) for the CI jobs and evidence boundaries.
