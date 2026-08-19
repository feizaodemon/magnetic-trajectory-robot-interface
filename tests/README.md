# Test suite

Run the offline/static suite from the repository root:

```bash
python3 -m pytest -p no:cacheprovider -q
```

The tests cover serial parsing, dashboard state, DTW ranking, confirmation,
dispatch, control-mode admission, keyboard Cartesian input, the shared
Cartesian core, Gazebo bridge behavior, FR3 safety helpers, Docker
configuration, package references, and route isolation. They do not launch ROS,
Gazebo, GUI, serial, or real hardware.

The ROS1 build is a separate validation layer. It compiles and links the shared
core plus the Gazebo/Real-FR3 Cartesian and discrete adapters.

See [Validation](../docs/VALIDATION.md) for evidence boundaries and portfolio-specific checks.
