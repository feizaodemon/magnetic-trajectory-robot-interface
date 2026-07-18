# Test suite

Run the offline/static suite from the repository root:

```bash
python3 -m pytest -p no:cacheprovider -q
```

The tests cover serial parsing, dashboard state, DTW ranking, confirmation, dispatch, Gazebo bridge behavior, FR3 safety helpers, Docker configuration, package references, and route isolation. They do not launch ROS, Gazebo, GUI, serial, or real hardware.

See [Validation](../docs/VALIDATION.md) for evidence boundaries and portfolio-specific checks.
