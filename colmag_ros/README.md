# `colmag_ros` compatibility package

`colmag_ros` is the retained ROS1 package name for the Magnetic Trajectory Robot Interface. The name remains unchanged to preserve imports, launch includes, install rules, and ROS topic contracts.

Reader documentation:

- [Project overview](../README.md)
- [Architecture](../docs/ARCHITECTURE.md)
- [Dual-mode teleoperation](../docs/DUAL_MODE_TELEOPERATION.md)
- [Running](../docs/RUNNING.md)
- [Interaction profiles](../docs/INTERACTION_PROFILES.md)
- [Hardware boundaries](../docs/HARDWARE_BOUNDARIES.md)

The neutral frontend aliases are `dtw_mouse_demo_frontend.launch` and `dtw_real_board_demo_frontend.launch`. Do not launch hardware routes without explicit authorization and the gates in the hardware-boundary document.

## TASK and TELEOP control modes

`control_mode_node.py` publishes the latched source of truth for exactly `TASK`
and `TELEOP`. `TASK` retains recognition, explicit confirmation, dispatch, and
the selected backend. `TELEOP` transfers command ownership to the continuous
Cartesian adapter. The dashboard requests a mode only; it never changes a hardware gate.

| Environment | `TASK` | `TELEOP` |
| --- | --- | --- |
| Gazebo | DTW dashboard pipeline and robot-body task bridge | Keyboard Cartesian input, shared continuous core, controller command |
| Real FR3 | Confirmed-task discrete adapter | Physical magnetic board, the same continuous core, controller command |

Start the combined Gazebo profile, switch the dashboard to `TELEOP`, then run
the terminal input owner in another terminal:

```bash
roslaunch colmag_ros colmag_keyboard_gazebo_teleop.launch
rosrun colmag_ros keyboard_cartesian_teleop_node.py
```

The continuous route is:

```text
keyboard_cartesian_teleop_node.py
  -> /colmag/teleop/cartesian_input
  -> colmag_gazebo_cartesian_teleop_adapter
  -> Fr3CartesianTeleopCore
  -> /position_joint_trajectory_controller/command
  -> Gazebo Panda robot body
```

`colmag_real_board_fr3_cartesian_teleop.launch` combines the physical-board
frontend with the continuous Cartesian adapter and the TASK-gated discrete
adapter. The C++ core owns bounded mapping, target shaping, Franka forward
kinematics, numerical Jacobian, DLS inverse kinematics, workspace/joint limits,
and finite-output validation.

Physical command publication remains disabled unless the operator explicitly
sets the required gates, including `allow_real_robot=true`,
`send_commands=true`, an active hardware profile, and
`calibration_valid=true`. The checked-in mapping values are reference values,
not physical calibration evidence. Physical Real-FR3 TELEOP runtime has not
been executed.

The retained `keyboard_teleop_node.py` and
`colmag_keyboard_fr3_teleop.launch` form a predefined-task diagnostic under
`TASK`; they are not continuous Cartesian TELEOP:

```bash
rosrun colmag_ros keyboard_teleop_node.py
```

That diagnostic publishes `/colmag/teleop/confirmed_label`; a TASK-gated
dispatcher remains the owner of `/colmag/task_command`.
