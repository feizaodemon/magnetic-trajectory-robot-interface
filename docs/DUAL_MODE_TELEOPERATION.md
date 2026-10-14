# Dual-Mode Teleoperation

## Product model

Control mode and backend are independent axes:

| Backend | `TASK` | `TELEOP` |
| --- | --- | --- |
| Gazebo | Recognition-driven task execution | Keyboard Cartesian control |
| Real FR3 | Discrete task execution | Magnetic-board Cartesian teleoperation |

`control_mode_node.py` is the latched source of truth on
`/colmag/control_mode`. The dashboard requests `TASK` or `TELEOP` on
`/colmag/control_mode/request`. Invalid or unavailable mode state fails closed.
Selecting a mode does not select a backend or authorize hardware.

## TASK route

```text
magnetic or canvas trajectory
  -> DTW recognition
  -> ranked candidates
  -> GUI confirmation
  -> TASK-gated dispatcher
  -> execution backend
```

Gazebo uses the robot-body task bridge. Real FR3 uses the discrete C++
task-to-trajectory adapter and `FollowJointTrajectory`. Recognition never owns
the execution topic directly.

## Gazebo TELEOP route

```text
keyboard Cartesian input
  -> /colmag/teleop/cartesian_input
  -> Gazebo Cartesian adapter
  -> Fr3CartesianTeleopCore
  -> simulated trajectory controller
  -> Gazebo Panda
```

The input maps held keys to normalized Cartesian direction. The shared core
owns destination filtering, bounded target progression, Franka forward
kinematics, a numerical Jacobian, damped-least-squares inverse kinematics,
workspace and joint bounds, residual rejection, and finite-output checks.

For keyboard input, the core treats the direction as a velocity command. It
ramps toward `max_linear_speed_m_s` inside the acceleration and jerk limits and
brakes before a workspace bound. The keyboard node keeps a key held through the
terminal's first auto-repeat delay (`~first_repeat_window_sec`, default 0.6 s)
and publishes a zero direction when the repeats stop. When input goes stale,
the adapter brakes to rest (`STOPPING`, at most 2 s) instead of dropping the
command stream. The board route keeps its position filter and S-curve.

In `TELEOP`, the mouse dashboard pauses its drawing and dwell buttons and shows
a read-only view instead: a keycap pad lit by the current
`/colmag/teleop/cartesian_input` direction, and a Teleoperation card with the
adapter state from `/colmag/gazebo_cartesian_teleop_state` (for example
`Moving +X`, `Paused`, `Waiting for input`). The dashboard publishes nothing on
this route. The board dashboard keeps its trajectory view under a `TELEOP
active` notice.

### Gazebo gripper

`O` and `C` in the keyboard terminal publish one discrete open or close command
on `/colmag/teleop/gripper_command`. `gazebo_gripper_bridge_node.py`
(`colmag_gazebo_stub`, started by the keyboard Gazebo launch unless
`start_gripper_bridge:=false`) accepts a command only while
`/colmag/control_mode` is `TELEOP`, the sequence id increases, and the command
is at most 1 s old. It then calls the simulated `franka_gripper` `move` (open)
or `grasp` (close) action and publishes `OPENING`, `CLOSING`, `OPEN`, `CLOSED`,
`HOLDING`, `REJECTED` or `FAILED` with the finger width on
`/colmag/teleop/gripper_state`. The dashboard shows that state on the
Teleoperation card and lights the `O`/`C` keycaps while the gripper moves.
There is no real-hardware gripper route.

`spawn_pick_objects:=true` adds a 5 cm cube and a static place pad for a
pick-and-place exercise (positions set by `pick_cube_x`/`pick_cube_y` and
`place_pad_x`/`place_pad_y`).

This continuous Cartesian Gazebo route was exercised at runtime, including GUI
`TASK -> TELEOP -> TASK` transitions, moving Panda joints, and a keyboard
pick-and-place of the spawned cube.

## Real-FR3 TELEOP route

```text
physical magnetic board
  -> serial acquisition
  -> continuous position
  -> Real-FR3 Cartesian adapter
  -> Fr3CartesianTeleopCore
  -> trajectory controller
  -> Real FR3
```

The physical frontend retains continuous board mapping, the interaction clutch,
stale-input handling, and configurable calibration/workspace bounds. Controller
publication additionally requires explicit hardware enable, an active hardware
profile, valid calibration, fresh post-transition input, and fresh matching
joint state.

The Real-FR3 Cartesian and discrete adapters compile and are integrated into
the launch graph. Physical Real-FR3 TELEOP runtime was intentionally not
executed, so this is not a physical-runtime success claim.

## Ownership transitions

On `TASK -> TELEOP`, task dispatch stops, each discrete adapter relinquishes
only its own action goal, and continuous control waits for fresh input and joint
state. On `TELEOP -> TASK`, the continuous target is reset and a future explicit
confirmation is required before another task command. These are command-owner
transitions, not an emergency stop or a replacement for controller safety.

## Validation boundary

CI checks every change with focused tests, launch XML checks, and a ROS1 Noetic
`catkin_make install` build. The build compiles and links the shared core and
both adapters. None of this repeats or extends physical hardware evidence. See
[Validation](VALIDATION.md).
