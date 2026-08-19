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

This continuous Cartesian Gazebo route was exercised at runtime, including GUI
`TASK -> TELEOP -> TASK` transitions and moving Panda joints.

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

The portfolio checkout is validated independently with focused tests, Python
parsing, launch/XML checks, and a ROS1 Noetic compile/link build. That proves the
curated export includes the shared core and both adapters. It does not repeat or
extend physical hardware evidence.
