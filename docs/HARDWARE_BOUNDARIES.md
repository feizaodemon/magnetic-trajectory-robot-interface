# Hardware Boundaries

## Default state

Real-robot motion is disabled by default. Offline tests must not start ROS, Gazebo, GUI, serial, or hardware processes.

The dashboard's `TASK`/`TELEOP` selection is command-mode state only. It never
sets `allow_real_robot`, enables command publication, validates calibration,
starts controller bringup, or grants physical-runtime authorization.

## Simulation

The Gazebo route is simulation-only. A valid simulation claim requires a Panda robot body, an active controller-like path, `FollowJointTrajectory` evidence, and changing joint state. Marker-only movement is not robot-body motion.

## FR3 adapter

The FR3 route uses a C++ adapter between `/colmag/task_command` and the standard position joint trajectory controller. Its source-level guards include:

- accepted-task allowlists;
- command freshness and deduplication;
- exact joint-name and limit checks;
- bounded trajectory generation;
- lifecycle and action-goal ownership;
- terminal-result and final-joint-state verification;
- cancel behavior limited to adapter-owned goals.

These guards are necessary but do not independently prove a safe physical run.

The shared Cartesian core and both Cartesian adapters compile in the ROS1
build. Continuous keyboard Cartesian Gazebo TELEOP was runtime validated.
Physical magnetic-board Cartesian TELEOP on a Real FR3 was not executed.

## Required connected-run gates

Before any connected FR3 command:

1. use a clean checkout and record `HEAD`;
2. build and inspect an image with a matching OCI revision;
3. review the ignored environment file without committing the robot address;
4. verify the isolated robot network and approved host;
5. confirm one controller/action owner and no conflicting process;
6. complete controller readiness without sending motion;
7. keep the physical E-stop within reach and use local supervision;
8. authorize the exact input profile and bounded task;
9. stop on any revision mismatch, fault, unexpected process, or preflight failure.

## Claims

This portfolio preserves previously validated implementation and conservative evidence boundaries. It does not claim current hardware availability, new runtime validation, certification, contact control, force control, torque control, arbitrary trajectories, or unattended operation.

## Private data

Robot addresses and site-specific values belong only in the ignored `docker/fr3-hardware.env`. Do not commit addresses, credentials, network details, screenshots, logs, machine paths, or identifiable hardware-site information.
