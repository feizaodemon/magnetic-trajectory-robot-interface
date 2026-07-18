# Architecture

## Supervised control path

```mermaid
flowchart TD
    Board["Board serial packets"] --> BoardAdapter["Packet to 2D trajectory"]
    Mouse["Mouse stroke"] --> Dashboard["Shared dashboard"]
    BoardAdapter --> Dashboard
    Dashboard --> Sample["Completed sample"]
    Sample --> Recognizer["DTW template-bank recognizer"]
    Recognizer --> Candidates["/colmag/symbol_candidates"]
    Candidates --> Dashboard
    Dashboard --> Confirmed["/colmag/confirmed_label"]
    Confirmed --> Dispatcher["Task dispatcher"]
    Dispatcher --> Command["/colmag/task_command"]
    Command --> GazeboBridge["Gazebo bridge"]
    Command --> FR3Adapter["FR3 C++ adapter"]
    GazeboBridge --> SimAction["Simulation FollowJointTrajectory"]
    FR3Adapter --> RealAction["Controller FollowJointTrajectory"]
```

The system has four distinct decision boundaries:

1. trajectory completion;
2. recognition and candidate ranking;
3. explicit human confirmation;
4. dispatcher acceptance and backend routing.

No recognizer component owns `/colmag/task_command`.

## Component ownership

| Responsibility | Primary implementation |
| --- | --- |
| Board packet conversion | `colmag_ros/scripts/raw_packet_to_trajectory_2d_node.py` |
| Shared Board/Mouse dashboard | `colmag_ros/scripts/magnetic_trajectory_dashboard_node.py` |
| Interaction geometry and clutch | `colmag_ros/src/colmag_ros/dashboard_geometry.py` |
| Candidate presentation | `colmag_ros/src/colmag_ros/dashboard_candidate_display.py` |
| Confirmation payload | `colmag_ros/src/colmag_ros/dashboard_confirm_publisher.py` |
| DTW ranking | `colmag_ros/scripts/trajectory_symbol_top3_recognizer_node.py` |
| Dispatcher | `colmag_ros/scripts/task_dispatcher_node.py` |
| Gazebo robot-body bridge | `colmag_gazebo_stub/scripts/fr3_gazebo_visible_task_bridge_node.py` |
| FR3 adapter | `colmag_ros/src/colmag_fr3_task_trajectory_adapter.cpp` |
| Runtime packaging | `docker/` |

## Frontend profiles

The neutral frontend aliases are `dtw_mouse_demo_frontend.launch` and `dtw_real_board_demo_frontend.launch`. They share recognizer and dashboard code but differ in sample completion and interaction behavior. GUI-only use can disable confirmed-label publication and omit dispatcher/backend components.

## Recognition

The tracked bank uses schema `colmag_dtw_template_bank.v1`. Each template stores label, normalized trajectory data, and provenance fields such as source type, source file identifier, source backend, and y-flip state. Ranking uses DTW distance plus confidence and margin gates.

## Execution separation

Gazebo and FR3 are separate backends:

- Gazebo receives accepted tasks through a simulation-only bridge and rejects real-robot control claims.
- FR3 receives accepted tasks through a C++ adapter with freshness, deduplication, limits, lifecycle ownership, and final-state checks.

The same task name does not imply the same safety qualification across simulation and hardware.

## Compatibility

The `colmag` namespace, several launch filenames, configuration paths, and module names are retained because they form implemented API and packaging contracts. Reader-facing branding is neutral; no broad runtime namespace rename was attempted.
