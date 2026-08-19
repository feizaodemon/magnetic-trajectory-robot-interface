# Glossary

| Term | Meaning |
| --- | --- |
| candidate | A ranked recognition result; it is not confirmation or execution authority. |
| confirmation | Explicit operator action that publishes the reviewed label. |
| dispatcher | The sole component that validates confirmed labels and may publish `/colmag/task_command`. |
| DTW | Dynamic Time Warping, used to compare normalized trajectories with stored templates. |
| interaction clutch | Board-profile convention that separates navigation from drawing/hover behavior. |
| Mouse profile | Dashboard drawing profile with recognition after stroke release. |
| Board profile | Magnetic-input profile with clutch-controlled drawing and explicit recognition. |
| Gazebo robot-body route | Simulation path that moves the Panda model through a controller-like trajectory interface. |
| marker route | Visual-only marker movement; not robot-body execution. |
| FR3 adapter | C++ component that validates accepted tasks and constructs bounded joint trajectories. |
| `FollowJointTrajectory` | ROS action interface used by the simulation and controller paths. |
| provenance label | OCI image metadata that records the source Git revision used at build time. |
| public portfolio edition | Public curated repository derived from a collaborative baseline while retaining attribution, provenance, validation boundaries, and third-party license notices. |
