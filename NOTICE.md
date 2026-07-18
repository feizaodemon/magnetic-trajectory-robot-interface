# Notice

## Private review status

This repository is a **private portfolio review copy** exported from a fixed tracked snapshot. **PUBLICATION RIGHTS NOT YET CONFIRMED.** It must not be made public until the relevant contributors and rights holders confirm redistribution terms.

The source snapshot contains no root `LICENSE`, `LICENSE.md`, `LICENSE.txt`, `COPYING`, or repository-level notice. The ROS package manifests declare `MIT` in metadata, but the corresponding license text is absent and repository-wide ownership is not established. No new license is asserted by this portfolio edition.

## Provenance

- Source repository: [original collaborative repository](https://github.com/emaema99/COLMAG-seminar-SS26)
- Source branch: `xiaowei`
- Source commit: `e3c81fb31f5113c50f197bb11eff183295cf3163`
- Export form: filtered tracked snapshot with a new Git history

The source repository, its branches, tags, issues, pull requests, releases, settings, and Git history were not modified or mirrored.

## Compatibility identifiers

Internal ROS packages, topics, environment variables, and selected compatibility-sensitive filenames retain the `colmag` namespace or legacy identifiers. This avoids risky changes to imports, catkin install rules, launch includes, topic contracts, Docker gates, and tests. The reader-facing project name is Magnetic Trajectory Robot Interface.

## Third-party components

This repository does not vendor the source trees for ROS, Gazebo, `libfranka`, or `franka_ros`. The FR3 Dockerfile fetches pinned upstream revisions during image construction:

| Component | Source model | Required action |
| --- | --- | --- |
| ROS1 Noetic packages | Ubuntu/ROS package repositories | Follow each package's upstream license. |
| Gazebo and Franka simulation packages | ROS package repositories | Follow upstream notices and licenses. |
| `libfranka` | Pinned upstream Git revision; Apache-2.0 | Runtime image copies upstream `LICENSE` and `NOTICE`. |
| `franka_ros` | Pinned upstream Git revision; Apache-2.0 | Runtime image copies upstream `LICENSE` and `NOTICE`. |
| Python dependencies | `requirements.txt` | Follow each package's upstream license. |

The FR3 Dockerfile copies the pinned upstream Apache-2.0 `LICENSE` and `NOTICE` files into the runtime image. Publishing an image still requires a separate transitive-dependency license and notice review.

## Model and data boundary

The tracked DTW bank was generated from user-handwritten mouse seeds and records source provenance per template. Raw seed directories, ignored outputs, logs, screenshots, external datasets, and large binary evidence are not included. The tracked bank remains covered by the unresolved repository-level publication boundary and may not be publicly redistributed without approval.

## Hardware evidence boundary

No private robot address, network credential, lab screenshot, machine path, raw hardware log, or identifiable background is included. Documentation describes the code boundary and previously recorded evidence conservatively; it does not certify current hardware behavior, arbitrary trajectories, sequential sessions, or physical safety.
