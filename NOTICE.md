# Notice

## Public portfolio status

This repository is published as a **public portfolio edition**. Its baseline was exported from a fixed tracked snapshot and the current edition includes a subsequent curated dual-mode working-tree export. Public availability does not change the authorship of the collaborative baseline, remove attribution, or alter third-party license obligations.

The source snapshot contains no root `LICENSE`, `LICENSE.md`, `LICENSE.txt`, `COPYING`, or repository-level notice. The ROS package manifests declare `MIT` in metadata, but the corresponding repository-wide license text is absent and repository-wide ownership is not asserted by this portfolio edition. No new repository-wide license is granted by this notice.

## Provenance

- Source repository: [original collaborative repository](https://github.com/emaema99/COLMAG-seminar-SS26)
- Baseline source branch: `xiaowei`
- Baseline source commit: `e3c81fb31f5113c50f197bb11eff183295cf3163`
- Subsequent source: validated dual-mode working tree based on the baseline, exported 2026-08-19
- Export form: curated files with a clean portfolio Git history

The source repository, its branches, tags, issues, pull requests, releases, settings, and Git history were not modified or mirrored. No claim of sole authorship is introduced by the portfolio curation.

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

The tracked DTW bank was generated from user-handwritten mouse seeds and records source provenance per template. Raw seed directories, ignored outputs, logs, screenshots, external datasets, and large binary evidence are not included. The tracked bank is included in this public portfolio edition with its provenance metadata retained. This notice does not grant a separate license for the bank beyond applicable rights and license terms.

## Hardware evidence boundary

No private robot address, network credential, lab screenshot, machine path, raw hardware log, or identifiable background is included. Continuous Cartesian Gazebo TELEOP has recorded runtime evidence, while the Real-FR3 adapters are compile/integration validated only; physical Real-FR3 TELEOP was not executed. Nothing here certifies current hardware behavior, arbitrary trajectories, sequential sessions, or physical safety.
