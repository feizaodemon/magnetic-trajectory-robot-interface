# Credits

Magnetic Trajectory Robot Interface is a personal portfolio edition derived from a collaborative robotics project. Portfolio curation and a clean Git history do not change the authorship of the original work.

## Portfolio contribution evidence

The baseline source branch records the portfolio owner's public GitHub account, `@feizaodemon`, under the commit aliases `xiaowei`, `Xiaowei`, `Xiaowei Rong`, and `feizao demon`. Representative file histories support contributions to:

- magnetic-board acquisition and interaction-clutch behavior;
- Mouse and Board dashboard workflows;
- DTW recognition and template-bank integration;
- candidate confirmation and task dispatch;
- Gazebo robot-body execution integration;
- the FR3 task-to-trajectory adapter and safety gates;
- Docker packaging, testing, validation, and documentation.

Subsequent portfolio development added the `TASK`/`TELEOP` control-mode seam,
keyboard Cartesian Gazebo control, magnetic-board Real-FR3 integration, and the
shared Cartesian teleoperation core. Those changes were exported from the
validated working tree based on the baseline below; they were not represented
by a newer collaborative-repository commit.

The safe portfolio wording is “designed,” “implemented,” “integrated,” “refactored,” “validated,” “documented,” and “maintained,” depending on the area. This repository does not claim that the entire collaborative project was independently invented or solely authored by the portfolio owner.

## Other public source-history contributors

- Federico Masiero — public source-history commits that established the initial collaboration README and added presentation PDFs. Those PDFs and the original process README are excluded from this portfolio edition, but the original authorship remains acknowledged here.

The original repository may contain collaboration outside the fixed branch history reviewed for this edition. Absence from this short list must not be interpreted as a removal of original authorship or rights.

## Provenance

- Baseline source branch: `xiaowei`
- Baseline source commit: `e3c81fb31f5113c50f197bb11eff183295cf3163`
- Baseline commit subject: `COLMAG docs: complete external-reader reference closure`
- Subsequent export basis: validated dual-mode working tree based on that commit, exported 2026-08-19
- [Original collaborative repository](https://github.com/emaema99/COLMAG-seminar-SS26)

No private email addresses are reproduced. The original Git history was not copied into this portfolio repository. This provenance distinction does not assert sole authorship of the collaborative baseline or erase other contributors.

## Third-party projects

The runtime depends on ROS1 Noetic, Gazebo, Franka ROS packages, `libfranka`, Python packages, and their transitive dependencies. These projects remain owned and licensed by their respective authors. See [NOTICE.md](NOTICE.md) for the dependency and license boundary.
