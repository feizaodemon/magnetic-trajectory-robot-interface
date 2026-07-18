# FR3 Docker boundary

The FR3 image is separate from Gazebo and defaults to `magnetic-trajectory-interface:fr3-noetic`. It fetches pinned `libfranka` and `franka_ros` revisions during build, copies installed artifacts into the runtime stage, and labels the image with the source Git revision.

Copy `fr3-hardware.env.example` to the ignored `fr3-hardware.env` only on the authorized host. Never commit the robot address or site-specific configuration.

Start with the non-motion status check:

```bash
docker/run_fr3_demo.sh status
```

Connected dry-run or active profiles require explicit authorization and every gate in [Hardware Boundaries](../docs/HARDWARE_BOUNDARIES.md). This document does not authorize hardware motion.
