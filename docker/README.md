# Docker workflows

The Docker directory separates Gazebo and FR3 build/runtime profiles.

- `compose.yaml` and `Dockerfile.noetic-gazebo` package GUI plus Gazebo simulation.
- `compose.fr3.yaml` and `Dockerfile.noetic-fr3` package the FR3-compatible controller environment.
- `run_gazebo_demo.sh` is the simulation wrapper.
- `run_fr3_demo.sh` owns FR3 provenance and environment gates.

Build the Gazebo image from the current revision:

```bash
COLMAG_GIT_SHA="$(git rev-parse HEAD)" \
docker compose -f docker/compose.yaml \
  --profile mouse-gazebo build mouse-gazebo
```

The default image is `magnetic-trajectory-interface:gazebo-noetic`. Inspect `org.opencontainers.image.revision` before runtime. See [Running](../docs/RUNNING.md) and [Hardware Boundaries](../docs/HARDWARE_BOUNDARIES.md).
