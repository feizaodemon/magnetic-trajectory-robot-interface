# Demo recording

These scripts record the `TELEOP` pick-and-place GIF in the main README
(`docs/media/teleop_pick_and_place.gif`). Everything runs in headless Docker
Gazebo; nothing connects to hardware.

- `Dockerfile.recorder`: the Gazebo image plus Xvfb, `xdotool`, `xterm`, and `ffmpeg`.
- `record_pick_and_place.sh`: starts `colmag_keyboard_gazebo_teleop.launch spawn_pick_objects:=true`
  on a virtual 1920x1080 display, lays out the dashboard, Gazebo, and the keyboard
  terminal, records the screen, and renders the GIF with the held key drawn on it.
- `pick_and_place_keys.py`: types into the keyboard terminal with `xdotool`. It holds
  each key like a person would and releases it from the flange position read from tf.
  A short corrective hold follows when the robot stops more than 1.2 cm (height) or 2 cm
  (horizontal) from the target.

From the repository root:

```bash
docker build -f docker/Dockerfile.noetic-gazebo \
  --build-arg COLMAG_GIT_SHA="$(git rev-parse HEAD)" \
  -t magnetic-trajectory-interface:gazebo-noetic .
docker build -f scripts/demo/Dockerfile.recorder -t colmag-demo-recorder scripts/demo
docker run --rm -v "$PWD/scripts/demo:/demo:ro" -v "$PWD/outputs/demo:/out" \
  colmag-demo-recorder bash /demo/record_pick_and_place.sh
```

The run takes about 3 minutes. Outputs go to the ignored `outputs/demo/`:
`pick_and_place.mp4`, `keys.txt` (key spans), `teleop_pick_and_place.gif`, and
the launch log. The script prints the final cube position; on the pad it is
close to (0.45, 0.15, 0.03).

The input is scripted, so the clip shows what the keyboard route can do, not a
person's driving. Simulation speed depends on the host CPU, so stop positions
vary by about a centimetre from run to run.
