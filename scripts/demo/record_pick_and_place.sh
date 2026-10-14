#!/usr/bin/env bash
# Record the README pick-and-place GIF inside the demo recorder image.
# Starts Xvfb and the keyboard Gazebo TELEOP profile with the pick objects,
# lays out dashboard, Gazebo and the keyboard terminal on one 1920x1080
# screen, switches to TELEOP, runs pick_and_place_keys.py, switches back to
# TASK, and writes /out/pick_and_place.mp4, /out/keys.txt and
# /out/teleop_pick_and_place.gif. Gazebo only.
set -euo pipefail
DEMO=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-/out}
mkdir -p "$OUT"
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1

Xvfb :99 -screen 0 1920x1080x24 >"$OUT/xvfb.log" 2>&1 &
sleep 2
# Terminal-like key repeat: first repeat after 0.5 s, then 30 Hz.
xset r rate 500 30
set +u
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/install/setup.bash
set -u
roslaunch colmag_ros colmag_keyboard_gazebo_teleop.launch spawn_pick_objects:=true \
  >"$OUT/roslaunch.log" 2>&1 &

wait_window() {  # wait_window <name regex> -> window id
  local w=""
  for _ in $(seq 1 180); do
    w=$(xdotool search --onlyvisible --name "$1" 2>/dev/null | head -1) && [ -n "$w" ] && break
    sleep 1
  done
  [ -n "$w" ] || { echo "window '$1' did not appear" >&2; exit 1; }
  echo "$w"
}

# Layout: dashboard left (0,160), Gazebo right, keyboard terminal at the bottom.
DASH=$(wait_window 'COLMAG Magnetic Trajectory Dashboard')
GZ=$(wait_window '^Gazebo$')
xdotool windowsize "$GZ" 1150 835
xdotool windowmove "$GZ" 790 85
xdotool windowmove "$DASH" 0 160
for _ in $(seq 1 120); do
  rostopic echo -n 1 /joint_states/position >/dev/null 2>&1 && break
  sleep 1
done
sleep 5

# Gazebo GUI camera: from (1.35, -0.95, 0.95) looking at (0.40, 0, 0.22).
POSE=$(python3 - <<'EOF'
import math
cx, cy, cz, tx, ty, tz = 1.35, -0.95, 0.95, 0.40, 0.0, 0.22
dx, dy, dz = tx - cx, ty - cy, tz - cz
yaw = math.atan2(dy, dx)
pitch = math.atan2(-dz, math.hypot(dx, dy))
q = (-math.sin(pitch / 2) * math.sin(yaw / 2), math.sin(pitch / 2) * math.cos(yaw / 2),
     math.cos(pitch / 2) * math.sin(yaw / 2), math.cos(pitch / 2) * math.cos(yaw / 2))
print("position {x: %f y: %f z: %f} orientation {x: %f y: %f z: %f w: %f}"
      % ((cx, cy, cz) + q))
EOF
)
gz topic -p /gazebo/default/user_camera/joy_pose -m "$POSE"

cat >/tmp/teleop_terminal.sh <<'EOF'
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/install/setup.bash
echo '$ rosrun colmag_ros keyboard_cartesian_teleop_node.py'
rosrun colmag_ros keyboard_cartesian_teleop_node.py
EOF
xterm -fa 'DejaVu Sans Mono' -fs 11 -bg '#0F172A' -fg '#E2E8F0' -b 8 \
  -T 'keyboard teleop' -geometry 200x8+0+925 -e bash /tmp/teleop_terminal.sh &
TERM_WIN=$(wait_window 'keyboard teleop')
xdotool windowmove "$TERM_WIN" 0 925
xdotool windowsize "$TERM_WIN" 1920 155
xdotool windowraise "$DASH"
sleep 3

glide() {  # glide x0 y0 x1 y1 steps
  local i
  for i in $(seq 1 "$5"); do
    xdotool mousemove $(( $1 + ($3 - $1) * i / $5 )) $(( $2 + ($4 - $2) * i / $5 ))
    sleep 0.03
  done
}
SWITCH_X=966 SWITCH_Y=197   # "Switch to TELEOP" / "Switch to TASK" on the dashboard

xdotool mousemove 700 1000
T0=$(python3 -c "import time; print(time.time())")
ffmpeg -loglevel error -y -f x11grab -framerate 15 -video_size 1920x920 -i :99.0+0,160 \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$OUT/pick_and_place.mp4" &
FFMPEG=$!
sleep 1.5
glide 700 1000 "$SWITCH_X" "$SWITCH_Y" 15; sleep 0.4
xdotool click 1; sleep 0.3
glide "$SWITCH_X" "$SWITCH_Y" 1500 1000 12
sleep 1.0
python3 "$DEMO/pick_and_place_keys.py" "$OUT/keys.txt" "$T0"
sleep 1.0
glide 1500 1000 "$SWITCH_X" "$SWITCH_Y" 15; sleep 0.4
xdotool click 1; sleep 0.3
glide "$SWITCH_X" "$SWITCH_Y" 700 1000 10
sleep 2
kill -INT "$FFMPEG"
wait "$FFMPEG" || true

# GIF: 1280 px wide, 12 fps, with the held key drawn on the terminal strip.
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
LABELS=""
while read -r start end label; do
  LABELS="$LABELS,drawtext=fontfile=$FONT:text='key  $label':x=w-tw-40:y=h-95:fontsize=34:fontcolor=white:box=1:boxcolor=0x4F46E5:boxborderw=14:enable='between(t,$start,$end)'"
done <"$OUT/keys.txt"
ffmpeg -loglevel error -y -i "$OUT/pick_and_place.mp4" -vf \
  "${LABELS#,},fps=12,scale=1280:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  "$OUT/teleop_pick_and_place.gif"

rosservice call /gazebo/get_model_state "{model_name: pick_cube, relative_entity_name: world}" \
  | grep -A3 'position:'
ls -l "$OUT/teleop_pick_and_place.gif"
