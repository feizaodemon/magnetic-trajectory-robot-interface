#!/usr/bin/env bash
# Gazebo runtime smoke test, run inside the noetic-gazebo image by the CI job
# gazebo-smoke. Starts Xvfb for the Tk dashboard, launches the keyboard TELEOP
# profile with headless Gazebo and the pick objects, then runs gazebo_smoke.py.
# Gazebo only: no FR3 launch, controller or hardware.
set -euo pipefail

apt-get update -qq
apt-get install -y -qq --no-install-recommends xvfb >/dev/null
Xvfb :99 -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1

set +u
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/install/setup.bash
set -u

roslaunch colmag_ros colmag_keyboard_gazebo_teleop.launch \
  headless:=true spawn_pick_objects:=true >/tmp/roslaunch.log 2>&1 &
LAUNCH_PID=$!

# The driver stamps commands with ROS time, so it must start after the launch
# has set /use_sim_time; otherwise it would stamp wall time against sim time.
for _ in $(seq 1 120); do
  rosparam get /use_sim_time >/dev/null 2>&1 && break
  sleep 1
done

status=0
python3 "$(dirname "$0")/gazebo_smoke.py" || status=$?

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" || true
if [ "$status" -ne 0 ]; then
  echo "---- roslaunch log (last 200 lines) ----"
  tail -n 200 /tmp/roslaunch.log
fi
exit "$status"
