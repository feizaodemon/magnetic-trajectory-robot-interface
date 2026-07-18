#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/install/setup.bash
set -u

exec "$@"
