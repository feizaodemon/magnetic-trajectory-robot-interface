#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
mode="${COLMAG_FR3_MODE:-dry-run}"

if [[ -z "${FR3_ROBOT_IP:-}" || "${FR3_ROBOT_IP}" == "__REQUIRED_FR3_ROBOT_IP__" ]]; then
  echo "FR3_ROBOT_IP must be set to the lab robot address." >&2
  exit 64
fi

case "${mode}" in
  dry-run)
    if [[ "${COLMAG_SEND_GOALS:-false}" != "false" ||
          "${COLMAG_HARDWARE_EXECUTION_ENABLED:-false}" != "false" ]]; then
      echo "dry-run requires COLMAG_SEND_GOALS=false and COLMAG_HARDWARE_EXECUTION_ENABLED=false." >&2
      exit 64
    fi
    ;;
  active)
    if [[ "${COLMAG_SEND_GOALS:-}" != "true" ||
          "${COLMAG_HARDWARE_EXECUTION_ENABLED:-}" != "true" ]]; then
      echo "active mode requires COLMAG_SEND_GOALS=true and COLMAG_HARDWARE_EXECUTION_ENABLED=true." >&2
      exit 64
    fi
    ;;
  *)
    echo "COLMAG_FR3_MODE must be dry-run or active." >&2
    exit 64
    ;;
esac

set +u
source /opt/ros/noetic/setup.bash
source /opt/franka_ros_ws/install/setup.bash
source /opt/colmag_ws/install/setup.bash
set -u

exec "$@"
