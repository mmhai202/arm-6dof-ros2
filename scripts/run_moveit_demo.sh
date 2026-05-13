#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="${ROS_LOG_DIR:-/tmp/arm_moveit_demo_logs}"
use_moveit_rviz="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      use_moveit_rviz="false"
      shift
      ;;
    *)
      echo "[error] Unknown option: $1"
      echo "Usage: $0 [--headless]"
      exit 1
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] Missing ROS setup: ${ROS_SETUP}"
  exit 1
fi

echo "== Dependency check =="
"${ROOT_DIR}/scripts/check_dependencies.sh" --with-moveit

echo
echo "== Build workspace =="
set +u
source "${ROS_SETUP}"
set -u
colcon build --symlink-install --base-paths "${ROOT_DIR}/src"

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "[error] Missing workspace setup: ${WORKSPACE_SETUP}"
  exit 1
fi

echo
echo "== Launch MoveIt demo =="
echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
echo "[hint] The baseline bringup auto-homes first, so wait a moment before planning."
echo "[hint] Run ./scripts/clean_ws.sh first if you want a fully clean rebuild."
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
set +u
source "${WORKSPACE_SETUP}"
set -u
exec ros2 launch arm_moveit_config moveit.launch.py use_moveit_rviz:="${use_moveit_rviz}"
