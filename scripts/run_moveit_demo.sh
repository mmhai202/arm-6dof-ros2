#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/install_dependencies.sh
source "${ROOT_DIR}/scripts/install_dependencies.sh"
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

arm_activate_conda
arm_check_dependencies moveit
arm_source_workspace arm_moveit_config moveit.launch.py

echo
echo "== Launch MoveIt demo =="
echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
echo "[hint] Wait for the initial move to 'ready' before dragging the goal."
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
exec ros2 launch arm_moveit_config moveit.launch.py use_moveit_rviz:="${use_moveit_rviz}"
