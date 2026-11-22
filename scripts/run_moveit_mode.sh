#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/install_dependencies.sh
source "${ROOT_DIR}/scripts/install_dependencies.sh"
ROS_LOG_DIR_PATH="${ROS_LOG_DIR:-/tmp/arm_moveit_mode_logs}"
use_moveit_rviz="true"
launch_file="moveit.launch.py"
task_mode="execute"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      use_moveit_rviz="false"
      shift
      ;;
    --pick-place)
      launch_file="shelf_pick_place.launch.py"
      shift
      ;;
    --scene-only)
      launch_file="shelf_pick_place.launch.py"
      task_mode="scene"
      shift
      ;;
    --plan-only)
      launch_file="shelf_pick_place.launch.py"
      task_mode="plan"
      shift
      ;;
    *)
      echo "[error] Unknown option: $1"
      echo "Usage: $0 [--headless] [--pick-place | --scene-only | --plan-only]"
      exit 1
      ;;
  esac
done

arm_activate_conda
arm_check_dependencies moveit
arm_source_workspace arm_moveit_config "${launch_file}"

echo
echo "== Launch MoveIt mode =="
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
if [[ "${launch_file}" == shelf_pick_place.launch.py ]]; then
  echo "[task] Shelf to floor box: ${task_mode} (parallel gripper, fake hardware)."
  exec ros2 launch arm_moveit_config "${launch_file}" use_moveit_rviz:="${use_moveit_rviz}" task_mode:="${task_mode}"
fi
echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
echo "[hint] Wait for the initial move to 'home' before dragging the goal."
exec ros2 launch arm_moveit_config "${launch_file}" use_moveit_rviz:="${use_moveit_rviz}"
