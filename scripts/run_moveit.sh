#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_LOG_DIR_PATH="${ROS_LOG_DIR:-/tmp/arm_moveit_mode_logs}"
use_gazebo_gui="true"
launch_file="moveit.launch.py"

usage() {
  echo "Usage: $0 [--pick-place] [--headless]"
  echo "Start Gazebo physics and MoveIt with Gazebo GUI and RViz by default."
  echo "  --pick-place  Plan and execute the shelf-to-box demo."
  echo "  --headless    Run Gazebo in the background; keep RViz open."
  echo "  -h, --help    Show this help."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --headless) use_gazebo_gui="false" ;;
    --pick-place) launch_file="shelf_pick_place.launch.py" ;;
    *)
      echo "[error] Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

# shellcheck source=scripts/install_dependencies.sh
source "${ROOT_DIR}/scripts/install_dependencies.sh"
arm_activate_conda
arm_check_dependencies gazebo
arm_source_workspace arm_moveit_config "${launch_file}"

echo
echo "== Launch MoveIt mode =="
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
launch_args=(use_moveit_rviz:=true use_gazebo_gui:="${use_gazebo_gui}")
if [[ "${launch_file}" == shelf_pick_place.launch.py ]]; then
  echo "[task] Shelf to floor box: execute (Gazebo physics)."
  launch_args+=(task_mode:=execute)
else
  echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
  echo "[hint] Wait for the initial move to 'home' before dragging the goal."
fi
exec ros2 launch arm_moveit_config "${launch_file}" "${launch_args[@]}"
