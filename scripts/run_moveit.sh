#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_LOG_DIR_PATH="${ROS_LOG_DIR:-/tmp/arm_moveit_mode_logs}"
launch_file="moveit.launch.py"

usage() {
  echo "Usage: $0 [--pick-place|--warehouse]"
  echo "Start MoveIt and RViz with Gazebo physics running in the background."
  echo "  --pick-place  Open the shelf-to-box scene; start with Run Pick & Place in RViz."
  echo "  --warehouse  Open the factory cell; start with Start Orders in RViz."
  echo "  -h, --help    Show this help."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --pick-place) launch_file="shelf_pick_place.launch.py" ;;
    --warehouse) launch_file="warehouse.launch.py" ;;
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
launch_args=(use_moveit_rviz:=true)
if [[ "${launch_file}" == warehouse.launch.py ]]; then
  echo "[task] Click Start Orders in RViz to fulfill both orders and rearrange blocked stock."
  launch_args+=(task_mode:=scene)
elif [[ "${launch_file}" == shelf_pick_place.launch.py ]]; then
  echo "[task] Click Run Pick & Place in RViz to start the full cycle."
  launch_args+=(task_mode:=scene)
else
  echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
  echo "[hint] Wait for the initial move to 'home' before dragging the goal."
fi
exec ros2 launch arm_moveit_config "${launch_file}" "${launch_args[@]}"
