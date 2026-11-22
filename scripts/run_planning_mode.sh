#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/install_dependencies.sh
source "${ROOT_DIR}/scripts/install_dependencies.sh"
ROS_LOG_DIR_PATH="${ROS_LOG_DIR:-/tmp/arm_planning_mode_logs}"
use_rviz="true"
auto_home="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      use_rviz="false"
      shift
      ;;
    --auto-home)
      auto_home="true"
      shift
      ;;
    *)
      echo "[error] Unknown option: $1"
      echo "Usage: $0 [--headless] [--auto-home]"
      exit 1
      ;;
  esac
done

arm_activate_conda
arm_check_dependencies planning
arm_source_workspace arm_planning custom_planning.launch.py

echo
echo "== Launch custom planning mode =="
echo "[hint] New shell:"
echo "       source scripts/install_dependencies.sh && arm_activate_conda"
echo "       source install/setup.bash"
echo "       ros2 run arm_planning call_planning_service.py --list-poses"
echo "       ros2 run arm_planning call_planning_service.py --pose home"
echo "       ros2 run arm_planning call_planning_service.py --pose-sequence home reach_forward inspection home"
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
exec ros2 launch arm_planning custom_planning.launch.py \
  use_rviz:="${use_rviz}" \
  auto_home:="${auto_home}"
