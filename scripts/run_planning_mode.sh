#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
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

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] Missing ROS setup: ${ROS_SETUP}"
  exit 1
fi

echo "== Dependency check =="
"${ROOT_DIR}/scripts/check_dependencies.sh"

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
echo "== Launch custom planning mode =="
echo "[hint] New shell:"
echo "       source /opt/ros/humble/setup.bash"
echo "       source install/setup.bash"
echo "       ros2 run arm_planning call_planning_service.py --list-poses"
echo "       ros2 run arm_planning call_planning_service.py --pose home"
echo "       ros2 run arm_planning call_planning_service.py --pose-sequence home reach_forward inspection home"
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
set +u
source "${WORKSPACE_SETUP}"
set -u
exec ros2 launch arm_planning custom_planning.launch.py \
  use_rviz:="${use_rviz}" \
  auto_home:="${auto_home}"
