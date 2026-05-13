#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"

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
echo "== Launch custom planning service mode =="
echo "[hint] List poses: python3 scripts/call_planning_service.py --list-poses"
echo "[hint] Execute one pose: python3 scripts/call_planning_service.py --pose home"
echo "[hint] Execute a sequence: python3 scripts/call_planning_service.py --pose-sequence home reach_forward inspection home"
set +u
source "${WORKSPACE_SETUP}"
set -u
ros2 launch arm_planning custom_planning.launch.py
