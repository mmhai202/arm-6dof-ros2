#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="/tmp/arm_moveit_smoke_test_logs"
LAUNCH_TIMEOUT_SECONDS=20

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
echo "== MoveIt headless smoke test =="
set +u
source "${WORKSPACE_SETUP}"
set -u
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"

set +e
timeout "${LAUNCH_TIMEOUT_SECONDS}s" ros2 launch arm_moveit_config moveit.launch.py use_moveit_rviz:=false
launch_exit_code=$?
set -e

if [[ "${launch_exit_code}" -eq 124 ]]; then
  echo "MoveIt smoke test passed: launch stayed alive until timeout."
  exit 0
fi

if [[ "${launch_exit_code}" -eq 0 ]]; then
  echo "MoveIt smoke test passed: launch exited cleanly before timeout."
  exit 0
fi

echo "MoveIt smoke test failed: launch exited with code ${launch_exit_code}."
exit "${launch_exit_code}"
