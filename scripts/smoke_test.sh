#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
URDF_OUTPUT="/tmp/arm_smoke_test.urdf"
ROS_LOG_DIR_PATH="/tmp/arm_smoke_test_logs"
LAUNCH_TIMEOUT_SECONDS=12

echo "== Dependency check =="
"${ROOT_DIR}/scripts/check_dependencies.sh"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] Missing ROS setup: ${ROS_SETUP}"
  exit 1
fi

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
echo "== Check URDF =="
set +u
source "${WORKSPACE_SETUP}"
set -u
xacro "${ROOT_DIR}/src/arm_description/urdf/arm.urdf.xacro" > "${URDF_OUTPUT}"
check_urdf "${URDF_OUTPUT}"

echo
echo "== Headless launch smoke test =="
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"

set +e
timeout "${LAUNCH_TIMEOUT_SECONDS}s" ros2 launch arm_bringup view_arm.launch.py use_rviz:=false
launch_exit_code=$?
set -e

if [[ "${launch_exit_code}" -eq 124 ]]; then
  echo "Smoke test passed: launch stayed alive until timeout."
  exit 0
fi

if [[ "${launch_exit_code}" -eq 0 ]]; then
  echo "Smoke test passed: launch exited cleanly before timeout."
  exit 0
fi

echo "Smoke test failed: launch exited with code ${launch_exit_code}."
exit "${launch_exit_code}"
