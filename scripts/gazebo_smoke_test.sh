#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="/tmp/arm_gazebo_smoke_test_logs"
GAZEBO_HOME_PATH="/tmp/arm_gazebo_home"
LAUNCH_LOG_PATH="/tmp/arm_gazebo_smoke_test.launch.log"
LAUNCH_TIMEOUT_SECONDS=20

echo "== Dependency check (Gazebo) =="
"${ROOT_DIR}/scripts/check_dependencies.sh" --with-gazebo

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
echo "== Gazebo launch smoke test =="
set +u
source "${WORKSPACE_SETUP}"
set -u
mkdir -p "${ROS_LOG_DIR_PATH}"
mkdir -p "${GAZEBO_HOME_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
export HOME="${GAZEBO_HOME_PATH}"

set +e
timeout "${LAUNCH_TIMEOUT_SECONDS}s" ros2 launch arm_bringup gazebo_arm.launch.py \
  use_rviz:=false \
  use_gazebo_gui:=false >"${LAUNCH_LOG_PATH}" 2>&1
launch_exit_code=$?
set -e

if grep -q "process has died" "${LAUNCH_LOG_PATH}"; then
  echo "Gazebo smoke test failed: a launch process died."
  echo "[hint] Check ${LAUNCH_LOG_PATH}"
  exit 1
fi

if ! grep -q "Configured and activated" "${LAUNCH_LOG_PATH}"; then
  echo "Gazebo smoke test failed: controllers did not activate."
  echo "[hint] Check ${LAUNCH_LOG_PATH}"
  exit 1
fi

if [[ "${launch_exit_code}" -eq 124 ]]; then
  echo "Gazebo smoke test passed: launch stayed alive until timeout."
  exit 0
fi

if [[ "${launch_exit_code}" -eq 0 ]]; then
  echo "Gazebo smoke test passed: launch exited cleanly before timeout."
  exit 0
fi

echo "Gazebo smoke test failed: launch exited with code ${launch_exit_code}."
exit "${launch_exit_code}"
