#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="/tmp/arm_custom_planning_service_smoke_logs"
LAUNCH_LOG_PATH="/tmp/arm_custom_planning_service_smoke.launch.log"
WAIT_TIMEOUT_SECONDS=20

cleanup() {
  if [[ -n "${launch_pid:-}" ]] && kill -0 "${launch_pid}" >/dev/null 2>&1; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}

wait_for_list_service() {
  local deadline
  deadline=$((SECONDS + WAIT_TIMEOUT_SECONDS))

  while (( SECONDS < deadline )); do
    if python3 "${ROOT_DIR}/scripts/call_planning_service.py" \
      --list-poses \
      --service-timeout 1 >/tmp/arm_custom_planning_service_smoke.wait.log 2>&1; then
      return 0
    fi
    sleep 1
  done

  return 1
}

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
echo "== Launch custom planning service headless =="
set +u
source "${WORKSPACE_SETUP}"
set -u
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"

trap cleanup EXIT
ros2 launch arm_planning custom_planning.launch.py use_rviz:=false auto_home:=false >"${LAUNCH_LOG_PATH}" 2>&1 &
launch_pid=$!

if ! wait_for_list_service; then
  echo "[error] Planning services are not ready."
  echo "[hint] Check ${LAUNCH_LOG_PATH}"
  exit 1
fi

echo
echo "== Execute planning service request =="
python3 "${ROOT_DIR}/scripts/call_planning_service.py" \
  --pose-sequence home reach_forward inspection home \
  --service-timeout 10

echo
echo "Custom planning service smoke test passed."
