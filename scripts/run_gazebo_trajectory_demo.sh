#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="/tmp/arm_gazebo_trajectory_demo_logs"
GAZEBO_HOME_PATH="/tmp/arm_gazebo_trajectory_demo_home"
LAUNCH_LOG_PATH="/tmp/arm_gazebo_trajectory_demo.launch.log"
WAIT_LOG_PATH="/tmp/arm_gazebo_trajectory_demo.wait.log"
ACTION_NAME="/arm_controller/follow_joint_trajectory"
STARTUP_TIMEOUT_SECONDS=40
GAZEBO_MASTER_PORT_RANGE_START=18000
GAZEBO_MASTER_PORT_RANGE_SIZE=2000
POSE_SEQUENCE=(reach_forward inspection home)

cleanup() {
  if [[ -n "${launch_pid:-}" ]] && kill -0 "${launch_pid}" >/dev/null 2>&1; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}

select_gazebo_master_uri() {
  local attempt
  local port

  if [[ -n "${GAZEBO_MASTER_URI:-}" ]]; then
    export GAZEBO_MASTER_URI
    return
  fi

  for attempt in $(seq 1 20); do
    port=$((GAZEBO_MASTER_PORT_RANGE_START + RANDOM % GAZEBO_MASTER_PORT_RANGE_SIZE))

    if ! command -v ss >/dev/null 2>&1 || ! ss -H -ltn | awk '{print $4}' | grep -Eq ":${port}$"; then
      export GAZEBO_MASTER_URI="http://127.0.0.1:${port}"
      return
    fi
  done

  echo "[error] No free Gazebo master port found in the loopback test range."
  echo "[hint] Set GAZEBO_MASTER_URI manually before rerunning."
  exit 1
}

wait_for_action_server() {
  local deadline
  deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))

  while (( SECONDS < deadline )); do
    if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
      return 1
    fi

    if python3 "${ROOT_DIR}/scripts/send_trajectory_goal.py" \
      --server-timeout 1 \
      --wait-only >"${WAIT_LOG_PATH}" 2>&1 \
      && grep -Eq "Configured and activated .*arm_controller" "${LAUNCH_LOG_PATH}"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

run_pose_sequence() {
  local step_label="$1"

  echo
  echo "== ${step_label} =="
  python3 "${ROOT_DIR}/scripts/send_trajectory_goal.py" \
    --pose-sequence "${POSE_SEQUENCE[@]}" \
    --duration 3 \
    --server-timeout 10 \
    --result-timeout 15
}

report_environment_bootstrap_failure_if_needed() {
  if grep -Eq "Unable to get local interface addresses|Unable to start server\\[open: Operation not permitted\\]|Error creating socket: Operation not permitted" "${LAUNCH_LOG_PATH}" \
    || grep -Eq "Unable to get local interface addresses|Unable to start server\\[open: Operation not permitted\\]" "${GAZEBO_HOME_PATH}"/.gazebo/server-*/gzserver.log 2>/dev/null; then
    echo "[error] Gazebo networking bootstrap was blocked by sandbox or local socket permissions."
    echo "[hint] Re-run outside restricted sandbox or verify local socket permissions."
    echo "[hint] Check ${LAUNCH_LOG_PATH} and ${WAIT_LOG_PATH}"
    exit 1
  fi

  if grep -Eq "Address already in use|bind: Address already in use" "${LAUNCH_LOG_PATH}" \
    || grep -Eq "Address already in use|bind: Address already in use" "${GAZEBO_HOME_PATH}"/.gazebo/server-*/gzserver.log 2>/dev/null; then
    echo "[error] Gazebo master port is already in use."
    echo "[hint] Stop the conflicting Gazebo process or override GAZEBO_MASTER_URI before rerunning."
    echo "[hint] Check ${LAUNCH_LOG_PATH}"
    exit 1
  fi
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] Missing ROS setup: ${ROS_SETUP}"
  exit 1
fi

echo "== Dependency check (Gazebo) =="
"${ROOT_DIR}/scripts/check_dependencies.sh" --with-gazebo

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
echo "== Launch Gazebo bringup headless =="
set +u
source "${WORKSPACE_SETUP}"
set -u
mkdir -p "${ROS_LOG_DIR_PATH}"
mkdir -p "${GAZEBO_HOME_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"
export HOME="${GAZEBO_HOME_PATH}"
select_gazebo_master_uri
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_HOSTNAME="${GAZEBO_HOSTNAME:-127.0.0.1}"

echo "[info] Using GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "[info] Using GAZEBO_IP=${GAZEBO_IP}"

trap cleanup EXIT
ros2 launch arm_bringup gazebo_arm.launch.py \
  use_rviz:=false \
  use_gazebo_gui:=false >"${LAUNCH_LOG_PATH}" 2>&1 &
launch_pid=$!

if ! wait_for_action_server; then
  report_environment_bootstrap_failure_if_needed
  echo "[error] Action server ${ACTION_NAME} is not ready."
  echo "[hint] Check ${LAUNCH_LOG_PATH} and ${WAIT_LOG_PATH}"
  exit 1
fi

run_pose_sequence "Send Gazebo trajectory sequence"

echo
echo "Gazebo trajectory demo passed."
