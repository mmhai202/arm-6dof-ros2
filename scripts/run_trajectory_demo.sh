#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
ROS_LOG_DIR_PATH="/tmp/arm_trajectory_demo_logs"
ACTION_NAME="/arm_controller/follow_joint_trajectory"
STARTUP_TIMEOUT_SECONDS=20
POSE_SEQUENCE=(reach_forward left_reach reach_high wrist_down inspection home)

cleanup() {
  if [[ -n "${launch_pid:-}" ]] && kill -0 "${launch_pid}" >/dev/null 2>&1; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}

wait_for_action_server() {
  local deadline
  deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))

  while (( SECONDS < deadline )); do
    if python3 "${ROOT_DIR}/scripts/send_trajectory_goal.py" \
      --server-timeout 1 \
      --wait-only >/tmp/arm_trajectory_demo.wait.log 2>&1; then
      return 0
    fi
    sleep 1
  done

  return 1
}

run_pose_goal() {
  local pose_name="$1"
  local step_label="$2"

  echo
  echo "== ${step_label}: ${pose_name} =="
  python3 "${ROOT_DIR}/scripts/send_trajectory_goal.py" \
    --pose "${pose_name}" \
    --duration 3 \
    --server-timeout 10 \
    --result-timeout 10
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
echo "== Launch bringup headless =="
set +u
source "${WORKSPACE_SETUP}"
set -u
mkdir -p "${ROS_LOG_DIR_PATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR_PATH}"

trap cleanup EXIT
ros2 launch arm_bringup view_arm.launch.py use_rviz:=false >/tmp/arm_trajectory_demo.launch.log 2>&1 &
launch_pid=$!

if ! wait_for_action_server; then
  echo "[error] Action server ${ACTION_NAME} is not ready."
  echo "[hint] Check /tmp/arm_trajectory_demo.launch.log and /tmp/arm_trajectory_demo.wait.log"
  exit 1
fi

for index in "${!POSE_SEQUENCE[@]}"; do
  run_pose_goal "${POSE_SEQUENCE[$index]}" "Send trajectory goal $((index + 1))"
done

echo
echo "Trajectory demo passed."
