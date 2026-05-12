#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
workspace_dirs=(
  "${ROOT_DIR}/build"
  "${ROOT_DIR}/install"
  "${ROOT_DIR}/log"
)

cleanup_workspace() {
  echo "== Clean workspace =="
  for dir_path in "${workspace_dirs[@]}"; do
    if [[ -e "${dir_path}" ]]; then
      echo "[remove] ${dir_path}"
      rm -rf "${dir_path}"
    else
      echo "[skip] ${dir_path}"
    fi
  done

  echo
  echo "== Remove Python cache directories =="
  while IFS= read -r cache_dir; do
    echo "[remove] ${cache_dir}"
    rm -rf "${cache_dir}"
  done < <(find "${ROOT_DIR}" -type d -name "__pycache__" -print)

  echo
  echo "== Remove Python cache files =="
  while IFS= read -r cache_file; do
    echo "[remove] ${cache_file}"
    rm -f "${cache_file}"
  done < <(find "${ROOT_DIR}" -type f \( -name "*.pyc" -o -name "*.pyo" \) -print)
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] Missing ROS setup: ${ROS_SETUP}"
  exit 1
fi

echo "== Dependency check =="
"${ROOT_DIR}/scripts/check_dependencies.sh" --with-moveit

echo
cleanup_workspace

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
echo "== Launch MoveIt demo =="
echo "[hint] In RViz, use the MotionPlanning panel with group 'arm'."
echo "[hint] The baseline bringup auto-homes first, so wait a moment before planning."
set +u
source "${WORKSPACE_SETUP}"
set -u
ros2 launch arm_moveit_config moveit.launch.py
