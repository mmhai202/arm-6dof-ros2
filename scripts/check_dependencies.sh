#!/usr/bin/env bash

set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
with_gazebo=0
with_moveit=0

for arg in "$@"; do
  case "${arg}" in
    --with-gazebo)
      with_gazebo=1
      ;;
    --with-moveit)
      with_moveit=1
      ;;
    *)
      echo "[error] Unknown option: ${arg}"
      echo "Usage: $0 [--with-gazebo] [--with-moveit]"
      exit 1
      ;;
  esac
done

system_commands=(
  python3
  colcon
  cmake
)

ros_commands=(
  ros2
  xacro
)

if [[ "${with_gazebo}" -eq 1 ]]; then
  system_commands+=(gazebo)
fi

ros_packages=(
  xacro
  robot_state_publisher
  rviz2
  controller_manager
  hardware_interface
  joint_state_broadcaster
  joint_trajectory_controller
)

if [[ "${with_gazebo}" -eq 1 ]]; then
  ros_packages+=(
    gazebo_ros
    gazebo_ros2_control
  )
fi

if [[ "${with_moveit}" -eq 1 ]]; then
  ros_packages+=(
    moveit_kinematics
    moveit_planners_ompl
    moveit_ros_move_group
    moveit_ros_visualization
    moveit_simple_controller_manager
  )
fi

missing=0

echo "== System commands =="
for cmd in "${system_commands[@]}"; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    echo "[ok] ${cmd}"
  else
    echo "[missing] ${cmd}"
    missing=1
  fi
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo
  echo "[missing] ${ROS_SETUP}"
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

echo
echo "== ROS commands =="
for cmd in "${ros_commands[@]}"; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    echo "[ok] ${cmd}"
  else
    echo "[missing] ${cmd}"
    missing=1
  fi
done

echo
echo "== ROS packages =="
for pkg in "${ros_packages[@]}"; do
  if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[ok] ${pkg}"
  else
    echo "[missing] ${pkg}"
    missing=1
  fi
done

echo
if [[ "${missing}" -eq 0 ]]; then
  echo "Dependency check passed."
else
  echo "Dependency check failed."
  exit 1
fi
