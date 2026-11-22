#!/usr/bin/env bash
# Setup entry point and shared helpers for the two launchers.
# Source this file to use arm_activate_conda without installing or building.

ARM_BASE_DEPENDENCIES=(
  'python'
  'cmake<4'
  'make'
  'ninja'
  'pkg-config'
  'c-compiler'
  'cxx-compiler'
  'colcon-common-extensions'
  'numpy'
  'pyyaml'
  'ros-humble-ros-base'
  'ros-humble-ament-cmake'
  'ros-humble-ament-cmake-python'
  'ros-humble-ament-index-python'
  'ros-humble-rosidl-default-generators'
  'ros-humble-rosidl-default-runtime'
  'ros-humble-rclpy'
  'ros-humble-launch'
  'ros-humble-launch-ros'
  'ros-humble-control-msgs'
  'ros-humble-sensor-msgs'
  'ros-humble-trajectory-msgs'
  'ros-humble-xacro'
  'ros-humble-robot-state-publisher'
  'ros-humble-rviz2'
  'ros-humble-controller-manager'
  'ros-humble-hardware-interface'
  'ros-humble-joint-state-broadcaster'
  'ros-humble-joint-trajectory-controller'
)
ARM_MOVEIT_DEPENDENCIES=(
  'ros-humble-moveit-kinematics'
  'ros-humble-moveit-planners-ompl'
  'ros-humble-moveit-ros-move-group'
  'ros-humble-moveit-ros-visualization'
  'ros-humble-moveit-simple-controller-manager'
)
ARM_EXTRA_DEPENDENCIES=(
  'gazebo=11.*'
  'ros-humble-gazebo-ros'
  'ros-humble-gazebo-ros2-control'
  'ocp'
)

ARM_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARM_CONDA_PREFIX="$(realpath -m "${ARM_CONDA_PREFIX:-${ARM_ROOT_DIR}/.conda}")"
export ARM_CONDA_PREFIX

arm_init_conda() {
  local conda_executable="${CONDA_EXE:-}" conda_base
  if [[ ! -x "${conda_executable}" ]]; then
    conda_executable="$(type -P conda || true)"
  fi
  if [[ -z "${conda_executable}" ]]; then
    echo "[error] Conda not found. Install Miniforge/Miniconda and add conda to PATH." >&2
    return 1
  fi
  conda_base="$("${conda_executable}" info --base)" || return 1
  if [[ "${ARM_CONDA_PREFIX}" == "$(realpath -m "${conda_base}")" ]]; then
    echo "[error] ARM_CONDA_PREFIX must not point to the Conda base environment." >&2
    return 1
  fi
  # Used by install_dependencies.sh after this helper is sourced.
  # shellcheck disable=SC2034
  ARM_CONDA_EXE="${conda_executable}"
  # shellcheck disable=SC1091
  source "${conda_base}/etc/profile.d/conda.sh"
}

arm_activate_conda() {
  if [[ ! -d "${ARM_CONDA_PREFIX}/conda-meta" ]]; then
    echo "[error] Missing Conda environment: ${ARM_CONDA_PREFIX}" >&2
    echo "Run ${ARM_ROOT_DIR}/scripts/install_dependencies.sh first." >&2
    return 1
  fi
  arm_init_conda || return 1

  # Remove system ROS and old workspace overlays inherited from the parent shell.
  # Preserve unrelated paths and ROS_DOMAIN_ID / DDS configuration.
  local var entry cleaned
  local -a entries
  for var in PATH PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH; do
    cleaned=""
    IFS=: read -r -a entries <<< "${!var-}"
    for entry in "${entries[@]}"; do
      case "${entry}" in
        /opt/ros/*|"${ARM_ROOT_DIR}/install"|"${ARM_ROOT_DIR}/install/"*) continue ;;
      esac
      [[ -z "${entry}" ]] && continue
      cleaned+="${cleaned:+:}${entry}"
    done
    if [[ -n "${cleaned}" ]]; then
      export "${var}=${cleaned}"
    else
      unset "${var}"
    fi
  done
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ETC_DIR

  # Third-party activation hooks may reference unset variables.
  local had_nounset=0 status=0
  [[ $- == *u* ]] && had_nounset=1
  set +u
  conda activate "${ARM_CONDA_PREFIX}" || status=$?
  if [[ "${had_nounset}" -eq 1 ]]; then set -u; fi
  [[ "${status}" -eq 0 ]] || return "${status}"
  # ROS activation hooks may prepend other directories. Keep this environment's
  # tools ahead of user/system installations, including ~/.local/bin/colcon.
  export PATH="${CONDA_PREFIX}/bin:${PATH}"
  if [[ ":${CMAKE_PREFIX_PATH:-}:" != *":${CONDA_PREFIX}:"* ]]; then
    export CMAKE_PREFIX_PATH="${CONDA_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  fi
}

# Internal preflight shared by the installer, builder and launchers. Never installs.
arm_check_dependencies() {
  local profile="$1" missing=0 command command_path compiler_var compiler compiler_path spec package
  local -a commands=(python3 ros2 xacro) specs=("${ARM_BASE_DEPENDENCIES[@]}")
  local -a modules=(numpy yaml rclpy ament_index_python launch launch_ros control_msgs sensor_msgs trajectory_msgs)
  case "${profile}" in
    planning) ;;
    moveit) specs+=("${ARM_MOVEIT_DEPENDENCIES[@]}"); modules+=(moveit_msgs) ;;
    all)
      commands+=(colcon cmake make ninja pkg-config)
      specs+=("${ARM_MOVEIT_DEPENDENCIES[@]}")
      modules+=(rosidl_generator_py moveit_msgs)
      ;;
    *) echo "[error] Unknown dependency profile: ${profile}" >&2; return 1 ;;
  esac
  if [[ "${profile}" == all ]]; then
    commands+=(gazebo)
    specs+=("${ARM_EXTRA_DEPENDENCIES[@]}")
    modules+=(OCP.BRepMesh OCP.STEPCAFControl OCP.XCAFDoc)
  fi
  for command in "${commands[@]}"; do
    command_path="$(type -P "${command}" || true)"
    if [[ "${command_path}" != "${CONDA_PREFIX}/bin/"* ]]; then
      echo "[missing] ${command} in ${CONDA_PREFIX}/bin (found: ${command_path:-none})" >&2
      missing=1
    fi
  done
  if [[ "${profile}" == all ]]; then
    for compiler_var in CC CXX; do
      compiler="${!compiler_var-}"
      if [[ -z "${compiler}" ]]; then
        case "${compiler_var}" in CC) compiler=cc ;; CXX) compiler=c++ ;; esac
      fi
      compiler_path="$(type -P "${compiler}" || true)"
      if [[ "${compiler_path}" != "${CONDA_PREFIX}/bin/"* ]]; then
        echo "[missing] ${compiler_var} compiler in ${CONDA_PREFIX}/bin (found: ${compiler_path:-none})" >&2
        missing=1
      elif ! "${compiler_path}" --version >/dev/null 2>&1; then
        echo "[missing] ${compiler_var} compiler is not runnable: ${compiler_path}" >&2
        missing=1
      fi
    done
  fi
  local -a packages=()
  for spec in "${specs[@]}"; do
    if [[ "${spec}" == ros-humble-* ]]; then
      package="${spec#ros-humble-}"
      packages+=("${package//-/_}")
    fi
  done
  if [[ -x "${CONDA_PREFIX}/bin/python3" ]]; then
    # One Python process checks imports and package origins; no solver or downloads.
    if ! "${CONDA_PREFIX}/bin/python3" - "${CONDA_PREFIX}" "${modules[*]}" "${packages[@]}" <<'PY'
import importlib
from pathlib import Path
import sys

prefix = Path(sys.argv[1]).resolve()
failed = False
for module in sys.argv[2].split():
    try:
        importlib.import_module(module)
    except Exception as error:
        print(f"[missing] {module}: {error}", file=sys.stderr)
        failed = True
try:
    from ament_index_python.packages import get_package_prefix
except Exception:
    get_package_prefix = None
    failed = True
packages = set(sys.argv[3:])
if get_package_prefix:
    for package in sorted(packages):
        try:
            location = Path(get_package_prefix(package)).resolve()
            if not location.is_relative_to(prefix):
                raise ValueError(f"found outside Conda: {location}")
        except Exception as error:
            print(f"[missing] {package} in Conda environment: {error}", file=sys.stderr)
            failed = True
sys.exit(1 if failed else 0)
PY
    then
      missing=1
    fi
  else
    missing=1
  fi
  if [[ "${missing}" -ne 0 ]]; then
    echo "[error] Missing or broken dependencies. Run ${ARM_ROOT_DIR}/scripts/install_dependencies.sh" >&2
    return 1
  fi
}

arm_source_workspace() {
  local package="$1" launch_file="$2" workspace_setup="${ARM_ROOT_DIR}/install/setup.bash"
  if [[ ! -f "${workspace_setup}" ]]; then
    echo "[missing] Workspace setup: ${workspace_setup}" >&2
    echo "Run ${ARM_ROOT_DIR}/scripts/install_dependencies.sh first." >&2
    return 1
  fi
  local had_nounset=0 source_status=0
  [[ $- == *u* ]] && had_nounset=1
  set +u
  # shellcheck disable=SC1090
  source "${workspace_setup}" || source_status=$?
  if [[ "${had_nounset}" -eq 1 ]]; then set -u; fi
  if [[ "${source_status}" -ne 0 ]]; then
    echo "[error] Cannot source workspace. Run ${ARM_ROOT_DIR}/scripts/install_dependencies.sh" >&2
    return 1
  fi
  if ! "${CONDA_PREFIX}/bin/python3" - "${ARM_ROOT_DIR}/install" "${package}" "${launch_file}" <<'PY'
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory

try:
    share = Path(get_package_share_directory(sys.argv[2]))
    # Keep the install path lexical: symlink installs point back into src/.
    if not share.is_relative_to(Path(sys.argv[1])):
        raise ValueError(f"package is outside this workspace: {share}")
    launch = share / "launch" / sys.argv[3]
    if not launch.is_file():
        raise FileNotFoundError(launch)
    for module in ("arm_description.launch", "arm_description.resources", "arm_control.poses"):
        __import__(module)
    if sys.argv[2] == "arm_planning":
        __import__("arm_planning.service")
except Exception as error:
    print(f"[missing] Workspace package {sys.argv[2]}: {error}", file=sys.stderr)
    sys.exit(1)
PY
  then
    echo "Run ${ARM_ROOT_DIR}/scripts/install_dependencies.sh first." >&2
    return 1
  fi
}

arm_install_dependencies() {
  local conda_action=create
  arm_init_conda
  if [[ -d "${ARM_CONDA_PREFIX}/conda-meta" ]]; then conda_action=install; fi
  echo "== Install all dependencies: ${ARM_CONDA_PREFIX} =="
  # Avoid the shell wrapper, which may reactivate the caller's environment.
  "${ARM_CONDA_EXE}" "${conda_action}" --yes --prefix "${ARM_CONDA_PREFIX}" \
    --override-channels --strict-channel-priority -c conda-forge -c robostack-humble \
    "${ARM_BASE_DEPENDENCIES[@]}" "${ARM_MOVEIT_DEPENDENCIES[@]}" "${ARM_EXTRA_DEPENDENCIES[@]}"
  arm_activate_conda
  arm_check_dependencies all

  echo "== Build workspace =="
  cd "${ARM_ROOT_DIR}"
  colcon build --symlink-install --base-paths "${ARM_ROOT_DIR}/src" --cmake-args \
    "-DPython_EXECUTABLE=${CONDA_PREFIX}/bin/python" \
    "-DPython3_EXECUTABLE=${CONDA_PREFIX}/bin/python" \
    "-DPYTHON_EXECUTABLE=${CONDA_PREFIX}/bin/python" \
    -DPython3_FIND_STRATEGY=LOCATION -DPython_FIND_STRATEGY=LOCATION
  echo "[ok] Workspace ready. Use run_custom_mode.sh or run_moveit_mode.sh."
}

# Launchers source the helpers above; installation only happens on direct execution.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  set -euo pipefail
  if [[ $# -ne 0 ]]; then
    echo "Usage: $0 (no arguments)" >&2
    exit 1
  fi
  arm_install_dependencies
fi
