#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 [--stop-only|--help]"
  echo "Stop this workspace's ROS, Gazebo and RViz processes, including suspended jobs."
  echo "Remove build/, install/, log/ and Python caches; preserve the Conda environment."
  echo "  --stop-only  Stop processes without deleting build artifacts or caches."
}

stop_only=false
if [[ $# -eq 1 && ( "$1" == -h || "$1" == --help ) ]]; then
  usage
  exit 0
fi
if [[ $# -eq 1 && "$1" == --stop-only ]]; then
  stop_only=true
elif [[ $# -ne 0 ]]; then
  usage >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
conda_prefix="$(realpath -m "${ARM_CONDA_PREFIX:-${ROOT_DIR}/.conda}")"
workspace_dirs=(
  "${ROOT_DIR}/build"
  "${ROOT_DIR}/install"
  "${ROOT_DIR}/log"
)

echo "== Stop workspace processes =="
# Use only the standard library: cleanup must also work without ROS or Conda.
python3 - "${ROOT_DIR}" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time

root = Path(sys.argv[1]).resolve()
ros_commands = {"gzserver", "gzclient", "gazebo", "rviz2", "move_group",
                "robot_state_publisher", "ros2_control_node", "spawner"}
packages = {"arm_bringup", "arm_control", "arm_description", "arm_moveit_config",
            "arm_perception", "arm_task_interfaces"}


def snapshot():
    result = {}
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            if path.stat().st_uid != os.getuid():
                continue
            fields = (path / "stat").read_text().rsplit(") ", 1)[1].split()
            args = (path / "cmdline").read_bytes().decode(errors="replace").rstrip("\0").split("\0")
            result[int(path.name)] = {
                "ppid": int(fields[1]), "state": fields[0], "start": fields[19],
                "args": args, "exe": os.readlink(path / "exe"),
                "cwd": os.readlink(path / "cwd"),
            }
        except (OSError, ValueError, IndexError):
            continue  # A process can exit while /proc is being read.
    return result


def inside(value, cwd="/"):
    return bool(value) and (Path(cwd) / value).resolve().is_relative_to(root)


def project_environment(pid):
    try:
        environment = (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
        return any(entry.startswith(b"AMENT_PREFIX_PATH=") and any(
            inside(value) for value in entry.decode(errors="replace").split("=", 1)[1].split(":"))
            for entry in environment)
    except OSError:
        return False


def belongs(pid, process):
    args, cwd = process["args"], process["cwd"]
    if inside(process["exe"]) or inside(args[0], cwd):
        return True
    # Match script paths, rather than arbitrary shell command strings or data arguments.
    interpreter = Path(process["exe"]).name
    command = args
    if (interpreter.startswith("python") or interpreter in {"bash", "sh", "zsh"}):
        index = 1
        while index < len(args) and args[index].startswith("-"):
            option = args[index]
            if (option == "-" or option.startswith(("-c", "-m"))
                    or (interpreter in {"bash", "sh", "zsh"} and "c" in option and not option.startswith("--"))):
                index = len(args)  # Inline code is not a script path.
                break
            index += 2 if option in {"-X", "-W", "-o", "--check-hash-based-pycs"} else 1
        command = args[index:]
        if command and inside(command[0], cwd):
            return True
    if (len(command) > 2 and Path(command[0]).name == "ros2"
            and command[1] == "launch" and command[2] in packages):
        return inside(cwd) or project_environment(pid)
    if Path(process["exe"]).name in ros_commands:
        if inside(cwd):
            return True
        return project_environment(pid)
    return False


# Never signal this cleaner, its invoking shell, or any of their ancestors.
initial = snapshot()
protected = {os.getpid()}
parent = os.getppid()
while parent in initial and parent not in protected:
    protected.add(parent)
    parent = initial[parent]["ppid"]
tracked = {}


def active():
    processes = snapshot()
    selected = {pid for pid, process in processes.items()
                if pid not in protected and (tracked.get(pid) == process["start"] or belongs(pid, process))}
    while True:
        children = {pid for pid, process in processes.items()
                    if pid not in protected and process["ppid"] in selected}
        if children <= selected:
            break
        selected.update(children)
    for pid in selected:
        tracked[pid] = processes[pid]["start"]
    return {pid: processes[pid] for pid in selected if processes[pid]["state"] != "Z"}


for sig, grace in ((signal.SIGINT, 8.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0)):
    deadline = time.monotonic() + grace
    sent = set()
    while True:
        pending = active()
        if not pending:
            break
        for pid, process in pending.items():
            identity = (pid, process["start"])
            if identity in sent:
                continue
            try:
                # Recheck the creation time before signalling a PID from the snapshot.
                fields = (Path("/proc") / str(pid) / "stat").read_text().rsplit(") ", 1)[1].split()
                if fields[19] != process["start"]:
                    continue
                print(f"[stop] {sig.name} PID {pid} ({Path(process['exe']).name})", flush=True)
                os.kill(pid, sig)
                if sig != signal.SIGKILL and process["state"] in {"T", "t"}:
                    # Queue shutdown first, then wake a Ctrl+Z job to handle it.
                    os.kill(pid, signal.SIGCONT)
                sent.add(identity)
            except (ProcessLookupError, FileNotFoundError):
                pass
        if time.monotonic() >= deadline:
            break
        time.sleep(0.1)
    if not pending:
        break
remaining = active()
if remaining:
    print("[error] Processes still alive: " + ", ".join(map(str, remaining)), file=sys.stderr)
    sys.exit(1)  # Keep build artifacts intact if stopping failed.
print("[ok] No workspace processes remain running or suspended.")
PY

if [[ "${stop_only}" == true ]]; then
  exit 0
fi

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
done < <(find "${ROOT_DIR}" \( -path "${ROOT_DIR}/.conda" -o -path "${conda_prefix}" \) -prune -o -type d -name "__pycache__" -print)

echo
echo "== Remove Python cache files =="
while IFS= read -r cache_file; do
  echo "[remove] ${cache_file}"
  rm -f "${cache_file}"
done < <(find "${ROOT_DIR}" \( -path "${ROOT_DIR}/.conda" -o -path "${conda_prefix}" \) -prune -o -type f \( -name "*.pyc" -o -name "*.pyo" \) -print)
