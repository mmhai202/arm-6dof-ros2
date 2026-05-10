#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

workspace_dirs=(
  "${ROOT_DIR}/build"
  "${ROOT_DIR}/install"
  "${ROOT_DIR}/log"
)

echo "== Remove ROS 2 workspace directories =="
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

echo
echo "Workspace cleanup completed."
