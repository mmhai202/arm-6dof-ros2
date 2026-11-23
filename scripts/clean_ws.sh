#!/usr/bin/env bash

set -euo pipefail

if [[ $# -eq 1 && ( "$1" == -h || "$1" == --help ) ]]; then
  echo "Usage: $0 [--help]"
  echo "Remove build/, install/, log/ and Python caches; preserve the Conda environment."
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--help]" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
conda_prefix="$(realpath -m "${ARM_CONDA_PREFIX:-${ROOT_DIR}/.conda}")"
workspace_dirs=(
  "${ROOT_DIR}/build"
  "${ROOT_DIR}/install"
  "${ROOT_DIR}/log"
)

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
