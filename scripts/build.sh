#!/usr/bin/env bash

set -euo pipefail

if [[ $# -eq 1 && ( "$1" == -h || "$1" == --help ) ]]; then
  echo "Usage: $0 [--help]"
  echo "Build the workspace using the existing Conda environment."
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--help]" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/setup.sh
source "${ROOT_DIR}/scripts/setup.sh"
arm_activate_conda
arm_check_dependencies all
arm_build_workspace
