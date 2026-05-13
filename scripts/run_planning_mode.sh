#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
backend="custom"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend)
      if [[ $# -lt 2 ]]; then
        echo "[error] Missing value for --backend"
        exit 1
      fi
      backend="$2"
      shift 2
      ;;
    --moveit)
      if [[ $# -lt 2 ]]; then
        echo "[error] Missing value for --moveit"
        exit 1
      fi
      case "$2" in
        true)
          backend="moveit"
          ;;
        false)
          backend="custom"
          ;;
        *)
          echo "[error] --moveit expects true or false"
          exit 1
          ;;
      esac
      shift 2
      ;;
    *)
      echo "[error] Unknown option: $1"
      echo "Usage: $0 [--backend custom|moveit] [--moveit true|false]"
      exit 1
      ;;
  esac
done

case "${backend}" in
  custom)
    exec "${ROOT_DIR}/scripts/run_custom_planning_mode.sh"
    ;;
  moveit)
    exec "${ROOT_DIR}/scripts/run_moveit_demo.sh"
    ;;
  *)
    echo "[error] Unsupported backend: ${backend}"
    echo "[hint] Use --backend custom or --backend moveit"
    exit 1
    ;;
esac
