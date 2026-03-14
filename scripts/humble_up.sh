#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${ROOT_DIR}/docker/compose.humble.yml"

mkdir -p "${ROOT_DIR}/ros2_ws/build" "${ROOT_DIR}/ros2_ws/install" "${ROOT_DIR}/ros2_ws/log"

if [[ ! -d "${ROOT_DIR}/external/ros2_orb_slam3/.git" ]]; then
  echo "ERROR: external/ros2_orb_slam3 is missing." >&2
  echo "Run scripts/setup_external.sh first." >&2
  exit 1
fi

if [[ -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

docker compose -f "${COMPOSE_FILE}" up -d --build
