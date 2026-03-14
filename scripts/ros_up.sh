#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/ros_common.sh"

GPU="$(detect_gpu)"
SERIAL="$(detect_serial)"
setup_gpu_env "${GPU}"
COMPOSE_ARGS="$(compose_files "${GPU}" "${ROOT_DIR}" "${SERIAL}")"

mkdir -p "${ROOT_DIR}/ros2_ws/build" "${ROOT_DIR}/ros2_ws/install" "${ROOT_DIR}/ros2_ws/log"

if [[ ! -d "${ROOT_DIR}/external/ros2_orb_slam3/.git" ]]; then
  echo "ERROR: external/ros2_orb_slam3 is missing." >&2
  echo "Run scripts/setup_external.sh first." >&2
  exit 1
fi

bash "${SCRIPT_DIR}/sync_rosdep_manifests.sh"

if [[ -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

echo "→ Using GPU mode: ${GPU}"
echo "→ Using serial mode: ${SERIAL}"

docker compose ${COMPOSE_ARGS} up -d --build
