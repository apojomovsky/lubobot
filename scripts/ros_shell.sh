#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/ros_common.sh"

GPU="$(detect_gpu)"
SERIAL="$(detect_serial)"
VIDEO="$(detect_video)"
INPUT="$(detect_input)"
setup_gpu_env "${GPU}"
COMPOSE_ARGS="$(compose_files "${GPU}" "${ROOT_DIR}" "${SERIAL}" "${VIDEO}" "${INPUT}")"

if ! container_running; then
  bash "${SCRIPT_DIR}/ros_up.sh"
fi

exec docker compose ${COMPOSE_ARGS} exec dev gosu "${HOST_USERNAME}" bash
