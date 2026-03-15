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

if ! container_exists; then
  exit 0
fi

if docker stop -t 10 lubobot-humble-dev; then
  exit 0
fi

echo "docker stop failed, forcing removal of lubobot-humble-dev" >&2
docker rm -f lubobot-humble-dev >/dev/null 2>&1 || true
