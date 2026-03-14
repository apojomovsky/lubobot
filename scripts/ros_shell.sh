#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/ros_common.sh"

GPU="$(detect_gpu)"
setup_gpu_env "${GPU}"
COMPOSE_ARGS="$(compose_files "${GPU}" "${ROOT_DIR}")"

set +e
docker compose ${COMPOSE_ARGS} run --rm --no-deps -u "${HOST_UID}:${HOST_GID}" dev bash
status=$?
set -e

if [[ "${status}" -eq 0 || "${status}" -eq 1 || "${status}" -eq 130 ]]; then
  exit 0
fi

exit "${status}"
