#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/humble_common.sh"

GPU="$(detect_gpu)"
setup_gpu_env "${GPU}"
COMPOSE_ARGS="$(compose_files "${GPU}" "${ROOT_DIR}")"
WORKERS="${HUMBLE_BUILD_WORKERS:-1}"
SKIP_KEYS="${HUMBLE_ROSDEP_SKIP_KEYS:-libcrypto boost_serialization}"

docker compose ${COMPOSE_ARGS} exec -u root dev bash -lc "\
  apt-get update && \
  source /opt/ros/humble/setup.bash && \
  cd /workspaces/ros2_ws && \
  rosdep update --rosdistro humble && \
  rosdep install -r --from-paths src --ignore-src -y --rosdistro humble --skip-keys '${SKIP_KEYS}'"

exec docker compose ${COMPOSE_ARGS} exec dev bash -lc "\
  source /opt/ros/humble/setup.bash && \
  cd /workspaces/ros2_ws && \
  colcon build --symlink-install --parallel-workers ${WORKERS}"
