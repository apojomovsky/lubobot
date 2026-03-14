#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${ROOT_DIR}/docker/compose.humble.yml"
WORKERS="${HUMBLE_BUILD_WORKERS:-1}"
SKIP_KEYS="${HUMBLE_ROSDEP_SKIP_KEYS:-libcrypto}"

docker compose -f "${COMPOSE_FILE}" exec -u root dev bash -lc "\
  apt-get update && \
  source /opt/ros/humble/setup.bash && \
  cd /workspaces/ros2_ws && \
  rosdep update --rosdistro humble && \
  rosdep install -r --from-paths src --ignore-src -y --rosdistro humble --skip-keys '${SKIP_KEYS}'"

exec docker compose -f "${COMPOSE_FILE}" exec dev bash -lc "\
  source /opt/ros/humble/setup.bash && \
  cd /workspaces/ros2_ws && \
  colcon build --symlink-install --parallel-workers ${WORKERS}"
