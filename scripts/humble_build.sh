#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="${ROOT_DIR}/docker/compose.humble.yml"
WORKERS="${HUMBLE_BUILD_WORKERS:-1}"

exec docker compose -f "${COMPOSE_FILE}" exec dev bash -lc "\
  source /opt/ros/humble/setup.bash && \
  cd /workspaces/ros2_ws && \
  rosdep update --rosdistro humble && \
  rosdep install -r --from-paths src --ignore-src -y --rosdistro humble && \
  colcon build --symlink-install --parallel-workers ${WORKERS}"
