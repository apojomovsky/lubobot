#!/usr/bin/env bash
# build.sh — trigger colcon build on the Jetson over SSH
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ -f "$ROOT_DIR/.env" ]]; then
  source "$ROOT_DIR/.env"
else
  echo "ERROR: .env not found." >&2
  exit 1
fi

REMOTE="${JETSON_USER}@${JETSON_HOST}"
REMOTE_WS="~/${JETSON_WORKSPACE}"

echo "→ Building on ${REMOTE}:${REMOTE_WS}"

ssh "$REMOTE" bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd ${REMOTE_WS}
  colcon build --symlink-install 2>&1
  echo '✓ Build done'
"
