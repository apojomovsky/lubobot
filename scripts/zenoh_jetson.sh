#!/usr/bin/env bash
# zenoh_jetson.sh — start the zenoh router+bridge on the Jetson over SSH
# Runs in the background on the Jetson; output is shown locally.
# Press Ctrl+C to stop tailing (the bridge keeps running on the Jetson).
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
REMOTE_CFG="~/${JETSON_WORKSPACE}/zenoh/jetson.json5"
LOG_FILE="/tmp/zenoh-bridge.log"

echo "→ Deploying zenoh config to Jetson"
ssh "$REMOTE" "mkdir -p ~/${JETSON_WORKSPACE}/zenoh"
rsync -az "$ROOT_DIR/zenoh/jetson.json5" "${REMOTE}:${REMOTE_CFG}"

echo "→ Killing any existing zenoh bridge on Jetson"
ssh "$REMOTE" "pkill -f zenoh-bridge-ros2dds 2>/dev/null && echo '  killed old instance' || echo '  none running'"

echo "→ Starting zenoh-bridge-ros2dds on ${REMOTE} (Ctrl+C to stop)"
echo ""
ssh -t "$REMOTE" "
  source /opt/ros/humble/setup.bash
  zenoh-bridge-ros2dds -c ${REMOTE_CFG}
"
