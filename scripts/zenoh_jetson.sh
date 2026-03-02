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

echo "→ Starting zenoh-bridge-ros2dds on ${REMOTE} (log: ${LOG_FILE})"
ssh "$REMOTE" "
  source /opt/ros/humble/setup.bash
  pkill -f zenoh-bridge-ros2dds 2>/dev/null || true
  nohup zenoh-bridge-ros2dds -c ${REMOTE_CFG} > ${LOG_FILE} 2>&1 &
  echo \"✓ zenoh bridge started (PID \$!)\"
"

echo ""
echo "Tailing Jetson zenoh log (Ctrl+C to stop tailing, bridge keeps running)..."
ssh "$REMOTE" "tail -f ${LOG_FILE}"
