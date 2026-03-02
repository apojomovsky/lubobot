#!/usr/bin/env bash
# zenoh_local.sh — start the zenoh client bridge on this PC
# Requires: zenoh-bridge-ros2dds installed locally (x86_64 build)
# Install: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ -f "$ROOT_DIR/.env" ]]; then
  source "$ROOT_DIR/.env"
else
  echo "ERROR: .env not found." >&2
  exit 1
fi

if ! command -v zenoh-bridge-ros2dds &>/dev/null; then
  echo "ERROR: zenoh-bridge-ros2dds not found in PATH." >&2
  echo "Download the x86_64 binary from:" >&2
  echo "  https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases" >&2
  exit 1
fi
echo "→ Connecting to zenoh router at ${JETSON_HOST}:7447 (Ctrl+C to stop)"
echo ""
source /opt/ros/humble/setup.bash
zenoh-bridge-ros2dds client -e "tcp/${JETSON_HOST}:7447" --ros-localhost-only
