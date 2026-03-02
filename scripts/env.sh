#!/usr/bin/env bash
# Source this once per terminal to get the project environment:
#   source scripts/env.sh
#
# Then all ros2 commands (topic echo, rviz2, etc.) will use project settings.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ ! -f "$ROOT_DIR/.env" ]]; then
  echo "ERROR: .env not found. Copy .env.example to .env and fill it in." >&2
  return 1
fi

set -a
source "$ROOT_DIR/.env"
set +a

source /opt/ros/humble/setup.bash

echo "✓ robolubo env loaded (JETSON_HOST=$JETSON_HOST, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY)"
