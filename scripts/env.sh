#!/usr/bin/env bash
# Source this once per terminal to get the project environment:
#   source scripts/env.sh
#
# Then all ros2 commands (topic echo, rviz2, etc.) will use project settings.

ROOT_DIR="${PWD}"

if [[ ! -f "$ROOT_DIR/.env" ]]; then
  echo "ERROR: .env not found. Source this script from the project root:" >&2
  echo "  cd ~/robolubo && source scripts/env.sh" >&2
  return 1
fi

set -a
source "$ROOT_DIR/.env"
set +a

source /opt/ros/humble/setup.bash

# If a local build exists, overlay it so 'ros2 launch robolubo ...' works on PC
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi

echo "✓ robolubo env loaded (JETSON_HOST=$JETSON_HOST, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY)"
