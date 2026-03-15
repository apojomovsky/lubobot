#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
KILTED_WS_ROOT="${KILTED_WS_ROOT:-${ROOT_DIR}/ros2_ws_kilted}"

if [[ ! -f /opt/ros/kilted/setup.bash ]]; then
  echo "ERROR: /opt/ros/kilted/setup.bash not found." >&2
  echo "Run scripts/setup_kilted_host_camera_lab.sh first." >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_WS_ROOT="${KILTED_WS_ROOT}"
source /opt/ros/kilted/setup.bash

if [[ -f "${KILTED_WS_ROOT}/install/setup.bash" ]]; then
  source "${KILTED_WS_ROOT}/install/setup.bash"
fi
