#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
KILTED_WS_ROOT="${KILTED_WS_ROOT:-${ROOT_DIR}/../ros2_ws_kilted}"
_KILTED_HOST_SOURCED=0
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  _KILTED_HOST_SOURCED=1
fi

_kilted_host_fail() {
  local message="$1"
  echo "ERROR: ${message}" >&2
  if (( _KILTED_HOST_SOURCED == 1 )); then
    return 1
  fi
  exit 1
}

if [[ ! -f /opt/ros/kilted/setup.bash ]]; then
  _kilted_host_fail "/opt/ros/kilted/setup.bash not found. Run scripts/setup_kilted_host_camera_lab.sh first."
fi

export ROS_WS_ROOT="${KILTED_WS_ROOT}"
_kilted_host_had_nounset=0
case $- in
  *u*) _kilted_host_had_nounset=1 ;;
esac
set +u
source /opt/ros/kilted/setup.bash

if [[ -f "${KILTED_WS_ROOT}/install/setup.bash" ]]; then
  source "${KILTED_WS_ROOT}/install/setup.bash"
fi

if (( _kilted_host_had_nounset == 1 )); then
  set -u
fi
