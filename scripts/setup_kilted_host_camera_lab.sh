#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
KILTED_WS_ROOT="${KILTED_WS_ROOT:-${ROOT_DIR}/ros2_ws_kilted}"

if [[ ! -f /etc/os-release ]]; then
  echo "ERROR: /etc/os-release not found." >&2
  exit 1
fi

. /etc/os-release
if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "ERROR: host-native Kilted workflow currently expects Ubuntu." >&2
  exit 1
fi

if [[ ! -f /opt/ros/kilted/setup.bash ]]; then
  cat >&2 <<EOF
ERROR: ROS 2 Kilted binaries are not installed.

Install Kilted first, then rerun this script.
EOF
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  v4l-utils \
  ros-kilted-v4l2-camera \
  ros-kilted-image-proc \
  ros-kilted-camera-calibration \
  ros-kilted-rviz2 \
  ros-kilted-tf2-ros

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update --rosdistro kilted

mkdir -p "${KILTED_WS_ROOT}/src" "${KILTED_WS_ROOT}/build" "${KILTED_WS_ROOT}/install" "${KILTED_WS_ROOT}/log"

ln -sfn "${ROOT_DIR}" "${KILTED_WS_ROOT}/src/lubobot"

if [[ ! -d "${ROOT_DIR}/external/ros2_orb_slam3/.git" ]]; then
  echo "ERROR: external/ros2_orb_slam3 is missing." >&2
  echo "Run scripts/setup_external.sh first." >&2
  exit 1
fi

ln -sfn "${ROOT_DIR}/external/ros2_orb_slam3" "${KILTED_WS_ROOT}/src/ros2_orb_slam3"

cat <<EOF
Kilted host camera lab is ready.

Workspace:
  ${KILTED_WS_ROOT}

Next steps:
  source scripts/env_kilted_host.sh
  scripts/build_kilted_host_camera_lab.sh
EOF
