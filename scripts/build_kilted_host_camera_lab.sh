#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/env_kilted_host.sh"

WORKERS="${KILTED_BUILD_WORKERS:-$(nproc)}"
SKIP_KEYS="${KILTED_ROSDEP_SKIP_KEYS:-}"

mkdir -p "${ROS_WS_ROOT}/src" "${ROS_WS_ROOT}/build" "${ROS_WS_ROOT}/install" "${ROS_WS_ROOT}/log"
ln -sfn "${ROOT_DIR}/src/robolubo" "${ROS_WS_ROOT}/src/robolubo"
ln -sfn "${ROOT_DIR}/external/ros2_orb_slam3" "${ROS_WS_ROOT}/src/ros2_orb_slam3"

rosdep update --rosdistro kilted

ROSDEP_CMD=(
  rosdep install
  -r
  --from-paths "${ROS_WS_ROOT}/src"
  --ignore-src
  -y
  --rosdistro kilted
)

if [[ -n "${SKIP_KEYS}" ]]; then
  ROSDEP_CMD+=(--skip-keys "${SKIP_KEYS}")
fi

"${ROSDEP_CMD[@]}"

cd "${ROS_WS_ROOT}"
colcon build \
  --symlink-install \
  --parallel-workers "${WORKERS}" \
  --packages-select ros2_orb_slam3 robolubo
