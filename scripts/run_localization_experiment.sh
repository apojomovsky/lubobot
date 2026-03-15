#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WATCHER="${SCRIPT_DIR}/odom_filtered_watch.py"
CMD_VEL_SEQUENCE="${SCRIPT_DIR}/publish_cmd_vel_sequence.py"
if [[ -d /workspaces/ros2_ws/log ]]; then
  OUT_BASE="/workspaces/ros2_ws/log/localization_experiments"
else
  OUT_BASE="${ROOT_DIR}/ros2_ws/log/localization_experiments"
fi
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/${STAMP}"
DURATION="${1:-30}"
BASE_SERIAL_PORT="${BASE_SERIAL_PORT:-/dev/ttyUSB0}"
IMU_PORT="${IMU_PORT:-/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_397036513135-if00}"

mkdir -p "${OUT_DIR}"

RAW_LOG="${OUT_DIR}/raw.log"
EKF_LOG="${OUT_DIR}/ekf.log"
BASE_LAUNCH_LOG="${OUT_DIR}/base.launch.log"
IMU_LAUNCH_LOG="${OUT_DIR}/imu.launch.log"
LOCALIZATION_LAUNCH_LOG="${OUT_DIR}/localization.launch.log"
CMD_VEL_LOG="${OUT_DIR}/cmd_vel_sequence.log"
declare -a STACK_PIDS=()
declare -a EXPERIMENT_PIDS=()
declare -a STACK_PATTERNS=(
  "/opt/ros/humble/lib/create_driver/create_driver --ros-args -r __node:=create_driver"
  "/workspaces/ros2_ws/install/imu_bridge/lib/imu_bridge/imu_bridge_node --ros-args -r __node:=imu_bridge"
  "/opt/ros/humble/lib/imu_filter_madgwick/imu_filter_madgwick_node --ros-args -r __node:=imu_filter"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x -0.07 --y 0.0 --z 0.0"
  "/opt/ros/humble/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node"
)

cleanup() {
  for pid in "${EXPERIMENT_PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in "${STACK_PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  local pattern=""
  for pattern in "${STACK_PATTERNS[@]}"; do
    pkill -TERM -f -- "${pattern}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

start_stack() {
  stdbuf -oL ros2 launch robolubo base.launch.py \
    serial_port:="${BASE_SERIAL_PORT}" \
    >"${BASE_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")

  stdbuf -oL ros2 launch robolubo imu.launch.py \
    port:="${IMU_PORT}" \
    >"${IMU_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")

  stdbuf -oL ros2 launch robolubo localization.launch.py \
    publish_tf:=false \
    >"${LOCALIZATION_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")
}

cat <<EOF
Localization experiment starting.

Output directory:
  ${OUT_DIR}

Bringing up:
  - base.launch.py         (${BASE_SERIAL_PORT})
  - imu.launch.py          (${IMU_PORT})
  - localization.launch.py
  - scripted /cmd_vel sequence

Motion sequence:
  1. stay still for 5s
  2. rotate slowly in place left for 5s
  3. drive forward slowly for 5s
  4. rotate slowly in place right for 5s
  5. drive forward slowly for 5s
  6. stop

Required running nodes:
  none; this script starts them
EOF

start_stack
sleep 5

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odom \
  | stdbuf -oL "${WATCHER}" --label raw \
  | tee "${RAW_LOG}" >/dev/null &
EXPERIMENT_PIDS+=("$!")

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odometry/filtered \
  | stdbuf -oL "${WATCHER}" --label ekf \
  | tee "${EKF_LOG}" >/dev/null &
EXPERIMENT_PIDS+=("$!")

stdbuf -oL timeout "${DURATION}" "${CMD_VEL_SEQUENCE}" \
  >"${CMD_VEL_LOG}" 2>&1 &
EXPERIMENT_PIDS+=("$!")

sleep 1

for remaining in $(seq "${DURATION}" -1 1); do
  printf '\rCollecting data... %2ss remaining' "${remaining}"
  sleep 1
done
printf '\n'

for pid in "${EXPERIMENT_PIDS[@]}"; do
  wait "${pid}" || true
done

cat <<EOF
Experiment complete.

Captured files:
  ${RAW_LOG}
  ${EKF_LOG}
  ${BASE_LAUNCH_LOG}
  ${IMU_LAUNCH_LOG}
  ${LOCALIZATION_LAUNCH_LOG}
  ${CMD_VEL_LOG}

Next step:
  Send me the output directory path above and I will inspect the results.
EOF
