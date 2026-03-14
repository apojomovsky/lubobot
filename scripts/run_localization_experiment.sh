#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WATCHER="${SCRIPT_DIR}/odom_filtered_watch.py"
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
TELEOP_LAUNCH_LOG="${OUT_DIR}/teleop.launch.log"

cleanup() {
  jobs -p | xargs -r kill >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

start_stack() {
  stdbuf -oL ros2 launch robolubo base.launch.py \
    serial_port:="${BASE_SERIAL_PORT}" \
    >"${BASE_LAUNCH_LOG}" 2>&1 &

  stdbuf -oL ros2 launch robolubo imu.launch.py \
    port:="${IMU_PORT}" \
    >"${IMU_LAUNCH_LOG}" 2>&1 &

  stdbuf -oL ros2 launch robolubo localization.launch.py \
    publish_tf:=false \
    >"${LOCALIZATION_LAUNCH_LOG}" 2>&1 &

  if command -v xterm >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    stdbuf -oL ros2 launch robolubo teleop_keyboard.launch.py \
      terminal_prefix:='xterm -e' \
      >"${TELEOP_LAUNCH_LOG}" 2>&1 &
  else
    cat <<EOF
No xterm/X11 terminal detected for teleop.
If you want live keyboard control, launch this in another shell:
  ros2 launch robolubo teleop_keyboard.launch.py
EOF
  fi
}

cat <<EOF
Localization experiment starting.

Output directory:
  ${OUT_DIR}

Bringing up:
  - base.launch.py         (${BASE_SERIAL_PORT})
  - imu.launch.py          (${IMU_PORT})
  - localization.launch.py
  - teleop_keyboard.launch.py (if xterm is available)

For the next ${DURATION}s, drive this sequence:
  1. stay still for 5s
  2. rotate slowly in place for 5s
  3. drive forward slowly for 5s
  4. rotate back the other way for 5s
  5. drive forward and stop for the remaining time

Required running nodes:
  none; this script starts them
EOF

start_stack
sleep 5

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odom \
  | stdbuf -oL "${WATCHER}" --label raw \
  | tee "${RAW_LOG}" >/dev/null &

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odometry/filtered \
  | stdbuf -oL "${WATCHER}" --label ekf \
  | tee "${EKF_LOG}" >/dev/null &

sleep 1

for remaining in $(seq "${DURATION}" -1 1); do
  printf '\rCollecting data... %2ss remaining' "${remaining}"
  sleep 1
done
printf '\n'

wait || true

cat <<EOF
Experiment complete.

Captured files:
  ${RAW_LOG}
  ${EKF_LOG}
  ${BASE_LAUNCH_LOG}
  ${IMU_LAUNCH_LOG}
  ${LOCALIZATION_LAUNCH_LOG}
  ${TELEOP_LAUNCH_LOG}

Next step:
  Send me the output directory path above and I will inspect the results.
EOF
