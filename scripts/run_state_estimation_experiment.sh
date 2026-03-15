#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WATCHER="${SCRIPT_DIR}/odom_filtered_watch.py"
CMD_VEL_SEQUENCE="${SCRIPT_DIR}/publish_cmd_vel_sequence.py"

if [[ -d /workspaces/ros2_ws/log ]]; then
  OUT_BASE="/workspaces/ros2_ws/log/state_estimation_experiments"
else
  OUT_BASE="${ROOT_DIR}/ros2_ws/log/state_estimation_experiments"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/${STAMP}"
DURATION="${1:-35}"
BASE_SERIAL_PORT="${BASE_SERIAL_PORT:-/dev/ttyUSB0}"
IMU_PORT="${IMU_PORT:-/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_397036513135-if00}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/usb-GENERAL_GENERAL_WEBCAM_JH3503_20230616_v001-video-index0}"

mkdir -p "${OUT_DIR}"

RAW_LOG="${OUT_DIR}/raw.log"
EKF_LOG="${OUT_DIR}/ekf.log"
SLAM_LOG="${OUT_DIR}/slam.log"
BASE_LAUNCH_LOG="${OUT_DIR}/base.launch.log"
IMU_LAUNCH_LOG="${OUT_DIR}/imu.launch.log"
LOCALIZATION_LAUNCH_LOG="${OUT_DIR}/localization.launch.log"
CAMERA_LAUNCH_LOG="${OUT_DIR}/camera.launch.log"
ORBSLAM_LAUNCH_LOG="${OUT_DIR}/orbslam.launch.log"
CMD_VEL_LOG="${OUT_DIR}/cmd_vel_sequence.log"
declare -a STACK_PIDS=()
declare -a EXPERIMENT_PIDS=()

cleanup() {
  for pid in "${EXPERIMENT_PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in "${STACK_PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
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

  stdbuf -oL ros2 launch robolubo camera.launch.py \
    video_device:="${VIDEO_DEVICE}" \
    >"${CAMERA_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")

  stdbuf -oL ros2 launch robolubo orbslam_mono.launch.py \
    image_topic:=/camera/image_raw \
    use_pangolin:=false \
    use_rviz:=false \
    >"${ORBSLAM_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))

  while (( SECONDS < deadline )); do
    if ros2 topic list | grep -Fxq "${topic}"; then
      return 0
    fi
    sleep 1
  done

  echo "ERROR: timed out waiting for topic ${topic}" >&2
  return 1
}

cat <<EOF
State-estimation experiment starting.

Output directory:
  ${OUT_DIR}

Bringing up:
  - base.launch.py         (${BASE_SERIAL_PORT})
  - imu.launch.py          (${IMU_PORT})
  - localization.launch.py
  - camera.launch.py       (${VIDEO_DEVICE})
  - orbslam_mono.launch.py
  - scripted /cmd_vel sequence

Motion sequence:
  1. stay still for 5s
  2. rotate slowly in place left for 5s
  3. drive forward slowly for 5s
  4. rotate slowly in place right for 5s
  5. drive forward slowly for 5s
  6. stop

Recorded topics:
  - /odom              -> ${RAW_LOG}
  - /odometry/filtered -> ${EKF_LOG}
  - /orb_slam3/pose    -> ${SLAM_LOG}

Required running nodes:
  none; this script starts them
EOF

start_stack
sleep 5

wait_for_topic /odom 20
wait_for_topic /odometry/filtered 20
wait_for_topic /camera/image_raw 20
wait_for_topic /orb_slam3/pose 30

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odom \
  | stdbuf -oL "${WATCHER}" --label raw \
  | tee "${RAW_LOG}" >/dev/null &
EXPERIMENT_PIDS+=("$!")

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odometry/filtered \
  | stdbuf -oL "${WATCHER}" --label ekf \
  | tee "${EKF_LOG}" >/dev/null &
EXPERIMENT_PIDS+=("$!")

stdbuf -oL timeout "${DURATION}" ros2 topic echo /orb_slam3/pose \
  | stdbuf -oL "${WATCHER}" --label slam \
  | tee "${SLAM_LOG}" >/dev/null &
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
  ${SLAM_LOG}
  ${BASE_LAUNCH_LOG}
  ${IMU_LAUNCH_LOG}
  ${LOCALIZATION_LAUNCH_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${CMD_VEL_LOG}

Next step:
  Send me the output directory path above and I will inspect the results.
EOF
