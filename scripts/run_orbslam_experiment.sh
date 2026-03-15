#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WATCHER="${SCRIPT_DIR}/odom_filtered_watch.py"
CMD_VEL_SEQUENCE="${SCRIPT_DIR}/publish_cmd_vel_sequence_gentle.py"

if [[ -d /workspaces/ros2_ws/log ]]; then
  OUT_BASE="/workspaces/ros2_ws/log/orbslam_experiments"
else
  OUT_BASE="${ROOT_DIR}/ros2_ws/log/orbslam_experiments"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/${STAMP}"
DURATION="${1:-42}"
BASE_SERIAL_PORT="${BASE_SERIAL_PORT:-/dev/ttyUSB0}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/usb-GENERAL_GENERAL_WEBCAM_JH3503_20230616_v001-video-index0}"

mkdir -p "${OUT_DIR}"

RAW_LOG="${OUT_DIR}/raw.log"
SLAM_LOG="${OUT_DIR}/slam.log"
BASE_LAUNCH_LOG="${OUT_DIR}/base.launch.log"
CAMERA_LAUNCH_LOG="${OUT_DIR}/camera.launch.log"
ORBSLAM_LAUNCH_LOG="${OUT_DIR}/orbslam.launch.log"
CMD_VEL_LOG="${OUT_DIR}/cmd_vel_sequence.log"
PREFLIGHT_LOG="${OUT_DIR}/preflight.log"
declare -a STACK_PIDS=()
declare -a EXPERIMENT_PIDS=()
declare -a STACK_PATTERNS=(
  "/opt/ros/humble/lib/create_driver/create_driver --ros-args -r __node:=create_driver"
  "/opt/ros/humble/lib/v4l2_camera/v4l2_camera_node --ros-args -r __node:=camera -r __ns:=/camera"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0.14 --y 0.0 --z 0.12"
  "/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --roll -1.57079632679"
  "/workspaces/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/mono_node_cpp --ros-args -r __node:=orbslam_mono"
)
INTERRUPTED=0
SHUTDOWN_DONE=0

stop_pid_list() {
  local signal="$1"
  shift
  local pid=""
  for pid in "$@"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "-${signal}" "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

stop_matching_processes() {
  local signal="$1"
  shift
  local pattern=""
  for pattern in "$@"; do
    if [[ -n "${pattern}" ]]; then
      pkill "-${signal}" -f -- "${pattern}" >/dev/null 2>&1 || true
    fi
  done
}

wait_pid_list() {
  local timeout_s="$1"
  shift
  local deadline=$((SECONDS + timeout_s))
  local remaining=1
  local pid=""

  while (( SECONDS < deadline )); do
    remaining=0
    for pid in "$@"; do
      if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
        remaining=1
        break
      fi
    done
    if (( remaining == 0 )); then
      return 0
    fi
    sleep 1
  done

  return 1
}

shutdown_all() {
  if (( SHUTDOWN_DONE == 1 )); then
    return 0
  fi
  SHUTDOWN_DONE=1

  stop_pid_list INT "${EXPERIMENT_PIDS[@]:-}"
  stop_pid_list INT "${STACK_PIDS[@]:-}"
  stop_matching_processes INT "${STACK_PATTERNS[@]}"
  wait_pid_list 5 "${EXPERIMENT_PIDS[@]:-}" "${STACK_PIDS[@]:-}" || true

  stop_pid_list TERM "${EXPERIMENT_PIDS[@]:-}"
  stop_pid_list TERM "${STACK_PIDS[@]:-}"
  stop_matching_processes TERM "${STACK_PATTERNS[@]}"
  wait_pid_list 5 "${EXPERIMENT_PIDS[@]:-}" "${STACK_PIDS[@]:-}" || true

  stop_pid_list KILL "${EXPERIMENT_PIDS[@]:-}"
  stop_pid_list KILL "${STACK_PIDS[@]:-}"
  stop_matching_processes KILL "${STACK_PATTERNS[@]}"
}

on_interrupt() {
  INTERRUPTED=1
  printf '\nInterrupt received. Stopping experiment and launched processes...\n' >&2
  shutdown_all
}

cleanup() {
  shutdown_all
}

trap cleanup EXIT
trap on_interrupt INT TERM

start_stack() {
  stdbuf -oL ros2 launch robolubo base.launch.py \
    serial_port:="${BASE_SERIAL_PORT}" \
    >"${BASE_LAUNCH_LOG}" 2>&1 &
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

wait_for_message() {
  local topic="$1"
  local timeout_s="${2:-10}"
  local log_file="${3:-/dev/null}"

  if timeout "${timeout_s}" ros2 topic echo "${topic}" --once >"${log_file}" 2>&1; then
    return 0
  fi

  echo "ERROR: timed out waiting for a message on ${topic}" >&2
  return 1
}

publisher_count() {
  local topic="$1"
  ros2 topic info "${topic}" -v 2>/dev/null | grep -c 'Endpoint type: PUBLISHER' || true
}

require_no_publishers() {
  local topic="$1"
  local count

  count="$(publisher_count "${topic}")"
  if [[ "${count}" != "0" ]]; then
    echo "ERROR: expected 0 publishers on ${topic} before startup, found ${count}" >&2
    return 1
  fi
}

require_exactly_one_publisher() {
  local topic="$1"
  local count

  count="$(publisher_count "${topic}")"
  if [[ "${count}" != "1" ]]; then
    echo "ERROR: expected exactly 1 publisher on ${topic}, found ${count}" >&2
    return 1
  fi
}

assert_clean_graph() {
  require_no_publishers /odom
  require_no_publishers /camera/image_raw
  require_no_publishers /camera/camera_info
  require_no_publishers /orb_slam3/pose
}

preflight_check() {
  : >"${PREFLIGHT_LOG}"

  echo "Preflight checks:" | tee -a "${PREFLIGHT_LOG}"

  wait_for_topic /odom 20
  wait_for_message /odom 5 "${OUT_DIR}/odom.preflight.log"
  require_exactly_one_publisher /odom
  echo "  PASS /odom" | tee -a "${PREFLIGHT_LOG}"

  wait_for_topic /camera/image_raw 20
  wait_for_message /camera/image_raw 8 "${OUT_DIR}/camera_image_raw.preflight.log"
  require_exactly_one_publisher /camera/image_raw
  echo "  PASS /camera/image_raw" | tee -a "${PREFLIGHT_LOG}"

  wait_for_topic /camera/camera_info 20
  wait_for_message /camera/camera_info 8 "${OUT_DIR}/camera_info.preflight.log"
  require_exactly_one_publisher /camera/camera_info
  echo "  PASS /camera/camera_info" | tee -a "${PREFLIGHT_LOG}"

  wait_for_topic /orb_slam3/pose 30
  wait_for_message /orb_slam3/pose 12 "${OUT_DIR}/orb_slam3_pose.preflight.log"
  require_exactly_one_publisher /orb_slam3/pose
  echo "  PASS /orb_slam3/pose" | tee -a "${PREFLIGHT_LOG}"
}

cat <<EOF
ORB-SLAM experiment starting.

Output directory:
  ${OUT_DIR}

Bringing up:
  - base.launch.py         (${BASE_SERIAL_PORT})
  - camera.launch.py       (${VIDEO_DEVICE})
  - orbslam_mono.launch.py
  - gentle scripted /cmd_vel sequence

Motion sequence:
  1. stay still for 5s
  2. rotate very slowly left for 8s
  3. drive forward very slowly for 8s
  4. rotate very slowly right for 8s
  5. drive forward very slowly for 8s
  6. stop

Recorded topics:
  - /odom           -> ${RAW_LOG}
  - /orb_slam3/pose -> ${SLAM_LOG}

Required running nodes:
  none; this script starts them
EOF

if ! assert_clean_graph; then
  cat <<EOF
Refusing to start: the ROS graph is already dirty.

Stop any existing base, camera, or ORB-SLAM publishers and rerun this script.
EOF
  exit 1
fi

start_stack
sleep 5

if ! preflight_check; then
  cat <<EOF
Preflight failed.

Relevant logs:
  ${BASE_LAUNCH_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${PREFLIGHT_LOG}

The scripted motion sequence was not started.
EOF
  exit 1
fi

stdbuf -oL timeout "${DURATION}" ros2 topic echo /odom \
  | stdbuf -oL "${WATCHER}" --label raw \
  | tee "${RAW_LOG}" >/dev/null &
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

if (( INTERRUPTED == 1 )); then
  cat <<EOF
Experiment interrupted.

Captured files so far:
  ${RAW_LOG}
  ${SLAM_LOG}
  ${BASE_LAUNCH_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${CMD_VEL_LOG}
  ${PREFLIGHT_LOG}

Next step:
  Send me the output directory path above and I will inspect the partial results.
EOF
  exit 130
fi

cat <<EOF
Experiment complete.

Captured files:
  ${RAW_LOG}
  ${SLAM_LOG}
  ${BASE_LAUNCH_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${CMD_VEL_LOG}
  ${PREFLIGHT_LOG}

Next step:
  Send me the output directory path above and I will inspect the results.
EOF
