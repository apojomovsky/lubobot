#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WATCHER="${SCRIPT_DIR}/odom_filtered_watch.py"

if [[ -d /workspaces/ros2_ws/log ]]; then
  OUT_BASE="/workspaces/ros2_ws/log/orbslam_manual_diagnostics"
else
  OUT_BASE="${ROOT_DIR}/ros2_ws/log/orbslam_manual_diagnostics"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/${STAMP}"
DURATION="${1:-60}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/usb-GENERAL_GENERAL_WEBCAM_JH3503_20230616_v001-video-index0}"
USE_PANGOLIN="${USE_PANGOLIN:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-0}"
CAMERA_PITCH_RAD="$(python3 -c 'import math,sys; print(f"{math.radians(float(sys.argv[1])):.6f}")' "${CAMERA_PITCH_DEG}")"
ORBSLAM_SETTINGS_FILE="${ORBSLAM_SETTINGS_FILE:-/workspaces/ros2_ws/install/robolubo/share/robolubo/config/orbslam/general_webcam_640x480.yaml}"

mkdir -p "${OUT_DIR}"

SLAM_LOG="${OUT_DIR}/slam.log"
CAMERA_LAUNCH_LOG="${OUT_DIR}/camera.launch.log"
ORBSLAM_LAUNCH_LOG="${OUT_DIR}/orbslam.launch.log"
PREFLIGHT_LOG="${OUT_DIR}/preflight.log"
declare -a STACK_PIDS=()
declare -a RECORDING_PIDS=()
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

  stop_pid_list INT "${RECORDING_PIDS[@]:-}"
  stop_pid_list INT "${STACK_PIDS[@]:-}"
  wait_pid_list 5 "${RECORDING_PIDS[@]:-}" "${STACK_PIDS[@]:-}" || true

  stop_pid_list TERM "${RECORDING_PIDS[@]:-}"
  stop_pid_list TERM "${STACK_PIDS[@]:-}"
  wait_pid_list 5 "${RECORDING_PIDS[@]:-}" "${STACK_PIDS[@]:-}" || true

  stop_pid_list KILL "${RECORDING_PIDS[@]:-}"
  stop_pid_list KILL "${STACK_PIDS[@]:-}"
}

on_interrupt() {
  INTERRUPTED=1
  printf '\nInterrupt received. Stopping diagnostics and launched processes...\n' >&2
  shutdown_all
}

cleanup() {
  shutdown_all
}

trap cleanup EXIT
trap on_interrupt INT TERM

start_stack() {
  stdbuf -oL ros2 launch robolubo camera.launch.py \
    video_device:="${VIDEO_DEVICE}" \
    camera_pitch:="${CAMERA_PITCH_RAD}" \
    >"${CAMERA_LAUNCH_LOG}" 2>&1 &
  STACK_PIDS+=("$!")

  stdbuf -oL ros2 launch robolubo orbslam_mono.launch.py \
    image_topic:=/camera/image_raw \
    settings_file:="${ORBSLAM_SETTINGS_FILE}" \
    use_pangolin:="${USE_PANGOLIN}" \
    use_rviz:="${USE_RVIZ}" \
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
  require_no_publishers /camera/image_raw
  require_no_publishers /camera/camera_info
  require_no_publishers /orb_slam3/pose
}

preflight_check() {
  : >"${PREFLIGHT_LOG}"

  echo "Preflight checks:" | tee -a "${PREFLIGHT_LOG}"

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
ORB-SLAM manual diagnostics starting.

Output directory:
  ${OUT_DIR}

Bringing up:
  - camera.launch.py       (${VIDEO_DEVICE})
  - orbslam_mono.launch.py

Viewer settings:
  - Pangolin: ${USE_PANGOLIN}
  - RViz:     ${USE_RVIZ}
  - Camera pitch: ${CAMERA_PITCH_DEG} deg (${CAMERA_PITCH_RAD} rad)
  - ORB settings: ${ORBSLAM_SETTINGS_FILE}

Manual motion guidance:
  1. wait for the Pangolin window to appear
  2. keep the robot still for 5 seconds
  3. point the camera at a textured, well-lit scene
  4. rotate the robot very slowly by hand for 10-15 seconds
  5. move it forward/back slowly by hand for 10-15 seconds
  6. keep walls, furniture, and room structure in view, not only floor
  7. if Pangolin clearly loses tracking or keeps resetting, press Ctrl-C

Recorded topics:
  - /orb_slam3/pose -> ${SLAM_LOG}

Duration:
  up to ${DURATION}s, or until you press Ctrl-C
EOF

if ! assert_clean_graph; then
  cat <<EOF
Refusing to start: the ROS graph is already dirty.

Stop any existing base, camera, or ORB-SLAM publishers and rerun this script.
Stop any existing camera or ORB-SLAM publishers and rerun this script.
EOF
  exit 1
fi

start_stack
sleep 5

if ! preflight_check; then
  cat <<EOF
Preflight failed.

Relevant logs:
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${PREFLIGHT_LOG}

Manual diagnostics were not started.
EOF
  exit 1
fi

stdbuf -oL timeout "${DURATION}" ros2 topic echo /orb_slam3/pose \
  | stdbuf -oL "${WATCHER}" --label slam \
  | tee "${SLAM_LOG}" >/dev/null &
RECORDING_PIDS+=("$!")

sleep 1

for remaining in $(seq "${DURATION}" -1 1); do
  printf '\rRecording manual diagnostics... %2ss remaining' "${remaining}"
  sleep 1
done
printf '\n'

for pid in "${RECORDING_PIDS[@]}"; do
  wait "${pid}" || true
done

if (( INTERRUPTED == 1 )); then
  cat <<EOF
Manual diagnostics interrupted.

Captured files so far:
  ${SLAM_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${PREFLIGHT_LOG}

Next step:
  Send me the output directory path above and I will inspect the partial results.
EOF
  exit 130
fi

cat <<EOF
Manual diagnostics complete.

Captured files:
  ${SLAM_LOG}
  ${CAMERA_LAUNCH_LOG}
  ${ORBSLAM_LAUNCH_LOG}
  ${PREFLIGHT_LOG}

Next step:
  Send me the output directory path above and I will inspect the results.
EOF
