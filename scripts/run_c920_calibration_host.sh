#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/env_kilted_host.sh"

VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_E524F92F-video-index0}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-camera}"
CAMERA_NAME="${CAMERA_NAME:-hd_pro_webcam_c920}"
CAMERA_INFO_URL="${CAMERA_INFO_URL:-file://${ROOT_DIR}/src/robolubo/config/camera/logitech_c920_640x480.yaml}"
IMAGE_WIDTH="${IMAGE_WIDTH:-640}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-480}"
FRAMERATE="${FRAMERATE:-30}"
PIXEL_FORMAT="${PIXEL_FORMAT:-YUYV}"
OUTPUT_ENCODING="${OUTPUT_ENCODING:-rgb8}"
CAMERA_AUTO_FOCUS="${CAMERA_AUTO_FOCUS:-false}"
CAMERA_FOCUS_ABSOLUTE="${CAMERA_FOCUS_ABSOLUTE:-0}"
CAMERA_AUTO_EXPOSURE="${CAMERA_AUTO_EXPOSURE:-3}"
CAMERA_EXPOSURE_DYNAMIC_FRAMERATE="${CAMERA_EXPOSURE_DYNAMIC_FRAMERATE:-false}"
CHESSBOARD_SIZE="${CHESSBOARD_SIZE:-8x6}"
SQUARE_SIZE="${SQUARE_SIZE:-0.024}"

cleanup() {
  pkill -TERM -f -- "${ROS_PREFIX:-/opt/ros/kilted}/lib/v4l2_camera/v4l2_camera_node --ros-args -r __node:=camera -r __ns:=/${CAMERA_NAMESPACE}" >/dev/null 2>&1 || true
  pkill -TERM -f -- "${ROS_PREFIX:-/opt/ros/kilted}/lib/tf2_ros/static_transform_publisher --x 0.14 --y 0.0 --z 0.12" >/dev/null 2>&1 || true
  pkill -TERM -f -- "${ROS_PREFIX:-/opt/ros/kilted}/lib/tf2_ros/static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --roll -1.57079632679" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

ros2 launch robolubo camera.launch.py \
  namespace:="${CAMERA_NAMESPACE}" \
  camera_name:="${CAMERA_NAME}" \
  camera_info_url:="${CAMERA_INFO_URL}" \
  video_device:="${VIDEO_DEVICE}" \
  image_width:="${IMAGE_WIDTH}" \
  image_height:="${IMAGE_HEIGHT}" \
  framerate:="${FRAMERATE}" \
  pixel_format:="${PIXEL_FORMAT}" \
  output_encoding:="${OUTPUT_ENCODING}" \
  auto_focus:="${CAMERA_AUTO_FOCUS}" \
  focus_absolute:="${CAMERA_FOCUS_ABSOLUTE}" \
  auto_exposure:="${CAMERA_AUTO_EXPOSURE}" \
  exposure_dynamic_framerate:="${CAMERA_EXPOSURE_DYNAMIC_FRAMERATE}" \
  > /tmp/c920_camera_calibration_launch.log 2>&1 &

sleep 5

exec ros2 run camera_calibration cameracalibrator \
  --no-service-check \
  --camera_name "${CAMERA_NAME}" \
  --size "${CHESSBOARD_SIZE}" \
  --square "${SQUARE_SIZE}" \
  --ros-args \
  --remap image:="/${CAMERA_NAMESPACE}/image_raw" \
  --remap camera:="/${CAMERA_NAMESPACE}"
