#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env_kilted_host.sh"

declare -a PATTERNS=(
  'mono_node_cpp'
  'v4l2_camera_node'
  'static_transform_publisher'
)

kill_patterns() {
  local signal="$1"
  local pattern
  local pid
  for pattern in "${PATTERNS[@]}"; do
    while read -r pid; do
      [[ -n "${pid}" ]] || continue
      kill "-${signal}" "${pid}" >/dev/null 2>&1 || true
    done < <(pgrep -f -- "${pattern}" || true)
  done
}

report_remaining() {
  ps -ef | rg 'mono_node_cpp|v4l2_camera_node|static_transform_publisher' || true
}

echo "Stopping stale ORB-SLAM host processes..."
kill_patterns TERM
sleep 2
kill_patterns KILL
sleep 1

echo "Refreshing ROS 2 daemon..."
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "Remaining matching processes:"
report_remaining

echo
echo "Current ROS nodes:"
ros2 node list || true
