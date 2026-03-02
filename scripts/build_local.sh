#!/usr/bin/env bash
# build_local.sh — build the ROS2 workspace on this PC
# Run once after cloning, and whenever you add new packages.
# After building, re-source scripts/env.sh to pick up the install overlay.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash

cd "$ROOT_DIR"

echo "→ Installing dependencies with rosdep"
rosdep install --from-paths src --ignore-src -r -y

echo "→ Building locally"
colcon build --symlink-install

echo ""
echo "✓ Local build done. Re-source the env to use it:"
echo "  source scripts/env.sh"
