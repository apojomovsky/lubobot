#!/usr/bin/env bash
# setup_local.sh — one-time setup for the PC development environment
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Local PC setup ==="

echo "→ Enabling multicast on loopback (required for ROS_LOCALHOST_ONLY=1)"
sudo ip link set lo multicast on
sudo mkdir -p /etc/systemd/network
echo '[Match]
Name=lo

[Link]
Multicast=yes' | sudo tee /etc/systemd/network/10-lo-multicast.network > /dev/null
echo "✓ Loopback multicast enabled (persistent)"

echo "→ Installing rosdep if missing"
if ! command -v rosdep &>/dev/null; then
  sudo apt install -y python3-rosdep
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
echo "✓ rosdep ready"

echo ""
echo "=== Local setup complete ==="
echo "Build the workspace:  scripts/build_local.sh"
echo "Load the env:         source scripts/env.sh"
