#!/usr/bin/env bash
# setup_jetson.sh — one-time setup of the Jetson Nano
# Run this once from your PC after fresh OS install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ -f "$ROOT_DIR/.env" ]]; then
  source "$ROOT_DIR/.env"
else
  echo "ERROR: .env not found." >&2
  exit 1
fi

REMOTE="${JETSON_USER}@${JETSON_HOST}"
DESIRED_HOSTNAME="jetson-lubo"

# zenoh-bridge-ros2dds version and aarch64 binary URL
ZENOH_VERSION="1.7.2"
ZENOH_BRIDGE_URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-standalone.zip"

echo "=== One-time Jetson setup ==="
echo "Target: ${REMOTE}"
echo ""

echo "→ Setting hostname to ${DESIRED_HOSTNAME} (may prompt for Jetson sudo password)"
ssh -t "$REMOTE" "
  sudo hostnamectl set-hostname ${DESIRED_HOSTNAME}
  # Ensure 127.0.1.1 entry exists for new hostname
  if ! grep -q '${DESIRED_HOSTNAME}' /etc/hosts; then
    sudo sed -i 's/127.0.1.1.*/127.0.1.1\t${DESIRED_HOSTNAME}/' /etc/hosts || \
    echo '127.0.1.1\t${DESIRED_HOSTNAME}' | sudo tee -a /etc/hosts
  fi
  echo '✓ Hostname set to ${DESIRED_HOSTNAME}'
"

echo "→ Creating workspace directory"
ssh "$REMOTE" "mkdir -p ~/${JETSON_WORKSPACE}/src && echo '✓ Workspace ready'"

echo "→ Installing zenoh-bridge-ros2dds (aarch64) (may prompt for Jetson sudo password)"
ssh -t "$REMOTE" "
  set -e
  if command -v zenoh-bridge-ros2dds &>/dev/null; then
    echo '✓ zenoh-bridge-ros2dds already installed, skipping'
    exit 0
  fi
  cd /tmp
  curl -Lo zenoh-bridge.zip '${ZENOH_BRIDGE_URL}'
  unzip -o zenoh-bridge.zip zenoh-bridge-ros2dds
  sudo mv zenoh-bridge-ros2dds /usr/local/bin/
  sudo chmod +x /usr/local/bin/zenoh-bridge-ros2dds
  rm -f zenoh-bridge.zip libzenoh_plugin_ros2dds.so 2>/dev/null || true
  echo '✓ zenoh-bridge-ros2dds installed'
"

echo "→ Checking ROS2 Humble installation"
ssh "$REMOTE" "
  source /opt/ros/humble/setup.bash
  echo \"✓ ROS2 \$(rosversion -d) ready\"
"

echo ""
echo "=== Setup complete ==="
echo "You can now SSH with: scripts/ssh.sh"
echo "Or use: ssh ${REMOTE} (IP may change; set jetson-lubo.local via mDNS)"
echo ""
echo "TIP: Add 'jetson-lubo.local' to your /etc/hosts on this PC if mDNS isn't working:"
echo "  echo '${JETSON_HOST}  jetson-lubo.local' | sudo tee -a /etc/hosts"
