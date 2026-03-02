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

echo "→ Installing zenoh-bridge-ros2dds via Eclipse Zenoh apt repo (may prompt for Jetson sudo password)"
ssh -t "$REMOTE" "
  set -e
  if command -v zenoh-bridge-ros2dds &>/dev/null; then
    echo '✓ zenoh-bridge-ros2dds already installed, skipping'
  else
    curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
    echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee /etc/apt/sources.list.d/zenoh.list
    sudo apt update -qq
    sudo apt install -y zenoh-bridge-ros2dds
    echo '✓ zenoh-bridge-ros2dds installed'
  fi
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
