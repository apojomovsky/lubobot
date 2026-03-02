# robolubo

ROS2 Humble robot project for the robolubo platform.
Developed on a PC, deployed to a Jetson Nano (`jetson-lubo`).

---

## Prerequisites

**PC:**
- ROS2 installed
- `rsync`, `ssh`
- `zenoh-bridge-ros2dds` — install via Eclipse Zenoh apt repo:
  ```bash
  curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
  echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee /etc/apt/sources.list.d/zenoh.list
  sudo apt update && sudo apt install zenoh-bridge-ros2dds
  ```

**Jetson Nano:**
- ROS2 Humble installed from debians
- SSH access from PC (`ssh alexis@<jetson-ip>`)
- `curl`, `unzip` (for one-time setup)

---

## One-time Setup

```bash
# 1. Clone and configure
git clone <repo-url> robolubo
cd robolubo
cp .env.example .env
# Edit .env: set JETSON_HOST to your Jetson's current IP

# 2. Make scripts executable
chmod +x scripts/*.sh

# 3. Set up the Jetson (sets hostname, installs zenoh bridge)
scripts/setup_jetson.sh
```

---

## Daily Workflow

**Terminal 1 — deploy & build:**
```bash
scripts/deploy_and_build.sh
```

**Terminal 2 — SSH into Jetson to run nodes:**
```bash
scripts/ssh.sh
source ~/robolubo_ws/install/setup.bash
ros2 launch robolubo <your_launch_file>.launch.py
```

**Terminal 3 — Zenoh bridge on PC (connects PC ROS2 to Jetson):**
```bash
scripts/zenoh_local.sh
```

**Terminal 4 — RViz on PC:**
```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## Project Structure

```
src/robolubo/       ROS2 package (mixed C++ / Python)
scripts/            Dev workflow scripts
zenoh/              Zenoh bridge configuration
```

See [CLAUDE.md](CLAUDE.md) for full developer reference.
