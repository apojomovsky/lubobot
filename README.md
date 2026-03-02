# robolubo

ROS2 Humble robot project for the robolubo platform.
Developed on a PC, deployed to a Jetson Nano (`jetson-lubo`).

---

## Prerequisites

**PC:**
- ROS2 Humble installed from debians
- `rsync`, `ssh`
- [`zenoh-bridge-ros2dds`](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases) (x86_64) in your PATH

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
