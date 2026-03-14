# Humble Docker Environment

This repository now includes a local ROS 2 Humble development container so the ORB-SLAM work can be exercised on the desktop in a Humble userspace before moving to the Jetson.

## Purpose

Use this container when:

- the host machine is not running Humble
- you want to validate Humble-specific package behavior locally
- you need a reproducible userspace for ROS 2 + Pangolin experiments

The container is intended for desktop development and validation, not deployment to the Jetson.

## What It Includes

The image defined in [`docker/humble/Dockerfile`](/home/alexis/lubobot/docker/humble/Dockerfile) provides:

- `ros:humble-desktop` base image
- ROS 2 Humble development tools
- `cv_bridge`, `image_transport`, `usb_cam`, `rviz2`, `image_proc`, `camera_calibration`
- OpenCV, Eigen, Boost, OpenSSL development packages
- Pangolin built from source in a dedicated builder stage and copied into the final dev image

The primary workflow uses:

- [`docker/compose.humble.yml`](/home/alexis/lubobot/docker/compose.humble.yml)
- [`scripts/setup_external.sh`](/home/alexis/lubobot/scripts/setup_external.sh)
- [`scripts/humble_up.sh`](/home/alexis/lubobot/scripts/humble_up.sh)
- [`scripts/humble_shell.sh`](/home/alexis/lubobot/scripts/humble_shell.sh)
- [`scripts/humble_build.sh`](/home/alexis/lubobot/scripts/humble_build.sh)
- [`scripts/humble_down.sh`](/home/alexis/lubobot/scripts/humble_down.sh)

The compose service mounts:

- this repo at `/workspaces/ros2_ws/src/lubobot`
- your fork clone at `/workspaces/ros2_ws/src/ros2_orb_slam3`
- persistent workspace caches under `/workspaces/ros2_ws/build|install|log`

## External Dependency Setup

Clone your fork into `external/ros2_orb_slam3`, or let the helper script do it.

If the clone already exists:

```bash
scripts/setup_external.sh
```

If it does not exist yet:

```bash
scripts/setup_external.sh <your-fork-url> humble-lubobot
```

## Build and Run

From the repo root:

```bash
chmod +x scripts/setup_external.sh scripts/humble_up.sh scripts/humble_shell.sh scripts/humble_build.sh scripts/humble_down.sh
scripts/humble_up.sh
```

The first run builds the image:

- image name: `lubobot:humble-dev`

Subsequent runs reuse the cached image.
The Dockerfile uses a multi-stage build so Pangolin compilation is cached separately from the final development image.

## Running Commands in the Container

Open a shell:

```bash
scripts/humble_shell.sh
```

Run a one-shot command:

```bash
docker compose -f docker/compose.humble.yml exec dev bash -lc 'source /opt/ros/humble/setup.bash && ros2 doctor'
```

## GUI Notes

The launcher uses:

- host networking
- `/tmp/.X11-unix`
- `DISPLAY`
- optional `XAUTHORITY`
- `/dev/dri` if available

This is meant to support:

- Pangolin viewer windows
- RViz
- OpenCV windows if needed

If GUI access fails on the host, allow local Docker access once:

```bash
xhost +local:docker
```

## Typical Workflow for ORB-SLAM Experiments

Bring the container up:

```bash
scripts/humble_up.sh
```

Build inside the mounted workspace:

```bash
scripts/humble_build.sh
```

Open an interactive shell:

```bash
scripts/humble_shell.sh
```

Inside the container, the workspace is already assembled at:

```text
/workspaces/ros2_ws
```

The default build helper uses:

```bash
colcon build --symlink-install --parallel-workers 1
```

The ORB-SLAM wrapper is heavy enough that a conservative first build is preferable to avoid host instability. Override with:

```bash
HUMBLE_BUILD_WORKERS=2 scripts/humble_build.sh
```

## Current Limits

- This is an x86_64 desktop Humble environment, not a Jetson simulation.
- A successful container build does not guarantee Jetson arm64 success.
- USB camera and device passthrough are not wired in by default.
- The compose service is optimized for GUI validation, not privileged hardware access.

## Why This Exists

The ORB-SLAM wrapper evaluation showed that:

- upstream targets Humble
- the host machine currently runs Kilted
- Kilted-specific compatibility work can obscure what is truly required for Humble

Using this container reduces that ambiguity and gives a cleaner path to:

1. validate the wrapper on Humble locally
2. harden it for Humble userspace
3. only then port to Jetson arm64
