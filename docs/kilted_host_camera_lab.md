# Kilted Host Camera Lab

This workflow is for camera and ORB-SLAM experiments on the host machine without Docker.

Scope:

- host-native ROS 2 Kilted binaries
- camera-only bringup
- Pangolin / ORB-SLAM manual diagnostics
- no robot base, no IMU, no wheel odometry

This is intentionally separate from the Docker/Humble workflow.

## Why

Use this when:

- Docker lifecycle issues are wasting time
- you only need camera + ORB-SLAM experiments
- you want a reproducible host-native path that can be recreated on another machine

This does not build ROS from source.

It uses:

- binary ROS 2 Kilted packages from apt
- source builds only for workspace packages not provided as binaries

## Workspace Layout

The host-native workspace lives under:

```bash
../ros2_ws_kilted/
```

It is kept separate from the existing Docker-oriented:

```bash
ros2_ws/
```

That avoids mixing Humble container artifacts with Kilted host artifacts.
It also avoids recursive symlink problems from placing a workspace inside the repo itself.

## One-Time Setup

Prerequisites:

- ROS 2 Kilted already installed under `/opt/ros/kilted`
- `external/ros2_orb_slam3` cloned

Run:

```bash
just kilted-setup
```

This script:

- installs required Kilted binary packages
- ensures `rosdep` is initialized
- creates `ros2_ws_kilted/`
- symlinks:
  - `src/robolubo` as `ros2_ws_kilted/src/robolubo`
  - `external/ros2_orb_slam3` as `ros2_ws_kilted/src/ros2_orb_slam3`

## Build

Source the host-native environment:

```bash
source scripts/env_kilted_host.sh
```

Then build:

```bash
just kilted-build
```

That builds only:

- `ros2_orb_slam3`
- `robolubo`

## Run Camera Diagnostics

Use the host-native wrapper:

```bash
source scripts/env_kilted_host.sh
scripts/run_orbslam_manual_diagnostics_host.sh 60
```

The wrapper sets:

- `ROS_WS_ROOT=ros2_ws_kilted`
- `/opt/ros/kilted`
- `ros2_ws_kilted/install/setup.bash` if present
- `USE_PANGOLIN=false` by default on host

So the existing manual diagnostics script can be reused without mixing logs/install paths with the Docker workspace.

If you explicitly want Pangolin despite the host instability risk:

```bash
USE_PANGOLIN=true scripts/run_orbslam_manual_diagnostics_host.sh 60
```

## Recovering From Stale ORB-SLAM Processes

If a previous host-native run leaves `/orbslam_mono` or camera nodes behind, use:

```bash
just kilted-orbslam-clean
```

This helper:

- kills stale `mono_node_cpp`, `v4l2_camera_node`, and camera TF publishers
- restarts the ROS 2 daemon
- prints remaining matching processes and current ROS nodes

## Calibrating The Logitech C920

Before serious ORB-SLAM testing with the C920, calibrate it in the exact intended mode.

Current intended mode:

- `640x480`
- `YUYV`
- fixed focus

The repo includes a dedicated C920 calibration target file:

- [`logitech_c920_640x480.yaml`](/home/alexis/lubobot/src/robolubo/config/camera/logitech_c920_640x480.yaml)

Run the host-native calibration helper:

```bash
source scripts/env_kilted_host.sh
CHESSBOARD_SIZE=8x6 SQUARE_SIZE=0.025 just kilted-c920-calibrate
```

This launches the C920 and opens `camera_calibration`.

What you need:

- a printed chessboard
- default board size currently assumed by the helper:
  - `8x6`
- default square size:
  - `0.024` meters

For the checkerboard currently in use in this repo workflow:

- `CHESSBOARD_SIZE=8x6`
- `SQUARE_SIZE=0.025`

Override if needed, for example:

```bash
CHESSBOARD_SIZE=7x6 SQUARE_SIZE=0.025 just kilted-c920-calibrate
```

After calibration:

- save the YAML over the dedicated C920 file in the repo
- then point camera and ORB-SLAM runs at the C920 profile instead of the generic webcam profile

The current calibrated C920 profile lives at:

- [`logitech_c920_640x480.yaml`](/home/alexis/lubobot/src/robolubo/config/camera/logitech_c920_640x480.yaml)

The matching ORB-SLAM settings file for the calibrated C920 currently lives at:

- [`logitech_c920_640x480_aggressive.yaml`](/home/alexis/lubobot/src/robolubo/config/orbslam/logitech_c920_640x480_aggressive.yaml)

## Notes

- This workflow is intentionally limited to camera/ORB-SLAM work first.
- If it proves stable, it can later be extended for IMU or other host-native experiments.
- Keep camera experiments here and robot-stack experiments in Docker until there is a reason to merge the workflows.
