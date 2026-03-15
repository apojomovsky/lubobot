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
ros2_ws_kilted/
```

It is kept separate from the existing Docker-oriented:

```bash
ros2_ws/
```

That avoids mixing Humble container artifacts with Kilted host artifacts.

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
  - repo root as `ros2_ws_kilted/src/lubobot`
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

So the existing manual diagnostics script can be reused without mixing logs/install paths with the Docker workspace.

## Notes

- This workflow is intentionally limited to camera/ORB-SLAM work first.
- If it proves stable, it can later be extended for IMU or other host-native experiments.
- Keep camera experiments here and robot-stack experiments in Docker until there is a reason to merge the workflows.
