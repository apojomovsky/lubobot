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
- OpenCV, Eigen, Boost, OpenSSL development packages
- Pangolin built from source in a dedicated builder stage and copied into the final dev image
- GPU/runtime support for AMD and NVIDIA hosts

Workspace-specific ROS package dependencies are not hardcoded into the image.
They are resolved from the actual package manifests in two places:

1. a cacheable Docker build layer fed by a manifest-only snapshot
2. the runtime `rosdep` pass in [`scripts/ros_build.sh`](/home/alexis/lubobot/scripts/ros_build.sh)

The helper [`scripts/sync_rosdep_manifests.sh`](/home/alexis/lubobot/scripts/sync_rosdep_manifests.sh) copies only `package.xml` files from:

- `src/`
- `external/ros2_orb_slam3/`

into `docker/rosdep-manifests/`, which the Dockerfile uses for a dedicated `rosdep` layer.

The primary workflow uses:

- [`docker/compose.humble.yml`](/home/alexis/lubobot/docker/compose.humble.yml)
- [`docker/compose.humble.amd.yml`](/home/alexis/lubobot/docker/compose.humble.amd.yml)
- [`docker/compose.humble.nvidia.yml`](/home/alexis/lubobot/docker/compose.humble.nvidia.yml)
- [`docker/compose.humble.devices.yml`](/home/alexis/lubobot/docker/compose.humble.devices.yml)
- [`scripts/ros_common.sh`](/home/alexis/lubobot/scripts/ros_common.sh)
- [`scripts/sync_rosdep_manifests.sh`](/home/alexis/lubobot/scripts/sync_rosdep_manifests.sh)
- [`scripts/setup_external.sh`](/home/alexis/lubobot/scripts/setup_external.sh)
- [`scripts/ros_up.sh`](/home/alexis/lubobot/scripts/ros_up.sh)
- [`scripts/ros_shell.sh`](/home/alexis/lubobot/scripts/ros_shell.sh)
- [`scripts/ros_attach.sh`](/home/alexis/lubobot/scripts/ros_attach.sh)
- [`scripts/ros_build.sh`](/home/alexis/lubobot/scripts/ros_build.sh)
- [`scripts/ros_down.sh`](/home/alexis/lubobot/scripts/ros_down.sh)
- [`justfile`](/home/alexis/lubobot/justfile)

The compose service mounts:

- this repo at `/workspaces/ros2_ws/src/lubobot`
- your fork clone at `/workspaces/ros2_ws/src/ros2_orb_slam3`
- persistent workspace caches under `/workspaces/ros2_ws/build|install|log`

The container user is built to match the host user by UID/GID:

- `HOST_UID`
- `HOST_GID`
- `HOST_USERNAME`

The helper scripts populate those automatically from the current host user so files created in mounted directories stay owned by the same user outside the container.

That mapped user also has passwordless `sudo` inside the dev container. Normal project dependencies should still be handled through the Dockerfile and `rosdep`, but ad-hoc package installs and debugging do not require dropping back to `docker exec -u root`.

GPU handling is selected automatically:

- NVIDIA if `nvidia-smi` is available
- AMD if `/dev/kfd` and `/dev/dri` are present
- CPU fallback otherwise

Override detection if needed:

```bash
HUMBLE_GPU=nvidia scripts/ros_up.sh
HUMBLE_GPU=amd scripts/ros_up.sh
HUMBLE_GPU=cpu scripts/ros_up.sh
```

Override the container identity if needed:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) HOST_USERNAME=$(id -un) scripts/ros_up.sh
```

## External Dependency Setup

Clone your fork into `external/ros2_orb_slam3`, or let the helper script do it.

If the clone already exists:

```bash
just setup-external
```

If it does not exist yet:

```bash
just setup-external <your-fork-url> humble-lubobot
```

## Build and Run

From the repo root:

```bash
just ros-up
```

The first run builds the image:

- image name: `lubobot:humble-dev`

Subsequent runs reuse the cached image.
The Dockerfile uses a multi-stage build so Pangolin compilation is cached separately from the final development image.
It also uses a manifest-only `rosdep` stage, so ROS apt dependencies are cached based on `package.xml` contents instead of the full source tree.

## GPU Runtime Notes

### AMD / Radeon

The AMD override enables:

- `/dev/dri`
- `/dev/kfd`
- `seccomp=unconfined`
- dynamic `group_add` for render/video/kfd
- `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.json`
- `XDG_RUNTIME_DIR=/tmp/runtime-root`
- `QT_QPA_PLATFORM=xcb`

This is based on the known-working approach from `/home/alexis/o3de-playground`.

### NVIDIA

The NVIDIA override enables:

- `runtime: nvidia`
- `gpus: all`
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=all`

This expects NVIDIA Container Toolkit to be installed on the host.

## Running Commands in the Container

Open a shell:

```bash
scripts/ros_shell.sh
```

This opens a one-off container with `docker compose run --rm`, so exiting the shell removes that shell container automatically.

Attach to the shared `dev` container started by `scripts/ros_up.sh`:

```bash
scripts/ros_attach.sh
```

Exiting an attached shell does not stop the shared container. Use `scripts/ros_down.sh` when you want that container removed.

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
- GPU-specific runtime settings selected by host detection

This is meant to support:

- Pangolin viewer windows
- RViz
- OpenCV windows if needed
- `vulkaninfo` and Mesa Vulkan tooling for debugging

When local hardware devices are present on the host, the helper scripts automatically add a device override that exposes:

- `/dev`
- `/run/udev`

This supports:

- `/dev/ttyACM*` and `/dev/ttyUSB*`
- stable symlinks under `/dev/serial/by-id`
- `/dev/video*`
- stable symlinks under `/dev/v4l/by-id`

If GUI access fails on the host, allow local Docker access once:

```bash
xhost +local:docker
```

## Typical Workflow for ORB-SLAM Experiments

Bring the container up:

```bash
just ros-up
```

Open a disposable shell:

```bash
just ros-shell
```

Attach to the shared container:

```bash
just ros-attach
```

Build inside the mounted workspace:

```bash
just ros-build
```

That helper performs:

1. `apt-get update`
2. `rosdep install -r --from-paths src --ignore-src ...`
3. `colcon build`

So generic system/GPU dependencies live in the image, while ROS workspace dependencies come from the project manifests.

Open an interactive shell:

```bash
just ros-shell
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
HUMBLE_BUILD_WORKERS=2 scripts/ros_build.sh
```

Recommended operator model:

- `just ros-shell`
  Use for one-off commands or short exploration. The container is removed on `exit`.
- `just ros-up`
  Start the shared long-lived `dev` container.
- `just ros-attach`
  Open another shell into that shared container.
- `just ros-down`
  Stop and remove the shared container.

## Current Limits

- This is an x86_64 desktop Humble environment, not a Jetson simulation.
- A successful container build does not guarantee Jetson arm64 success.
- USB camera and device passthrough are not wired in by default.
- The compose service is optimized for GUI validation, not privileged hardware access.
- NVIDIA support depends on host-side NVIDIA Container Toolkit configuration.

## Why This Exists

The ORB-SLAM wrapper evaluation showed that:

- upstream targets Humble
- the host machine currently runs Kilted
- Kilted-specific compatibility work can obscure what is truly required for Humble

Using this container reduces that ambiguity and gives a cleaner path to:

1. validate the wrapper on Humble locally
2. harden it for Humble userspace
3. only then port to Jetson arm64
