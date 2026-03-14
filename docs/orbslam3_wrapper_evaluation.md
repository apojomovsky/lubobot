# ORB-SLAM3 ROS 2 Wrapper Evaluation

## Context

This document records the evaluation of the `Mechazo11/ros2_orb_slam3` wrapper as a candidate VSLAM base for `lubobot`.

Goal of the experiment:

- verify whether the wrapper is actually usable
- determine how much work is required to make it run
- determine whether it is suitable as the basis for live robot integration

Environment used during the experiment:

- local development machine: Ubuntu 24.04 x86_64
- ROS 2 distro on local machine: Kilted
- target robot environment: Jetson Nano, ROS 2 Humble

Important constraint:

- the upstream wrapper explicitly targets ROS 2 Humble
- local validation was done on Kilted because it is the current desktop environment

## Repository Evaluated

The evaluated code was cloned locally under:

- `/home/alexis/lubobot/ros2_orb_slam3`

The wrapper includes:

- a C++ ORB-SLAM3 node
- a Python dataset driver
- bundled ORB-SLAM3 source
- bundled DBoW2 and g2o source
- a small EuRoC sample dataset under `TEST_DATASET/sample_euroc_MH05`

## High-Level Result

The wrapper is usable as a base.

It was successfully built and run against the included EuRoC sample dataset after multiple fixes. The sample completed the Python/C++ handshake, loaded the ORB vocabulary and camera settings, and initialized a map successfully.

Evidence from runtime:

- handshake completed between Python and C++ nodes
- `ORBvoc.txt.bin` loaded successfully
- `EuRoC.yaml` loaded successfully
- ORB-SLAM3 initialized from scratch
- first keyframe and map points were created

Representative runtime output:

```text
Vocabulary loaded!
Initialization of Atlas from scratch
Creation of new map with id: 0
MonocularMode node initialized
First KF:0; Map init KF:0
New Map created with 144 points
```

## What Was Learned

### 1. The wrapper is not turnkey

The wrapper is a real starting point, but it is not production-ready and not even fully self-contained as cloned.

It is best understood as:

- a working demo wrapper around ORB-SLAM3
- a sample dataset player
- a base for further engineering

It is not yet:

- a clean ROS robot component
- a ready-to-use live camera node
- a Nav2-ready mapping/localization component

### 2. The upstream code has hardcoded assumptions

The wrapper assumed:

- a workspace named `~/ros2_test`
- local source-tree-relative paths for vocabulary, config, and dataset

Those assumptions broke immediately in our environment.

### 3. The wrapper ships stale third-party binaries

The repo linked against prebuilt shared libraries for:

- `DBoW2`
- `g2o`

The shipped `libDBoW2.so` depended on:

```text
libopencv_core.so.4.5d
```

That is not compatible with the system OpenCV on Ubuntu 24.04 / ROS 2 Kilted, and is a general portability problem even outside Kilted.

This is the most important structural issue found during the evaluation.

### 4. The sample pipeline works once patched

After fixing the path assumptions, compatibility issues, and third-party linking model, the sample dataset pipeline worked.

That means:

- ORB-SLAM3 core is functioning
- wrapper architecture is viable enough for experimentation
- the repo is worth considering as a starting base

## Problems Encountered and Resolutions

### Problem A: Pangolin not found by CMake

Build error:

```text
Could not find a package configuration file provided by "Pangolin"
```

Cause:

- Pangolin was not installed in a CMake-discoverable way

Resolution:

- install Pangolin from source
- ensure `PangolinConfig.cmake` exists under `/usr/local/lib/cmake/Pangolin`

Observed install path:

- `/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake`

### Problem B: `cv_bridge` header path incompatible with Kilted

Build error:

```text
fatal error: cv_bridge/cv_bridge.h: No such file or directory
```

Cause:

- Kilted installs the C++ header as `cv_bridge/cv_bridge.hpp`
- wrapper used old include path `cv_bridge/cv_bridge.h`

Resolution:

- update include to `cv_bridge/cv_bridge.hpp`

### Problem C: `image_transport` header path incompatible with Kilted

Build error:

```text
fatal error: image_transport/image_transport.h: No such file or directory
```

Cause:

- Kilted installs `image_transport/image_transport.hpp`
- wrapper used old include path `image_transport/image_transport.h`

Resolution:

- update include to `image_transport/image_transport.hpp`

### Problem D: Broken install-space assumptions from building in the package directory

Symptoms:

- `ros2 run` could not find the package
- `source install/setup.bash` referenced missing package-local setup files

Cause:

- package was first built from its own repo root rather than from a proper colcon workspace
- an `AMENT_IGNORE` file ended up present and blocked package discovery

Resolution:

- create a proper workspace:
  - `/home/alexis/orbslam_ws`
- symlink the repo into:
  - `/home/alexis/orbslam_ws/src/ros2_orb_slam3`
- remove `AMENT_IGNORE`
- rebuild from the workspace root

### Problem E: Pangolin runtime library not found

Runtime error:

```text
error while loading shared libraries: libpango_display.so.0: cannot open shared object file
```

Cause:

- Pangolin libraries were installed in `/usr/local/lib`
- runtime linker path did not include `/usr/local/lib`

Observed library path:

- `/usr/local/lib/libpango_display.so.0`

Temporary resolution:

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

Code-side permanent mitigation added during evaluation:

- RPATH was added for `/usr/local/lib` in the wrapper CMake

### Problem F: stale prebuilt `DBoW2` binary linked against old debug OpenCV

Runtime error:

```text
error while loading shared libraries: libopencv_core.so.4.5d
```

Cause:

- upstream CMake linked directly against shipped third-party `.so` files
- `libDBoW2.so` had an embedded dependency on `libopencv_core.so.4.5d`

Confirmed via:

```text
readelf -d .../libDBoW2.so | grep NEEDED
```

Resolution:

- stop linking the shipped `.so` files
- build bundled `DBoW2` and `g2o` from source inside the workspace
- link `orb_slam3_lib` against the locally-built `DBoW2` and `g2o` targets instead

This was the critical fix that made the sample runnable.

## Code Changes Made During Evaluation

The following changes were made to the local wrapper checkout under `/home/alexis/lubobot/ros2_orb_slam3`.

### Path handling fixes

Files:

- `/home/alexis/lubobot/ros2_orb_slam3/scripts/mono_driver_node.py`
- `/home/alexis/lubobot/ros2_orb_slam3/src/common.cpp`
- `/home/alexis/lubobot/ros2_orb_slam3/include/ros2_orb_slam3/common.hpp`

Changes:

- replaced hardcoded `~/ros2_test/...` path logic
- resolved package data via ROS package share directory
- used `ament_index_cpp` and `ament_index_python`

Effect:

- wrapper no longer depends on a specific workspace name
- dataset, vocabulary, and config resolve from installed package data

### Packaging fixes

Files:

- `/home/alexis/lubobot/ros2_orb_slam3/CMakeLists.txt`
- `/home/alexis/lubobot/ros2_orb_slam3/package.xml`

Changes:

- added `ament_index_cpp`
- installed:
  - `TEST_DATASET`
  - `orb_slam3/Vocabulary`
  - `orb_slam3/config`

Effect:

- runtime data required by the wrapper is available from install space

### ROS 2 Kilted compatibility fixes

File:

- `/home/alexis/lubobot/ros2_orb_slam3/include/ros2_orb_slam3/common.hpp`

Changes:

- changed:
  - `cv_bridge/cv_bridge.h` -> `cv_bridge/cv_bridge.hpp`
  - `image_transport/image_transport.h` -> `image_transport/image_transport.hpp`

Effect:

- wrapper builds on Kilted with currently installed headers

### Runtime library path mitigation

File:

- `/home/alexis/lubobot/ros2_orb_slam3/CMakeLists.txt`

Changes:

- added build/install RPATH for `/usr/local/lib`

Effect:

- intended to reduce runtime linker issues for Pangolin installs under `/usr/local/lib`

### Third-party build model fix

File:

- `/home/alexis/lubobot/ros2_orb_slam3/CMakeLists.txt`

Changes:

- added:
  - `add_subdirectory(orb_slam3/Thirdparty/DBoW2)`
  - `add_subdirectory(orb_slam3/Thirdparty/g2o)`
- linked against built targets:
  - `DBoW2`
  - `g2o`
- removed direct linkage to shipped `.so` files
- installed built `DBoW2` and `g2o` targets

Effect:

- local build now matches local OpenCV and compiler toolchain
- removed dependence on stale prebuilt third-party binaries

## Commands and Workflow That Worked

### Workspace setup

```bash
mkdir -p /home/alexis/orbslam_ws/src
ln -s /home/alexis/lubobot/ros2_orb_slam3 /home/alexis/orbslam_ws/src/ros2_orb_slam3
```

### Build

```bash
cd /home/alexis/orbslam_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select ros2_orb_slam3
```

### Run sample

Terminal 1:

```bash
cd /home/alexis/orbslam_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

Terminal 2:

```bash
cd /home/alexis/orbslam_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

### Successful Python-side output

```text
self.image_sequence_dir: /home/alexis/orbslam_ws/install/ros2_orb_slam3/share/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05
Configuration to be sent: EuRoC
MonoDriver initialized, attempting handshake with CPP node
Got ack: ACK
Handshake complete
```

### Successful C++-side output

```text
Waiting to finish handshake ......
Path to settings file: /home/alexis/orbslam_ws/install/ros2_orb_slam3/share/ros2_orb_slam3/orb_slam3/config/Monocular/EuRoC.yaml
Vocabulary loaded!
Initialization of Atlas from scratch
MonocularMode node initialized
New Map created with 144 points
```

## Current Assessment of Suitability for `lubobot`

### What this wrapper is good for

- validating ORB-SLAM3 itself
- acting as a starting point for a monocular SLAM integration
- providing a codebase we can extend rather than starting from zero

### What this wrapper is not yet good for

- direct live robot deployment
- clean webcam subscription from a robot camera topic
- publishing a ready-to-use pose/TF interface for ROS navigation workflows
- direct Nav2 integration

### Main architectural limitation right now

The current wrapper is still centered around:

- a dataset playback Python node
- internal handshake topics
- a C++ node that calls ORB-SLAM3

It is not yet centered around:

- a live `sensor_msgs/Image` pipeline from a robot camera
- `sensor_msgs/CameraInfo`
- pose publication
- TF publication
- RViz-friendly outputs

That means the sample success is necessary but not sufficient for robot integration.

## Specific Weaknesses Observed

### 1. Logging bug in the C++ node

The C++ node printed:

```text
Configuration YAML file name:
```

with an empty value because it logs `receivedConfig` instead of `experimentConfig`.

This is harmless, but indicates the wrapper has rough edges.

### 2. GUI thread warning

Observed warning:

```text
QSocketNotifier: Can only be used with threads started with QThread
```

This did not block execution in the sample run, but should be noted for future GUI stability questions.

### 3. Deprecated CMake usage on Kilted

Kilted emitted deprecation warnings for `ament_target_dependencies()`.

This does not block current use, but the build system is older-style and may need modernization eventually.

## Implications for the Robot Project

The evaluation supports the following conclusion:

- ORB-SLAM3 remains a valid technical direction for the robot
- this wrapper is a workable basis
- but it must be treated as an experimental integration branch, not a drop-in package

For `lubobot`, the next engineering work is not “install and run”.

It is:

1. convert the wrapper from dataset mode to live camera mode
2. define clean robot-facing ROS interfaces
3. publish pose and likely TF
4. test with the real webcam
5. only after that, consider IMU coupling or higher-level navigation integration

## Recommended Next Steps

### Immediate next step

Audit the wrapper’s current ROS I/O and modify it into a live-camera ROS node.

That likely means:

- remove or bypass the dataset Python feeder
- subscribe directly to camera images in the C++ node, or replace the Python feeder with a live topic adapter
- add pose publication
- add optional TF publication

### Short-term integration plan

1. Keep this wrapper in a separate workspace during continued experimentation.
2. Create a live monocular mode using the robot webcam.
3. Validate live tracking on desktop first.
4. Port the resulting code path to Jetson Humble.
5. Only after monocular SLAM works live, evaluate whether IMU integration is worth the added complexity.

### Caution for final system goals

Even if ORB-SLAM3 works well for localization and loop closure, it does not automatically give:

- a robust 2D obstacle map
- a Nav2-ready occupancy grid
- reliable point-and-click navigation from RViz

So this wrapper should currently be considered a path toward:

- visual localization
- loop closure
- sparse map / trajectory estimation

not yet a full navigation-stack solution.

## Bottom Line

This experiment was successful.

`Mechazo11/ros2_orb_slam3` is not clean out of the box, but after fixing:

- path assumptions
- Kilted header incompatibilities
- runtime linker issues
- stale prebuilt third-party binary usage

it did run the provided sample dataset correctly.

That makes it a credible candidate for further experimentation, but not yet a package ready to integrate directly into the main `lubobot` robot workflow.
