# ORB-SLAM3 Live Monocular Mode

This repository now supports a minimal live monocular integration path built around the forked [`ros2_orb_slam3`](https://github.com/apojomovsky/ros2_orb_slam3) package in `external/ros2_orb_slam3`.

## Design

The split is intentional:

- the fork owns only generic ORB-SLAM3 wrapper behavior
- `robolubo` owns robot-specific launch composition

To keep the fork small, the C++ node was extended in only two ways:

- it can subscribe directly to a live `sensor_msgs/Image` topic without the Python dataset feeder
- it publishes a `geometry_msgs/PoseStamped` estimate on a ROS topic

Everything else stays in `lubobot`.

## Current Interface

Launch from `robolubo`:

```bash
ros2 launch robolubo orbslam_mono.launch.py
```

Useful launch arguments:

- `image_topic`
  Example: `/camera/image_raw`
- `settings_file`
  Full path to the ORB-SLAM3 monocular YAML
- `pose_topic`
  Default: `/orb_slam3/pose`
- `world_frame`
  Default: `map`
- `use_pangolin`
  Default: `true`
- `use_rviz`
  Default: `false`

Example:

```bash
ros2 launch robolubo orbslam_mono.launch.py \
  image_topic:=/usb_cam/image_raw \
  use_rviz:=true
```

The default settings file is now the repo-owned camera-specific config:

- [`src/robolubo/config/orbslam/general_webcam_640x480.yaml`](/home/alexis/lubobot/src/robolubo/config/orbslam/general_webcam_640x480.yaml)

## Expectations

This launch file assumes the camera driver is already running and publishing calibrated images.

For a real robot validation pass, the minimum expected inputs are:

- a live monocular image topic
- a stable camera frame and mount
- a calibration YAML that matches the physical webcam

The original EuRoC config is no longer the default. The current default is the first local webcam-specific calibration-based config, but real tracking quality still depends on validating that configuration against the actual camera stream.

## Current Output

The wrapper currently publishes:

- `geometry_msgs/PoseStamped` on the configured pose topic

The pose is published in the configured world frame, defaulting to `map`.

## Not Done Yet

This is intentionally still minimal. It does not yet provide:

- TF publication
- path publication
- camera driver launch
- camera calibration tooling
- wheel odom / IMU fusion

Those belong in later integration steps once live image tracking is validated.
