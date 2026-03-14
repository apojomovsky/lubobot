# Camera Workflow

This document covers the current webcam workflow for `lubobot`.

## Goal

Bring up a generic USB webcam on the desktop first, then reuse the same launch path later on the robot.

This stage is only about:

- image transport
- device selection
- calibration preparation
- making ORB-SLAM consume a real camera topic later

It is not yet about robot mounting or final TF integration.

## Current Launch

Camera driver entrypoint:

```bash
ros2 launch robolubo camera.launch.py
```

Useful arguments:

- `namespace`
  Default: `camera`
- `video_device`
  Default: `/dev/video0`
- `frame_id`
  Reserved for later TF integration
- `camera_name`
  Reserved for later calibration naming
- `camera_info_url`
  Default: empty
- `image_width`
  Default: `640`
- `image_height`
  Default: `480`
- `framerate`
  Default: `30`
- `pixel_format`
  Default: `YUYV`
- `output_encoding`
  Default: `rgb8`

Example:

```bash
ros2 launch robolubo camera.launch.py \
  video_device:=/dev/v4l/by-id/<stable-camera-path> \
  namespace:=camera \
  image_width:=640 \
  image_height:=480 \
  pixel_format:=YUYV \
  output_encoding:=rgb8
```

Expected topics are then under:

- `/camera/image_raw`
- `/camera/camera_info`

## What To Do When The Webcam Is Plugged In

### 1. Identify the device

On the host:

```bash
v4l2-ctl --list-devices
```

Then inspect the webcam modes:

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

The goal is to determine:

- actual `/dev/videoX`
- supported resolutions
- supported frame rates
- whether the driver should start from a conservative YUYV mode first

### 2. Start the camera driver

Inside the Humble container:

```bash
ros2 launch robolubo camera.launch.py \
  video_device:=/dev/v4l/by-id/<stable-camera-path> \
  image_width:=640 \
  image_height:=480 \
  pixel_format:=YUYV \
  output_encoding:=rgb8
```

Then verify:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

### 3. Calibrate intrinsics

This can and should be done before the camera is mounted on the robot.

Intrinsic calibration does not depend on robot mounting. It depends on:

- the webcam lens
- the active resolution
- the camera mode being used

The result should be a calibration YAML later referenced via:

- `camera_info_url:=file:///...`

The repository now includes the first calibration result at:

- [`src/robolubo/config/camera/general_webcam_640x480.yaml`](/home/alexis/lubobot/src/robolubo/config/camera/general_webcam_640x480.yaml)

The camera launch defaults to that file.

### 4. Extrinsics come later

Camera extrinsics cannot be finalized yet because the camera is not mounted on the robot.

Those will be defined later through:

- the physical mount
- `base_link -> camera_link` static transform

## Current Limits

- `v4l2_camera` is now the preferred driver path for this repo
- the current launch resolves stable `/dev/v4l/by-id/...` paths to the underlying `/dev/videoN`
- no calibration YAML exists yet
- no combined camera + ORB-SLAM launch exists yet
- no rectification pipeline is configured yet

## Next Expected Step

Once the webcam is physically available on the dev PC:

1. inspect `/dev/video*`
2. inspect supported modes with `v4l2-ctl`
3. launch `robolubo camera.launch.py`
4. verify `/camera/image_raw`
5. run intrinsic calibration
6. feed the calibrated topic into `orbslam_mono.launch.py`
