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
  Default: `camera_link`
- `camera_name`
  Default: `usb_camera`
- `camera_info_url`
  Default: empty
- `image_width`
  Default: `640`
- `image_height`
  Default: `480`
- `framerate`
  Default: `30.0`
- `pixel_format`
  Default: `mjpeg2rgb`
- `av_device_format`
  Default: `MJPG`
- `io_method`
  Default: `mmap`

Example:

```bash
ros2 launch robolubo camera.launch.py \
  video_device:=/dev/video0 \
  namespace:=camera \
  frame_id:=camera_link
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
- whether MJPEG works reliably

### 2. Start the camera driver

Inside the Humble container:

```bash
ros2 launch robolubo camera.launch.py video_device:=/dev/video0
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

### 4. Extrinsics come later

Camera extrinsics cannot be finalized yet because the camera is not mounted on the robot.

Those will be defined later through:

- the physical mount
- `base_link -> camera_link` static transform

## Current Limits

- `usb_cam` is not yet validated in the current Humble image build
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
