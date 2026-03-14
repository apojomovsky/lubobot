# IMU Bridge

`lubobot` now vendors the ROS 2 side of the MPU6050 project as a local package:

- [`imu_bridge`](/home/alexis/lubobot/src/imu_bridge)

This is the ROS-side subset imported from the external repository at:

- `/home/alexis/mpu6050-ros2`

The STM32 firmware remains external. Only the ROS package was brought into this workspace.

## What Was Imported

The local `imu_bridge` package contains:

- the serial bridge node that reads 16-byte MPU6050 packets over USB CDC
- a lean launch file that starts:
  - `imu_bridge_node`
  - `imu_filter_madgwick`

It intentionally does not carry over:

- the toy URDF visualizer
- the package-local RViz config

Those are not needed for robot bringup.

## Topics

The imported package publishes:

- `/imu/data_raw`

The launch file also starts `imu_filter_madgwick`, which publishes:

- `/imu/data`

## Launch

Robot-facing entrypoint:

```bash
ros2 launch robolubo imu.launch.py
```

Useful arguments:

- `port`
  Default: `/dev/ttyACM0`
- `frame_id`
  Default: `imu_link`
- `fixed_frame`
  Default: `base_link`
- `publish_tf`
  Default: `false`

Direct package launch:

```bash
ros2 launch imu_bridge imu_bridge.launch.py
```

## Notes

- The raw bridge currently assumes the existing 16-byte packet format from the STM32 firmware.
- Orientation is not estimated by the bridge itself. `imu_filter_madgwick` handles that separately.
- The current covariance values are placeholders and should be tuned after real sensor validation.
