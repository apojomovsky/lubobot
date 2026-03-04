# robolubo – System Architecture

## Overview

2D SLAM + autonomous navigation on a iRobot Create 2 base, running on a Jetson Nano 4GB.
Sensor stack: monocular USB webcam (wide FoV) + MPU6050 IMU + Create 2 wheel odometry.

---

## Hardware

| Component | Role |
|---|---|
| iRobot Create 2 | Mobile base, wheel encoders |
| Jetson Nano 4GB | Main compute (ROS2 Humble) |
| USB webcam (wide FoV) | Visual loop closure for SLAM |
| MPU6050 | 6-DOF IMU (accelerometer + gyroscope) |
| STM32 Nucleo | IMU host MCU, streams data over USB serial (ST-Link VCP) |

**Why STM32 Nucleo over ESP32:** integrated ST-Link means no extra USB-serial adapter,
hardware I2C is robust, and the VCP shows up as a single `/dev/ttyACM*` device on the Jetson.

**Why not micro-ROS:** for a single IMU at ~100 Hz, a simple serial protocol + a ROS2
translator node is far simpler to develop, debug, and maintain than running a micro-ROS agent.

---

## ROS2 Node Graph

```
MPU6050
  └─► STM32 Nucleo (I2C)
        └─► USB serial (/dev/ttyACM*)
              └─► imu_node  ──────────────────► /imu/data_raw
                                                       │
                                          imu_filter_madgwick
                                                       │
                                                 /imu/data
                                                       │
Create 2 ──► create_driver ──► /odom                  │
                                    └──────────────────┤
                                          robot_localization (EKF)
                                                       │
                                          /odometry/filtered
                                                       │
USB webcam ──► usb_cam ──► /image_raw                  │
                    └─► /camera_info    ───────────────┤
                                                    rtabmap
                                                  /    │    \
                                               /map  /tf  /scan (2D projected)
                                                       │
                                                     Nav2
```

---

## Component Details

### 1. IMU pipeline

- **Nucleo firmware**: reads MPU6050 over I2C, streams packets over USB serial
  (simple ASCII or binary — TBD during firmware implementation)
- **`imu_node`** (Python, in this package): reads serial port, publishes `sensor_msgs/Imu`
  on `/imu/data_raw` (raw accel + gyro, no orientation)
- **`imu_filter_madgwick`**: fuses accel + gyro into orientation estimate, publishes
  `/imu/data`. Runs in no-magnetometer mode (MPU6050 has no mag).

### 2. Odometry fusion

- **`robot_localization` EKF**: fuses `/odom` (Create 2 wheel encoders) and `/imu/data`
  into `/odometry/filtered` — reliable local pose estimate that SLAM builds on top of.

### 3. Camera

- **`usb_cam`**: publishes `/image_raw` and `/camera_info`
- Camera must be **calibrated** before use (saves intrinsics + distortion coefficients to
  a YAML file). Use `ros2 run camera_calibration cameracalibrator` with a printed
  checkerboard. One-time procedure, ~5 minutes.
- Wide FoV introduces lens distortion — rtabmap undistorts automatically using the
  calibration file.

### 4. SLAM

- **`rtabmap`** in **2D mode**:
  - Input odometry: `/odometry/filtered` (EKF output)
  - Input camera: `/image_raw` + `/camera_info`
  - Camera role: appearance-based **loop closure only** (not primary pose source)
  - Output: 2D occupancy grid (`/map`), corrected pose via `/tf`
  - Wheel odom + IMU do the heavy lifting; camera corrects accumulated drift when
    revisiting known locations

### 5. Navigation

- **Nav2** for autonomous 2D navigation, consuming `/map` and `/odometry/filtered`.

---

## Sensor Limitations & Mitigations

| Limitation | Mitigation |
|---|---|
| Monocular camera has no absolute scale | Scale initialized from wheel odometry |
| MPU6050 has no magnetometer | Madgwick runs in IMU mode (no yaw absolute reference); wheel odom provides heading |
| Cheap webcam, unknown distortion | Camera calibration corrects distortion before SLAM processing |
| Wide FoV → more distortion | Same — calibration handles it; wide FoV actually helps feature tracking |

---

## Implementation Roadmap

1. **Nucleo firmware** — MPU6050 I2C read + USB serial output
2. **`imu_node`** — serial → `sensor_msgs/Imu` ROS2 node
3. **`usb_cam` integration** — bring up camera node, verify `/image_raw`
4. **Camera calibration** — checkerboard calibration, save YAML
5. **`imu_filter_madgwick` + `robot_localization`** — wire up odometry fusion
6. **rtabmap** — configure for 2D mode with monocular camera + EKF odom
7. **Nav2** — configure and tune for the robot's footprint and kinematics

---

## Future: LiDAR

A **Sick Tim 551** will be added later. When available, it will replace the camera as the
primary mapping sensor (much more reliable for 2D SLAM), with the camera kept for
appearance-based loop closure. The IMU + wheel odom fusion pipeline remains unchanged.
