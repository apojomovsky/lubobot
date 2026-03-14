# Frame Ownership

This document captures the intended separation of responsibilities between the
Create 2 base driver, `robot_localization`, and ORB-SLAM3.

## Current State

The current live stack behaves as follows:

- `create_driver`
  - publishes wheel odometry on `/odom`
  - publishes `odom -> base_footprint` TF
- `imu_bridge` + `imu_filter_madgwick`
  - publish IMU data on `/imu/data_raw` and `/imu/data`
- `robot_localization`
  - publishes fused local odometry on `/odometry/filtered`
  - currently does **not** publish TF
- `ros2_orb_slam3`
  - publishes camera SLAM pose on `/orb_slam3/pose`
  - uses `map` as its world frame

## Intended Responsibility Split

### Local Motion

Use the EKF output as the robot's local motion estimate:

- topic: `/odometry/filtered`
- frame pair: `odom -> base_footprint`

This should eventually replace raw wheel odometry as the main local pose source
for downstream consumers.

### Global / Drift-Corrected Pose

Use ORB-SLAM3 as the visual global pose source:

- topic: `/orb_slam3/pose`
- frame: `map`

At this stage it is a pose estimate and tracking system, not yet a complete TF
authority in the robot frame tree.

## TF Ownership Rule

Only one component should publish `odom -> base_footprint`.

That means:

- while `create_driver` publishes this TF, keep
  `robot_localization publish_tf:=false`
- if we later want EKF to own `odom -> base_footprint`, then disable TF
  publication in `create_driver` first

Do not let both publish the same transform.

## Planned End State

Likely target:

- EKF owns `odom -> base_footprint`
- ORB-SLAM3 contributes `map`-level pose information
- robot consumers use:
  - `odometry/filtered` for local motion
  - SLAM for higher-level global correction / relocalization

## Not Done Yet

Still pending before changing TF ownership:

- compare raw `/odom` vs `/odometry/filtered` during real driving
- verify IMU frame alignment under motion
- decide how `map`, `odom`, and camera-related frames should connect
- decide whether ORB-SLAM remains topic-only or also becomes a TF publisher
