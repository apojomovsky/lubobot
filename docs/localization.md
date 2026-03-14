# Localization Scaffold

This document describes the current `robot_localization` integration scaffold in `lubobot`.

## Goal

Fuse:

- wheel odometry from the Create 2 stack
- filtered IMU orientation/angular velocity from `imu_filter_madgwick`

into a single local odometry estimate.

The intended role of this layer is:

- provide a better local `odom` estimate than wheel odometry alone
- keep local motion estimation available even if camera tracking drops
- serve as the local-motion backbone under later SLAM integration

## Current Files

- [`src/robolubo/config/localization/ekf.yaml`](/home/alexis/lubobot/src/robolubo/config/localization/ekf.yaml)
- [`src/robolubo/launch/localization.launch.py`](/home/alexis/lubobot/src/robolubo/launch/localization.launch.py)

## Current Assumptions

The scaffold currently assumes:

- wheel odometry is available on `/odom`
- filtered IMU data is available on `/imu/data`
- the robot is planar, so `two_d_mode` is enabled
- the IMU is mounted in REP-style orientation, with a provisional static offset
  of `x = -0.07 m`, `y = 0.0 m`, `z = 0.0 m` relative to `base_footprint`

The EKF is intentionally conservative:

- `publish_tf` defaults to `false`
- the config only fuses a narrow subset of wheel/IMU state variables

This is deliberate because the current Create 2 config already publishes:

- `odom -> base_footprint` TF

Blindly enabling EKF TF publication without first checking the live frame tree would create conflicts.

## Launch

```bash
ros2 launch robolubo localization.launch.py
```

Useful arguments:

- `publish_tf`
  Default: `false`
- `odom_frame`
  Default: `odom`
- `base_link_frame`
  Default: `base_footprint`
- `world_frame`
  Default: `odom`

## What Still Needs Validation

Before this should be treated as authoritative, we still need to confirm:

- the actual Create 2 odom topic and frame IDs on the live robot
- whether the IMU frame and robot frame conventions line up
- whether the IMU yaw axis sign is correct
- whether the Create 2 stack already publishes TF in a way that conflicts with EKF TF

## Expected Next Steps

1. Run the live robot with wheel odometry and IMU available.
2. Inspect:
   - `/odom`
   - `/imu/data`
   - the TF tree
3. Adjust:
   - `base_link_frame`
   - `odom0_config`
   - `imu0_config`
   - `publish_tf`
   - IMU mount translation once measured more precisely
4. Only then decide whether the EKF should replace the current odom TF source.
