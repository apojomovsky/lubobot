# Xbox 360 Teleop

`robolubo` now includes joystick teleoperation via `joy` and `teleop_twist_joy`.

## Launch

```bash
ros2 launch robolubo teleop_joy.launch.py
```

Useful arguments:

- `cmd_vel_topic`
  Default: `/cmd_vel`
- `joy_dev`
  Default: `/dev/input/js0`
- `deadzone`
  Default: `0.12`
- `autorepeat_rate`
  Default: `20.0`

## Default Xbox 360 Mapping

Config file:

- [`src/robolubo/config/teleop/xbox360.config.yaml`](/home/alexis/lubobot/src/robolubo/config/teleop/xbox360.config.yaml)

Current mapping:

- left stick vertical: forward/backward
- right stick horizontal: yaw
- left bumper: enable motion
- right bumper: turbo mode

## Docker

The ROS dev container now detects joystick devices under `/dev/input/js*` and
includes the shared device override automatically when present.
