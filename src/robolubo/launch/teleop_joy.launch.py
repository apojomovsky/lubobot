from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Velocity command topic",
    )
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device node to open",
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.12",
        description="Joystick deadzone",
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        "autorepeat_rate",
        default_value="20.0",
        description="Rate at which held joystick state is republished",
    )

    config = PathJoinSubstitution(
        [FindPackageShare("robolubo"), "config", "teleop", "xbox360.config.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": 0,
                "dev": LaunchConfiguration("joy_dev"),
                "deadzone": LaunchConfiguration("deadzone"),
                "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
            }
        ],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[config],
        remappings=[("/cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
    )

    return LaunchDescription(
        [
            cmd_vel_arg,
            joy_dev_arg,
            joy_deadzone_arg,
            joy_autorepeat_arg,
            joy_node,
            teleop_node,
        ]
    )
