"""
Keyboard teleop launch file.

Run this on your PC (not the Jetson) — it needs an interactive terminal.
The zenoh bridge forwards /cmd_vel to the Jetson automatically.

  ros2 launch robolubo teleop_keyboard.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Velocity command topic",
    )
    terminal_prefix_arg = DeclareLaunchArgument(
        "terminal_prefix",
        default_value="",
        description="Optional command prefix for opening teleop in another terminal, e.g. 'xterm -e'",
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        remappings=[("/cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
        prefix=LaunchConfiguration("terminal_prefix"),
    )

    return LaunchDescription([
        cmd_vel_arg,
        terminal_prefix_arg,
        teleop_node,
    ])
