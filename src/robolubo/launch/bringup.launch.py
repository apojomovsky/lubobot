"""
Top-level bringup launch file for the robolubo Create 2 base.

Runs on the Jetson:
  ros2 launch robolubo bringup.launch.py

Optional args:
  serial_port   — serial device for Create 2  (default: /dev/ttyUSB0)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial device connected to the Create 2",
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robolubo"), "launch", "base.launch.py"
            ])
        ]),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        base_launch,
    ])
