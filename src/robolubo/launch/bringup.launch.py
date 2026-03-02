"""
Top-level bringup launch file for the robolubo Create 2 base.

Runs on the Jetson:
  ros2 launch robolubo bringup.launch.py

Optional args:
  serial_port   — serial device for Create 2  (default: /dev/ttyUSB0)
  lidar_port    — serial device for HLS-LFCD2  (default: /dev/ttyUSB1)
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

    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB1",
        description="Serial device connected to the HLS-LFCD2 lidar",
    )

    pkg = FindPackageShare("robolubo")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, "launch", "base.launch.py"])
        ]),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, "launch", "lidar.launch.py"])
        ]),
        launch_arguments={
            "port": LaunchConfiguration("lidar_port"),
        }.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        base_launch,
        lidar_launch,
    ])
