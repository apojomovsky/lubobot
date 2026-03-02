from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB1",
        description="Serial port for the HLS-LFCD2 lidar",
    )

    lidar_node = Node(
        package="hls_lfcd_lds_driver",
        executable="hlds_laser_publisher",
        name="hlds_laser_publisher",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "frame_id": "laser",
        }],
        output="screen",
    )

    return LaunchDescription([
        port_arg,
        lidar_node,
    ])
