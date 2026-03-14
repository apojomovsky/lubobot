from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            DeclareLaunchArgument("fixed_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            Node(
                package="imu_bridge",
                executable="imu_bridge_node",
                name="imu_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[
                    {
                        "use_mag": False,
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "world_frame": "enu",
                        "fixed_frame": LaunchConfiguration("fixed_frame"),
                    }
                ],
                remappings=[
                    ("imu/data_raw", "/imu/data_raw"),
                    ("imu/data", "/imu/data"),
                ],
            ),
        ]
    )
