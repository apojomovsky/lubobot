from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial device for the STM32 USB CDC IMU bridge",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="imu_link",
        description="Frame ID attached to raw IMU measurements",
    )
    fixed_frame_arg = DeclareLaunchArgument(
        "fixed_frame",
        default_value="base_link",
        description="Base frame used by imu_filter_madgwick",
    )
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Whether imu_filter_madgwick should publish TF",
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("imu_bridge"), "launch", "imu_bridge.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "frame_id": LaunchConfiguration("frame_id"),
            "fixed_frame": LaunchConfiguration("fixed_frame"),
            "publish_tf": LaunchConfiguration("publish_tf"),
        }.items(),
    )

    return LaunchDescription(
        [
            port_arg,
            frame_id_arg,
            fixed_frame_arg,
            publish_tf_arg,
            imu_launch,
        ]
    )
