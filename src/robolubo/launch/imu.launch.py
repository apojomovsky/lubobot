from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
        default_value="base_footprint",
        description="Base frame used by imu_filter_madgwick",
    )
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Whether imu_filter_madgwick should publish TF",
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="base_footprint",
        description="Parent frame for the static IMU mount transform",
    )
    imu_x_arg = DeclareLaunchArgument(
        "imu_x",
        default_value="-0.07",
        description="IMU x offset in meters relative to the parent frame",
    )
    imu_y_arg = DeclareLaunchArgument(
        "imu_y",
        default_value="0.0",
        description="IMU y offset in meters relative to the parent frame",
    )
    imu_z_arg = DeclareLaunchArgument(
        "imu_z",
        default_value="0.0",
        description="IMU z offset in meters relative to the parent frame",
    )
    imu_roll_arg = DeclareLaunchArgument(
        "imu_roll",
        default_value="0.0",
        description="IMU roll offset in radians relative to the parent frame",
    )
    imu_pitch_arg = DeclareLaunchArgument(
        "imu_pitch",
        default_value="0.0",
        description="IMU pitch offset in radians relative to the parent frame",
    )
    imu_yaw_arg = DeclareLaunchArgument(
        "imu_yaw",
        default_value="0.0",
        description="IMU yaw offset in radians relative to the parent frame",
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

    imu_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_tf",
        arguments=[
            "--x",
            LaunchConfiguration("imu_x"),
            "--y",
            LaunchConfiguration("imu_y"),
            "--z",
            LaunchConfiguration("imu_z"),
            "--roll",
            LaunchConfiguration("imu_roll"),
            "--pitch",
            LaunchConfiguration("imu_pitch"),
            "--yaw",
            LaunchConfiguration("imu_yaw"),
            "--frame-id",
            LaunchConfiguration("parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("frame_id"),
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            frame_id_arg,
            fixed_frame_arg,
            publish_tf_arg,
            parent_frame_arg,
            imu_x_arg,
            imu_y_arg,
            imu_z_arg,
            imu_roll_arg,
            imu_pitch_arg,
            imu_yaw_arg,
            imu_launch,
            imu_static_tf,
        ]
    )
