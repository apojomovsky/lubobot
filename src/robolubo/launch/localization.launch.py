from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Whether the EKF should publish odom -> base_link TF",
    )
    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="EKF odometry frame",
    )
    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame",
        default_value="base_link",
        description="Base link frame used by robot_localization",
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="odom",
        description="World frame for the EKF output",
    )

    config = PathJoinSubstitution(
        [FindPackageShare("robolubo"), "config", "localization", "ekf.yaml"]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            config,
            {
                "publish_tf": LaunchConfiguration("publish_tf"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_link_frame": LaunchConfiguration("base_link_frame"),
                "world_frame": LaunchConfiguration("world_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            publish_tf_arg,
            odom_frame_arg,
            base_link_frame_arg,
            world_frame_arg,
            ekf_node,
        ]
    )
