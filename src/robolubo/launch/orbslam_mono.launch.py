from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/image_raw",
        description="Monocular image topic for ORB-SLAM3",
    )
    settings_file_arg = DeclareLaunchArgument(
        "settings_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("ros2_orb_slam3"),
                "orb_slam3",
                "config",
                "Monocular",
                "EuRoC.yaml",
            ]
        ),
        description="ORB-SLAM3 monocular settings YAML",
    )
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/orb_slam3/pose",
        description="Pose output topic",
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="map",
        description="Frame ID used for the published pose",
    )
    use_pangolin_arg = DeclareLaunchArgument(
        "use_pangolin",
        default_value="true",
        description="Enable the Pangolin viewer window",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz alongside ORB-SLAM3",
    )

    orbslam_node = Node(
        package="ros2_orb_slam3",
        executable="mono_node_cpp",
        name="orbslam_mono",
        output="screen",
        parameters=[
            {
                "node_name_arg": "orbslam_mono",
                "live_mode_arg": True,
                "image_topic_arg": LaunchConfiguration("image_topic"),
                "settings_name_arg": LaunchConfiguration("settings_file"),
                "pose_topic_arg": LaunchConfiguration("pose_topic"),
                "world_frame_arg": LaunchConfiguration("world_frame"),
                "enable_pangolin_arg": LaunchConfiguration("use_pangolin"),
            }
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("robolubo"), "launch", "rviz.launch.py"]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            image_topic_arg,
            settings_file_arg,
            pose_topic_arg,
            world_frame_arg,
            use_pangolin_arg,
            use_rviz_arg,
            orbslam_node,
            rviz_launch,
        ]
    )
