import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera_node(context):
    video_device = LaunchConfiguration("video_device").perform(context)
    resolved_video_device = os.path.realpath(video_device)
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))
    framerate = int(LaunchConfiguration("framerate").perform(context))

    if not resolved_video_device.startswith("/dev/video"):
        resolved_video_device = video_device

    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            namespace=LaunchConfiguration("namespace"),
            name="camera",
            output="screen",
            parameters=[
                {
                    "video_device": resolved_video_device,
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "pixel_format": LaunchConfiguration("pixel_format"),
                    "output_encoding": LaunchConfiguration("output_encoding"),
                    "image_size": [image_width, image_height],
                    "time_per_frame": [1, framerate],
                }
            ],
        )
    ]


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="camera",
        description="Namespace for the USB camera topics",
    )
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="Video device to open",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="camera_link",
        description="Reserved for later TF integration; current v4l2_camera uses its own frame defaults",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="usb_camera",
        description="Reserved for later calibration naming; not currently used by v4l2_camera launch",
    )
    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value=PathJoinSubstitution(
            [
                "file://",
                FindPackageShare("robolubo"),
                "config",
                "camera",
                "general_webcam_640x480.yaml",
            ]
        ),
        description="Optional camera calibration URL, e.g. file:///path/to/calibration.yaml",
    )
    image_width_arg = DeclareLaunchArgument(
        "image_width",
        default_value="640",
        description="Image width",
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height",
        default_value="480",
        description="Image height",
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate",
        default_value="30",
        description="Requested frame rate numerator for time_per_frame [1, N]",
    )
    pixel_format_arg = DeclareLaunchArgument(
        "pixel_format",
        default_value="YUYV",
        description="V4L2 FOURCC pixel format",
    )
    output_encoding_arg = DeclareLaunchArgument(
        "output_encoding",
        default_value="rgb8",
        description="ROS image encoding produced by v4l2_camera",
    )

    return LaunchDescription(
        [
            namespace_arg,
            video_device_arg,
            frame_id_arg,
            camera_name_arg,
            camera_info_url_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            pixel_format_arg,
            output_encoding_arg,
            OpaqueFunction(function=_camera_node),
        ]
    )
