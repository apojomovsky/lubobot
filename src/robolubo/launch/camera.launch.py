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
                    "frame_id": LaunchConfiguration("frame_id"),
                    "pixel_format": LaunchConfiguration("pixel_format"),
                    "output_encoding": LaunchConfiguration("output_encoding"),
                    "image_size": [image_width, image_height],
                    "time_per_frame": [1, framerate],
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_mount_tf",
            arguments=[
                "--x",
                LaunchConfiguration("camera_x"),
                "--y",
                LaunchConfiguration("camera_y"),
                "--z",
                LaunchConfiguration("camera_z"),
                "--roll",
                LaunchConfiguration("camera_roll"),
                "--pitch",
                LaunchConfiguration("camera_pitch"),
                "--yaw",
                LaunchConfiguration("camera_yaw"),
                "--frame-id",
                LaunchConfiguration("parent_frame"),
                "--child-frame-id",
                LaunchConfiguration("frame_id"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_optical_tf",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "-1.57079632679",
                "--pitch",
                "0.0",
                "--yaw",
                "-1.57079632679",
                "--frame-id",
                LaunchConfiguration("frame_id"),
                "--child-frame-id",
                LaunchConfiguration("optical_frame"),
            ],
        ),
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
        description="Physical camera frame attached to the robot body",
    )
    optical_frame_arg = DeclareLaunchArgument(
        "optical_frame",
        default_value="camera_optical_frame",
        description="Optical frame derived from camera_link using REP-style optical axes",
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="base_footprint",
        description="Parent robot frame for the camera mount transform",
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
    camera_x_arg = DeclareLaunchArgument(
        "camera_x",
        default_value="0.14",
        description="Camera x offset in meters relative to the parent frame",
    )
    camera_y_arg = DeclareLaunchArgument(
        "camera_y",
        default_value="0.0",
        description="Camera y offset in meters relative to the parent frame",
    )
    camera_z_arg = DeclareLaunchArgument(
        "camera_z",
        default_value="0.12",
        description="Camera z offset in meters relative to the parent frame",
    )
    camera_roll_arg = DeclareLaunchArgument(
        "camera_roll",
        default_value="0.0",
        description="Camera roll offset in radians relative to the parent frame",
    )
    camera_pitch_arg = DeclareLaunchArgument(
        "camera_pitch",
        default_value="-0.15",
        description="Camera pitch offset in radians relative to the parent frame",
    )
    camera_yaw_arg = DeclareLaunchArgument(
        "camera_yaw",
        default_value="0.0",
        description="Camera yaw offset in radians relative to the parent frame",
    )

    return LaunchDescription(
        [
            namespace_arg,
            video_device_arg,
            frame_id_arg,
            optical_frame_arg,
            parent_frame_arg,
            camera_name_arg,
            camera_info_url_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            pixel_format_arg,
            output_encoding_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_roll_arg,
            camera_pitch_arg,
            camera_yaw_arg,
            OpaqueFunction(function=_camera_node),
        ]
    )
