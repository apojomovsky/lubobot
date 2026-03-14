from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        description="Frame ID stamped on camera images",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="usb_camera",
        description="Camera name used by camera_info_manager",
    )
    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value="",
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
        default_value="30.0",
        description="Requested frame rate",
    )
    pixel_format_arg = DeclareLaunchArgument(
        "pixel_format",
        default_value="mjpeg2rgb",
        description="usb_cam pixel format conversion to request",
    )
    av_device_format_arg = DeclareLaunchArgument(
        "av_device_format",
        default_value="MJPG",
        description="Underlying V4L/ffmpeg device format",
    )
    io_method_arg = DeclareLaunchArgument(
        "io_method",
        default_value="mmap",
        description="usb_cam I/O method",
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace=LaunchConfiguration("namespace"),
        name="usb_cam",
        output="screen",
        parameters=[
            {
                "video_device": LaunchConfiguration("video_device"),
                "frame_id": LaunchConfiguration("frame_id"),
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "framerate": LaunchConfiguration("framerate"),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "av_device_format": LaunchConfiguration("av_device_format"),
                "io_method": LaunchConfiguration("io_method"),
            }
        ],
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
            av_device_format_arg,
            io_method_arg,
            camera_node,
        ]
    )
