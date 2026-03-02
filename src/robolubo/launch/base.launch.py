from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial device connected to the Create 2",
    )

    config = PathJoinSubstitution(
        [FindPackageShare("robolubo"), "config", "create2.yaml"]
    )

    create_driver_node = Node(
        package="create_driver",
        executable="create_driver",
        name="create_driver",
        output="screen",
        parameters=[
            config,
            {
                "dev": LaunchConfiguration("serial_port"),
                "robot_model": "CREATE_2",
            },
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        create_driver_node,
    ])
