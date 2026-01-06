from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_name = LaunchConfiguration("device_name")
    omni_name = LaunchConfiguration("omni_name")

    return LaunchDescription([
        DeclareLaunchArgument("device_name", default_value="phantom2"),
        DeclareLaunchArgument("omni_name", default_value="phantom2"),
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[{"omni_name": omni_name}, {"device_name": device_name}],
            name=omni_name
        ),
    ])
