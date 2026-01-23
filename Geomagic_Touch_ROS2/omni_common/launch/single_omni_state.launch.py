from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.actions import ExecuteProcess


def generate_launch_description():
    touch_driver_path = os.path.expanduser('~/Documentos/TouchDriver_2024_09_19/bin/Touch_HeadlessSetup')
    calibrate = ExecuteProcess(
        cmd=[touch_driver_path, 'auto=phantom1'],
        shell=False,
        output='screen'
    )
    
    omni_name = LaunchConfiguration("omni_name")

    return LaunchDescription([
        calibrate,
        DeclareLaunchArgument("device_name", default_value="phantom1"),
        DeclareLaunchArgument("omni_name", default_value="phantom1"),
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[{"omni_name": omni_name}, {"device_name": omni_name}],
            name=omni_name
        ),
    ])
