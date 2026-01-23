# launch para lanzar ros2 launch omni_common omni_state_iz.launch.py, ros2 run ur5_controller controller_backup --ros-args -p ur5_time:=0.5 -p control_topic:="/joint_trajectory_controller/joint_trajectory" -p geomagic:=true -p geomagic_topic:="/phantom2/phantom/pose" -p geomagic_button_topic:="/phantom2/phantom/button"
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    laptop_camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='laptop_camera',
            namespace='camera/laptop',  # Usando el argumento 'namespace'
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [1280, 720],
            }]
        )

    # Nodo para la cámara USB externa
    usb_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        namespace='camera/usb', # Namespace diferente
        parameters=[{
            'video_device': '/dev/video2',
            'image_size': [1280, 720], # Puede tener una resolución diferente
        }]
    )

    return LaunchDescription([
        #laptop_camera_node,
        usb_camera_node
    ])
