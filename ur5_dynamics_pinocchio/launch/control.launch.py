from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_dynamics_pinocchio',
            executable='ur5_control_node',
            name='sliding_mode_control',
            output='screen',
            parameters=[
                {'lambda': 1.5},
                {'k_gain': 8.0},
                {'phi': 0.03},
                {'control_rate': 100.0},
                {'use_sim_time': True}
            ]
        )
    ])