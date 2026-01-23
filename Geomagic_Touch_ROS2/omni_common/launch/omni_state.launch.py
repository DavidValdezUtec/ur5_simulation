
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
 

def generate_launch_description():
    #Launch required components
    
    nodes = []

    nodes.append(
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[                
                {"omni_name": "phantom1"},
                {"device_name": "phantom1"},
                {"publish_rate": 1000},
                {"reference_frame": "/map"},
                {"units": "mm"}
            ]
        )
    )


    return LaunchDescription(nodes
        
    )
    
##SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", ATTRS{idProduct}=="0304", GROUP="users", MODE="0666"