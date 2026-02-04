import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():

    # --- Nodos (definidos al principio para claridad) ---
    # La ruta ahora apunta a la copia local dentro del paquete
    touch_driver_path = os.path.join(
        get_package_share_directory('omni_common'),'omni_common',
        'bin', 'Touch_HeadlessSetup'
    )
    
    calibrate = ExecuteProcess(
        cmd=[touch_driver_path, 'auto=phantom1,phantom2'],
        shell=False,
        output='screen'
    )

    # Nodo para el dispositivo DERECHO (phantom1)
    omni_der_node = Node(
        package="omni_common",
        executable="omni_state",
        output="screen",
        parameters=[{"omni_name": "phantom1"}, {"device_name": "phantom1"}]
    )

    # Nodo para el dispositivo IZQUIERDO (phantom2)
    omni_iz_node = Node(
        package="omni_common",
        executable="omni_state",
        output="screen",
        parameters=[{"omni_name": "phantom2"}, {"device_name": "phantom2"}]
    )

    # --- Encadenamiento de Eventos ---

    return LaunchDescription([
        calibrate,
        RegisterEventHandler(
            OnProcessExit(
                target_action=calibrate,
                on_exit=[omni_der_node, omni_iz_node]
            )
        )
    ])
