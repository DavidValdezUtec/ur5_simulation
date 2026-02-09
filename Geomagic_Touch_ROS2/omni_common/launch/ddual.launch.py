import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
 

def generate_launch_description():

    # --- Nodos (definidos al principio para claridad) ---
    touch_driver_bin = os.environ.get(
        "TOUCH_DRIVER_BIN_DIR",
        os.path.expanduser("~/.local/share/geomagic/bin")
    )
    touch_driver_path = os.path.join(touch_driver_bin, "Touch_HeadlessSetup")
    
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
