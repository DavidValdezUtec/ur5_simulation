"""Llama al nodo controlador ur5_controller con sus argumentos y define los namespaces para cada robot.

Un launch por robot: el nodo queda como /<robot_id>/ur5_ik_node, asi los
controladores de r1 y r2 son nodos distintos (antes ambos se llamaban
/ur5_ik_node y rqt_graph / ros2 param los mezclaban).

Los parametros del controlador (incluido robot_description) llegan en un
YAML con clave '/**' que arma el panel (ver
RobotsLaunchModule.escribir_params_controller). 'nmspace' se fuerza al
robot_id para que namespace y topicos no puedan quedar desalineados.

Uso:
  ros2 launch ur5e_bringup controller.launch.py robot_id:=r1 \
      params_file:=$HOME/.ros/ur5_panel/r1_controller_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="r1",
            description="Robot a controlar (r1, r2): namespace del nodo y prefijo de sus topicos.",
        ),
        DeclareLaunchArgument(
            "params_file",
            description="YAML con los parametros de controller_node (clave '/**').",
        ),
        Node(
            package="ur5_controller",
            executable="controller_node",
            name="ur5_ik_node",
            namespace=robot_id,
            output="screen",
            emulate_tty=True,
            parameters=[params_file, {"nmspace": robot_id}],
        ),
    ])
