"""
Simula en Gazebo (Ignition Fortress) los mismos robots de config/config.json
que lanza multi_ur5e.launch.py, cada uno en su namespace y con su propio
controller_manager.

Diferencias con el launch real/fake:
  - El controller_manager de cada robot NO es un ros2_control_node: lo crea
    el plugin ign_ros2_control dentro de Gazebo al aparecer el modelo, en el
    namespace indicado en el xacro (<ros><namespace>/rN</namespace></ros>).
  - No hay nodos del driver UR (dashboard, controller_stopper, ...).
  - Hay una sola instancia de Gazebo y un bridge de /clock para todos, y
    todos los nodos usan use_sim_time.
  - Solo se cargan los controladores que funcionan sobre IgnitionSystem
    (config/ur5e_sim_controllers.yaml).
  - robot_ip, tcp_port y use_fake_hardware de config.json se ignoran.

Los robots se leen del JSON del argumento 'config', igual que en
multi_ur5e.launch.py (por defecto config/config.json del paquete).

Uso:
  ros2 launch ur5e_bringup multi_ur5e_sim.launch.py
  ros2 launch ur5e_bringup multi_ur5e_sim.launch.py gui:=false ft_sensor:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from ur5e_bringup.launch_utils import (
    base_xacro_args,
    declare_config_argument,
    load_robots,
    render_controllers_file,
    robot_description,
    spawner,
)

# Controladores por robot. Mismo criterio que el launch real:
# forward_position_controller activo, el resto cargados pero inactivos
# para poder cambiar con switch_controllers.
CONTROLLERS_ACTIVE = [
    "joint_state_broadcaster",
    "forward_position_controller",
]
CONTROLLERS_INACTIVE = [
    "joint_trajectory_controller",
    "scaled_joint_trajectory_controller",
    "forward_velocity_controller",
]


def _chain(steps):
    """Encadena procesos para que cada uno arranque cuando el anterior
    termina (con o sin error). steps = [(accion, espera_previa_s), ...]; el
    primero arranca de inmediato. Devuelve las acciones a lanzar."""
    first, _ = steps[0]
    actions = [first]
    for (prev, _), (action, delay) in zip(steps, steps[1:]):
        start = [TimerAction(period=delay, actions=[action])] if delay else [action]
        actions.append(
            RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=start))
        )
    return actions


def _robot(robot, controllers_file, context):
    """robot_state_publisher (en su grupo con namespace), el spawn del modelo
    y los spawners de controladores de un robot. El spawn y los spawners se
    devuelven sueltos para encadenarlos entre TODOS los robots."""
    xacro_args = base_xacro_args(robot)
    name = xacro_args["name"]
    use_ft_sensor = LaunchConfiguration("ft_sensor").perform(context) == "true"

    description = robot_description({
        **xacro_args,
        "use_fake_hardware": "false",
        "sim_ignition": "true",
        "simulation_controllers": controllers_file,
    })

    # El plugin ign_ros2_control lee robot_description de este nodo
    # (/rN/robot_state_publisher) para armar el controller_manager.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[description, {"use_sim_time": True}],
    )

    # La pose va en el propio URDF (base_mount_joint con el xyz/rpy de
    # config.json respecto a 'world'), asi que todos se crean en el origen.
    # namespace absoluto: se lanza desde la cadena, fuera del grupo.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=f"/{name}",
        output="screen",
        arguments=[
            "-name", name,
            "-topic", f"/{name}/robot_description",
            "-allow_renaming", "false",
        ],
    )

    spawners = [
        spawner(name, CONTROLLERS_ACTIVE),
        spawner(name, CONTROLLERS_INACTIVE, active=False),
    ]
    # El sensor F/T va en su propio spawner, al final: si el modelo no lo
    # declara, falla solo este y el resto de controladores queda intacto.
    if use_ft_sensor:
        spawners.append(spawner(name, ["force_torque_sensor_broadcaster"]))

    group = GroupAction([PushRosNamespace(name), robot_state_publisher_node])
    return group, spawn_entity, spawners


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    robot_configs = load_robots(LaunchConfiguration("config").perform(context))

    # Los meshes con package:// (ej. la herramienta de ur5e_bringup) Gazebo
    # los busca en IGN_GAZEBO_RESOURCE_PATH como '<ruta>/<paquete>/...'.
    # Con sim_ignition:=true los de ur_description ya van con rutas
    # absolutas, pero se agrega igual su share por si llegan como package://.
    resource_path = AppendEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.pathsep.join([
            os.path.dirname(get_package_share_directory("ur5e_bringup")),
            os.path.dirname(get_package_share_directory("ur_description")),
        ]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 3 {'' if gui else '-s '}{world}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Un solo yaml con los controladores de todos los robots (ver
    # render_controllers_file: los plugins comparten argumentos de rcl).
    controllers_file = render_controllers_file(
        os.path.join(
            get_package_share_directory("ur5e_bringup"),
            "config",
            "ur5e_sim_controllers.yaml",
        ),
        [robot["name"] for robot in robot_configs],
    )
    robots = [_robot(robot, controllers_file, context) for robot in robot_configs]

    # Una sola cadena para toda la celda: primero se crean todos los
    # modelos y luego se cargan los controladores robot por robot, nunca
    # dos spawners a la vez. A diferencia del launch real, aqui todos los
    # controller_manager viven en el MISMO proceso (Gazebo), y si dos
    # cargan la misma libreria de controladores a la vez pluginlib falla
    # con "no factory exists for it".
    # Tras el ultimo spawn se deja un margen para que los controller_manager
    # del plugin terminen de levantarse (el spawner igual espera hasta
    # --controller-manager-timeout); entre spawners, el mismo margen que en
    # el launch real para no perder el primer load_controller.
    steps = [(spawn_entity, 0.0) for _, spawn_entity, _ in robots]
    first_spawner = True
    for _, _, spawners in robots:
        for s in spawners:
            steps.append((s, 2.0 if first_spawner else 1.5))
            first_spawner = False

    return (
        [resource_path, gazebo, clock_bridge]
        + [group for group, _, _ in robots]
        + _chain(steps)
    )


def generate_launch_description():
    declared_arguments = [
        declare_config_argument(),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("ur5e_bringup"), "worlds", "celda.sdf"
            ),
            description="Mundo SDF de Gazebo (debe cargar UserCommands y ForceTorque).",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Abrir la interfaz grafica de Gazebo (false = solo servidor).",
        ),
        DeclareLaunchArgument(
            "ft_sensor",
            default_value="true",
            description="Activar force_torque_sensor_broadcaster en cada robot.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
