"""
Lanza varios robots UR (driver oficial ur_robot_driver) en namespaces
separados, cada uno con su propio controller_manager, robot_state_publisher
y controladores. Cada robot puede ser de un modelo distinto (ur5e, ur5, ...)
segun el campo "ur_type" de su entrada en config/config.json. Pensado
primero para probarse con use_fake_hardware:=true.

NOTA: el launch 'ur_control.launch.py' de ur_robot_driver no se reutiliza
directamente porque sus 'spawner' referencian el controller_manager con la
ruta absoluta "/controller_manager", lo que rompe al namespacear el robot.
Aqui se reconstruyen los mismos nodos usando una ruta relativa
"controller_manager", que si respeta el namespace de cada robot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
import json 

# Robots de la celda: nombre/namespace, prefijo de TF, pose respecto al
# 'world' local de cada robot, su IP real y su modelo ("ur_type": "ur5e",
# "ur5", etc; si se omite se usa DEFAULT_UR_TYPE). Para agregar un tercer
# robot solo hay que añadir una entrada aqui.
#importar json de config/config.json para obtener la configuracion de los robots
with open(FindPackageShare("ur5e_bringup").find("ur5e_bringup") + "/config/config.json") as f:
    config = json.load(f)
ROBOTS = config if config else [
    {"name": "r1",
     "xyz": ("0", "0", "0"),
     "ur_type": "ur5e",
     "robot_ip": "192.168.1.102",
     "tcp_port": "30002",
     "use_fake_hardware": "true"},
    {"name": "r2",
     "xyz": ("1.2", "0", "0"),
     "ur_type": "ur5",
     "robot_ip": "192.168.1.103",
     "tcp_port": "30002",
     "use_fake_hardware": "true"},
]

# Modelo UR usado si un robot no especifica "ur_type" en config.json.
DEFAULT_UR_TYPE = "ur5e"


def _robot_group(robot, context):
    name = robot["name"]
    tf_prefix = name+"_"
    x, y, z = robot["xyz"]
    ur_type = robot.get("ur_type", DEFAULT_UR_TYPE)

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    is_fake = use_fake_hardware.perform(context) == "true"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5e_bringup"), "urdf", "ur5e_single.urdf.xacro"]
            ),
            " name:=", name,
            " tf_prefix:=", tf_prefix,
            " x:=", x, " y:=", y, " z:=", z,
            " ur_type:=", ur_type,
            " tcp_port:=", robot["tcp_port"],
            " robot_ip:=", robot["robot_ip"],
            " use_fake_hardware:=", use_fake_hardware,
            " headless_mode:=", headless_mode,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", f"{ur_type}_update_rate.yaml"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur5e_bringup"), "config", "ur5e_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
            {"verify_payload_on_set": NotSubstitution(use_fake_hardware)},
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
            {"verify_payload_on_set": NotSubstitution(use_fake_hardware)},
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = IncludeLaunchDescription(
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        launch_description_source=AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_dashboard_client.launch.py"]
            )
        ),
        launch_arguments={"robot_ip": robot["robot_ip"]}.items(),
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[{"headless_mode": headless_mode}, {"robot_ip": robot["robot_ip"]}],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot["robot_ip"]}],
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": True},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    def spawner(controllers, active=True):
        # namespace absoluto y explicito: el spawner de inactivos se crea
        # despues (via OnProcessExit) fuera del alcance del PushRosNamespace
        # de este grupo, asi que no puede depender de ese contexto ambiental.
        # Debe ser absoluto ("/r1") y no relativo ("r1"): el spawner activo
        # SI corre dentro del PushRosNamespace, y un namespace relativo se
        # apilaria con el del grupo (-> "/r1/r1").
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=f"/{name}",
            arguments=[
                "--controller-manager", "controller_manager",
                "--controller-manager-timeout", "20",
            ]
            + ([] if active else ["--inactive"])
            + controllers,
        )

    # Misma cobertura de controladores que ur_control.launch.py: todo lo
    # necesario para control cinematico (trayectorias, velocidad, posicion,
    # esfuerzo) y dinamico (lectura de fuerza/torque, freedrive, force mode).
    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "friction_model_controller",
        "scaled_joint_trajectory_controller",
    ]
    if not is_fake:
        # tcp_pose_broadcaster depende de datos reales del controlador UR
        controllers_active.append("tcp_pose_broadcaster")

    controllers_inactive = [
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]

    # Los dos spawner (activos/inactivos) hablan con el MISMO controller_manager.
    # Si corren en paralelo, con una lista larga de controladores las llamadas
    # de servicio se pisan y una de ellas termina en timeout. Por eso el
    # spawner de inactivos se encadena para arrancar solo cuando el de
    # activos ya termino, y ambos arrancan con un pequeño margen: con dos
    # controller_managers arrancando a la vez, el primer load_controller
    # a veces se pierde si se dispara apenas el servicio aparece disponible
    # (el discovery DDS del lado servidor todavia no esta completamente
    # asentado aunque el cliente ya vea el servicio como disponible).
    active_spawner = spawner(controllers_active)
    delayed_active_spawner = TimerAction(period=3.0, actions=[active_spawner])
    inactive_spawner = spawner(controllers_inactive, active=False)
    chained_inactive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=active_spawner,
            on_exit=[TimerAction(period=1.5, actions=[inactive_spawner])],
        )
    )

    return GroupAction(
        [
            PushRosNamespace(name),
            SetLaunchConfiguration("tf_prefix", tf_prefix),
            control_node,
            ur_control_node,
            dashboard_client_node,
            robot_state_helper_node,
            urscript_interface,
            controller_stopper_node,
            robot_state_publisher_node,
            delayed_active_spawner,
            chained_inactive_spawner,
        ]
    )


def _launch_setup(context, *args, **kwargs):
    return [_robot_group(robot, context) for robot in ROBOTS]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Usar hardware simulado (mock) en vez de conectarse a los robots reales.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Modo headless del driver UR (sin usar la URCap externa).",
        ),
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Lanzar el dashboard_client de cada robot (ignorado si use_fake_hardware:=true).",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
