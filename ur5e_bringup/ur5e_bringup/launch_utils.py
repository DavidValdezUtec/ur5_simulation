"""
Utilidades compartidas por los launch de ur5e_bringup:
  - multi_ur5e.launch.py      (robots reales / use_fake_hardware)
  - multi_ur5e_sim.launch.py  (Gazebo / Ignition Fortress)

Se instalan como modulo Python del paquete (setup.py -> find_packages), asi
que los launch los importan con 'from ur5e_bringup.launch_utils import ...'.
"""

import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Modelo UR usado si un robot no especifica "ur_type" en config.json.
DEFAULT_UR_TYPE = "ur5e"

# Celda por defecto si config/config.json esta vacio.
DEFAULT_ROBOTS = [
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


def default_config_path():
    """config/config.json instalado con el paquete: la celda por defecto al
    lanzar a mano. Quien genere su propia config (ej. ur5_panel) debe pasarla
    con el argumento 'config:=<ruta>' en vez de sobrescribir este archivo
    (con --symlink-install es un enlace al de src/)."""
    return os.path.join(
        get_package_share_directory("ur5e_bringup"), "config", "config.json"
    )


def declare_config_argument():
    """Argumento 'config' comun a multi_ur5e.launch.py y
    multi_ur5e_sim.launch.py."""
    return DeclareLaunchArgument(
        "config",
        default_value=default_config_path(),
        description="JSON con la lista de robots de la celda (ver config/config.json).",
    )


def load_robots(config_path=None):
    """Robots de la celda segun el JSON 'config_path' (por defecto
    config/config.json del paquete): nombre/namespace, pose respecto al
    'world' local de cada robot, IP real, modelo ("ur_type"), etc. Para
    agregar un robot solo hay que añadir una entrada alli. El prefijo de TF
    no se configura: es siempre '<name>_' (ver base_xacro_args)."""
    with open(config_path or default_config_path()) as f:
        config = json.load(f)
    return config if config else DEFAULT_ROBOTS


def base_xacro_args(robot):
    """Argumentos de xacro comunes a real y simulacion: identidad, pose y
    modelo del robot."""
    name = robot["name"]
    x, y, z = robot["xyz"]
    rx, ry, rz = robot.get("rpy", ("0", "0", "0"))
    return {
        "name": name,
        "tf_prefix": name + "_",
        "x": x, "y": y, "z": z,
        "rx": rx, "ry": ry, "rz": rz,
        "ur_type": robot.get("ur_type", DEFAULT_UR_TYPE),
    }


def robot_description(xacro_args):
    """Parametro 'robot_description' generado desde ur5e_single.urdf.xacro
    con los argumentos dados ({"name": "r1", "x": "0", ...}). Los valores
    pueden ser strings o Substitutions (ej. LaunchConfiguration)."""
    command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ur5e_bringup"), "urdf", "ur5e_single.urdf.xacro"]
        ),
    ]
    for key, value in xacro_args.items():
        command += [f" {key}:=", value if isinstance(value, Substitution) else str(value)]
    return {
        "robot_description": ParameterValue(value=Command(command), value_type=str)
    }


def render_controllers_file(template_path, robot_names):
    """Genera UN yaml de controladores con las secciones de todos los
    robots y devuelve su ruta. Es el que recibe, en <parameters>, el plugin
    ign_ros2_control de CADA robot en Gazebo. Por cada robot se copia la
    plantilla con:

      - '$(var tf_prefix)' -> '<robot>_': el plugin pasa el archivo tal cual
        a rclcpp, sin la sustitucion que hace ParameterFile(allow_substs=True)
        en el launch real.
      - '/**/' -> '/<robot>/': cada seccion aplica solo a los nodos de ese
        robot.

    Por que uno solo y no uno por robot: todos los plugins viven en el MISMO
    proceso (Gazebo) y cada uno sobrescribe los argumentos globales de rcl
    con su --params-file (gz_ros2_control 0.7, gz_ros2_control_plugin.cpp).
    Los controladores se crean despues, al cargarlos, y leen los argumentos
    del ULTIMO robot creado: con archivos separados, r1 recibia los joints de
    r2 (o ningun parametro). Con el archivo combinado da igual cual quede."""
    with open(template_path) as f:
        template = f.read()

    sections = []
    for name in robot_names:
        sections.append(
            f"# ---------- {name} ----------\n"
            + template.replace("$(var tf_prefix)", f"{name}_").replace("/**/", f"/{name}/")
        )

    out_dir = os.path.join(tempfile.gettempdir(), "ur5e_bringup")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "sim_controllers.yaml")
    with open(out_path, "w") as f:
        f.write("\n".join(sections))
    return out_path


def spawner(robot_name, controllers, active=True):
    """Spawner de controladores para el controller_manager de un robot.

    namespace absoluto y explicito: los spawner encadenados via
    OnProcessExit se crean fuera del alcance del PushRosNamespace del
    grupo, asi que no pueden depender de ese contexto ambiental. Debe ser
    absoluto ("/r1") y no relativo ("r1"): un spawner que SI corre dentro
    del PushRosNamespace apilaria un namespace relativo con el del grupo
    (-> "/r1/r1")."""
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=f"/{robot_name}",
        arguments=[
            "--controller-manager", "controller_manager",
            "--controller-manager-timeout", "20",
        ]
        + ([] if active else ["--inactive"])
        + controllers,
    )
