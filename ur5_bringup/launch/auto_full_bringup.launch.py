import os
import glob
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _read_file(path: str) -> str:
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return ""


def _is_geomagic_usb(tty_path: str) -> bool:
    # tty_path example: /sys/class/tty/ttyACM0
    # Walk up to the USB device and read manufacturer/product fields
    dev_path = os.path.join(tty_path, "device")
    if not os.path.exists(dev_path):
        return False

    # Resolve symlink chain to reach the USB device folder that contains manufacturer/product
    try:
        real = os.path.realpath(dev_path)
        # Climb parents looking for manufacturer/product files
        curr = real
        keywords = ["Geomagic", "SensAble", "3D Systems", "Touch"]
        for _ in range(6):  # limit climb depth
            manuf = _read_file(os.path.join(curr, "manufacturer"))
            prod = _read_file(os.path.join(curr, "product"))
            if any(k.lower() in manuf.lower() for k in keywords) or any(
                k.lower() in prod.lower() for k in keywords
            ):
                return True
            curr = os.path.dirname(curr)
    except Exception:
        return False

    return False


def detect_geomagic_usb_count() -> int:
    tty_list = sorted(glob.glob("/sys/class/tty/ttyACM*"))
    count = 0
    for tty in tty_list:
        if _is_geomagic_usb(tty):
            count += 1
    return count


def _compose(context, *args, **kwargs):
    r1_sim = LaunchConfiguration("r1_simulation").perform(context)
    r2_sim = LaunchConfiguration("r2_simulation").perform(context)
    single_device = LaunchConfiguration("single_device_name").perform(context)
    single_omni = LaunchConfiguration("single_omni_name").perform(context)
    force_usb = LaunchConfiguration("force_usb_count").perform(context)

    actions = []

    # 1) Detectar dispositivos Geomagic por USB (o forzar para pruebas)
    try:
        usb_count = int(force_usb) if force_usb not in (None, "", "unset") else detect_geomagic_usb_count()
    except Exception:
        usb_count = detect_geomagic_usb_count()

    # 2) Lanzar Omni(s) según disponibilidad
    omni_share = get_package_share_directory("omni_common")
    if usb_count >= 2:
        dual_path = os.path.join(omni_share, "launch", "dual_omni_state.launch.py")
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dual_path))
        )
    elif usb_count == 1:
        single_path = os.path.join(omni_share, "launch", "single_omni_state.launch.py")
        actions.append(
            Node(
                package="ros2launch",
                executable="ros2launch",
                name="_log_single_omni_present",
                output="screen",
                arguments=["echo", f"Geomagic: 1 dispositivo detectado (single: {single_device})"]
            )
        )
        # Incluir el launch de un solo omni con parámetros
        actions.append(
            Node(
                package="omni_common",
                executable="omni_state",
                output="screen",
                parameters=[{"omni_name": single_omni}, {"device_name": single_device}],
                name=f"{single_omni}_state"
            )
        )
    else:
        actions.append(
            Node(
                package="ros2launch",
                executable="ros2launch",
                name="_log_no_omni_present",
                output="screen",
                arguments=["echo", "Geomagic: 0 dispositivos detectados (no se lanzan tópicos)"]
            )
        )

    # 3) Finalmente, lanzar auto_dual_bringup (URs y controladores con gating por tópicos)
    bringup_share = get_package_share_directory("ur5_bringup")
    auto_dual_path = os.path.join(bringup_share, "launch", "auto_dual_bringup.launch.py")
    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(auto_dual_path))
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("r1_simulation", default_value="true"),
        DeclareLaunchArgument("r2_simulation", default_value="true"),
        DeclareLaunchArgument("single_device_name", default_value="phantom2"),
        DeclareLaunchArgument("single_omni_name", default_value="phantom2"),
        DeclareLaunchArgument("force_usb_count", default_value=""),  # para pruebas: "0", "1" o "2"
        OpaqueFunction(function=_compose),
    ])
