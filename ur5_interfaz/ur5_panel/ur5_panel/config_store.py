"""
Persistencia de la configuracion de robots (r1_config/r2_config) del panel.

El paquete instala una plantilla de fabrica en share/ur5_panel/config/config.json;
esa copia se pisa en cada 'colcon build' porque install/ se regenera desde src/.
Los cambios hechos desde la UI se guardan en ~/.ros/ur5_panel/config.json, que
sobrevive a rebuilds. Esa copia de usuario se crea a partir de la plantilla la
primera vez que no existe, y es la unica que este modulo lee/escribe despues.
"""
import json
import os
import shutil

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except Exception:
    PackageNotFoundError = Exception
    get_package_share_directory = None

PACKAGE_NAME = 'ur5_panel'
USER_CONFIG_DIR = os.path.join(os.path.expanduser('~'), '.ros', PACKAGE_NAME)
USER_CONFIG_PATH = os.path.join(USER_CONFIG_DIR, 'config.json')


# Claves que ya no se configuran y se descartan al cargar un archivo viejo.
#  - tf_prefix: se deriva siempre del id del robot (ver tf_prefix()).
#  - use_fake_hardware: reemplazada por 'mode' (se migra en load_config).
OBSOLETE_KEYS = ('tf_prefix', 'use_fake_hardware')

# Modo de cada robot ('mode' en su config) -> texto del selector de la UI.
#  - fake:   ur5e_bringup multi_ur5e.launch.py con use_fake_hardware:=true
#  - real:   ur5e_bringup multi_ur5e.launch.py con el driver del UR real
#  - gazebo: ur5e_bringup multi_ur5e_sim.launch.py (Ignition Gazebo)
MODE_LABELS = {'fake': 'Simulation', 'real': 'Real', 'gazebo': 'Gazebo'}
DEFAULT_MODE = 'fake'


def mode_from_label(label):
    """Texto del selector de la UI -> valor de 'mode'."""
    return next((m for m, text in MODE_LABELS.items() if text == label), DEFAULT_MODE)


def _migrate_robot_config(robot_cfg):
    """Convierte claves de versiones anteriores del archivo de usuario:
    use_fake_hardware ("true"/"false") -> mode ("fake"/"real")."""
    if 'mode' not in robot_cfg and 'use_fake_hardware' in robot_cfg:
        robot_cfg['mode'] = 'fake' if robot_cfg['use_fake_hardware'] == 'true' else 'real'
    for key in OBSOLETE_KEYS:
        robot_cfg.pop(key, None)
    return robot_cfg


def tf_prefix(robot_id):
    """Prefijo de TF/joints de un robot: '<robot_id>_' (ej. 'r1_').

    robot_id es a la vez el namespace ROS del robot (/r1) y su nombre en
    ur5e_bringup. Es la misma convencion que usan ur5e_bringup
    (launch_utils.base_xacro_args), ur5_controller y ur5_torque; no es
    configurable para que el URDF de todos coincida siempre."""
    return f"{robot_id}_"


def _template_config_path():
    share_dir = None
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory(PACKAGE_NAME)
        except PackageNotFoundError:
            share_dir = None
    if share_dir is None:
        share_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    return os.path.join(share_dir, 'config', 'config.json')


def load_config(defaults):
    """Carga la config de usuario en ~/.ros/ur5_panel/config.json.

    Si ese archivo todavia no existe, se crea copiando la plantilla instalada
    por el paquete (o, si tampoco esta disponible, volcando 'defaults').
    'defaults' es un dict {'r1': {...}, 'r2': {...}} usado tanto como respaldo
    como para completar claves nuevas que un archivo de usuario viejo no tenga.
    """
    if not os.path.exists(USER_CONFIG_PATH):
        os.makedirs(USER_CONFIG_DIR, exist_ok=True)
        template_path = _template_config_path()
        if os.path.exists(template_path):
            shutil.copyfile(template_path, USER_CONFIG_PATH)
            print(f"[Config] Configuracion de usuario creada en {USER_CONFIG_PATH} "
                  f"a partir de la plantilla del paquete.")
        else:
            with open(USER_CONFIG_PATH, 'w') as f:
                json.dump(defaults, f, indent=4)
            print(f"[Config] No se encontro la plantilla del paquete; se creo "
                  f"{USER_CONFIG_PATH} con valores por defecto.")

    try:
        with open(USER_CONFIG_PATH, 'r') as f:
            loaded = json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        print(f"[Config] Error leyendo {USER_CONFIG_PATH} ({e}); usando valores por defecto.")
        loaded = {}

    # La migracion va ANTES de mezclar con 'defaults': si no, el 'mode' por
    # defecto pisaria el que se deriva de un use_fake_hardware viejo.
    return {
        robot_id: _migrate_robot_config(
            {**default_cfg, **_migrate_robot_config(dict(loaded.get(robot_id, {})))}
        )
        for robot_id, default_cfg in defaults.items()
    }


def save_config(config):
    """Guarda 'config' ({'r1': {...}, 'r2': {...}}) en ~/.ros/ur5_panel/config.json.

    Nunca escribe en share/ del paquete: esa copia es solo la plantilla de fabrica.
    """
    try:
        os.makedirs(USER_CONFIG_DIR, exist_ok=True)
        with open(USER_CONFIG_PATH, 'w') as f:
            json.dump(config, f, indent=4)
    except OSError as e:
        print(f"[Config] Error guardando {USER_CONFIG_PATH}: {e}")
