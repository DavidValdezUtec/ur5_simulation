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

    return {
        robot_id: {**default_cfg, **loaded.get(robot_id, {})}
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
