"""Lanzamiento de los robots con ur5e_bringup segun el modo de cada uno
('mode' en r1_config/r2_config, ver config_store.MODE_LABELS):

  - fake / real -> multi_ur5e.launch.py
  - gazebo      -> multi_ur5e_sim.launch.py

Los robots se reparten entre ambos launch: cada uno recibe con 'config:='
un JSON (en ~/.ros/ur5_panel/) con SOLO sus robots, y solo se lanza el que
tenga alguno. Asi se pueden mezclar, p.ej. r1 real y r2 en Gazebo.

Despues sondea hasta que ambos topicos /r{1,2}/robot_description existen
para agregar los robots al rviz_widget."""
import json
import os
import subprocess

import yaml
from PyQt5.QtCore import QTimer

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except Exception:
    PackageNotFoundError = Exception
    get_package_share_directory = None

from ur5_panel.config_store import DEFAULT_MODE, USER_CONFIG_DIR, tf_prefix

from .procesos import terminar_proceso_gracefully

# Launch de ur5e_bringup por grupo de robots: (archivo, args extra).
LAUNCHES = {
    "driver": ("multi_ur5e.launch.py", []),
    "gazebo": ("multi_ur5e_sim.launch.py", []),
}


def _launch_de(mode):
    """Modo de un robot -> clave de LAUNCHES que lo lanza."""
    return "gazebo" if mode == "gazebo" else "driver"


class RobotsLaunchModule:
    def __init__(self, rviz_widget):
        self.rviz_widget = rviz_widget
        # Un proceso por launch activo: {"driver": Popen, "gazebo": Popen}
        self.processes = {}
        # Robots lanzados: {"r1": {"launch": "driver", "mode": "real"}, ...}
        self.robots = {}
        self.running = False
        self._timer = None
        self._intentos = 0
        self._on_ready = None

    def _escribir_configs(self, robot_configs):
        """Escribe un JSON por launch con los robots que le tocan y devuelve
        {clave de LAUNCHES: ruta}. robot_configs = {"r1": cfg, "r2": cfg}.

        El tf_prefix no se escribe: ur5e_bringup lo deriva de "name"
        ('<name>_'), igual que config_store.tf_prefix(). Se escriben junto a
        la config de usuario: NO se toca el config.json de ur5e_bringup (con
        --symlink-install es un enlace al archivo versionado de src/)."""
        grupos = {}
        for name, cfg in robot_configs.items():
            mode = cfg.get("mode", DEFAULT_MODE)
            grupos.setdefault(_launch_de(mode), []).append({
                "name": name,
                "xyz": [cfg["pos_x"], cfg["pos_y"], cfg["pos_z"]],
                "rpy": [cfg["rot_x"], cfg["rot_y"], cfg["rot_z"]],
                "ur_type": cfg["ur_type"],
                "robot_ip": cfg["robot_ip"],
                "tcp_port": cfg["script_sender_port"],
                # Solo lo usa multi_ur5e.launch.py; el de Gazebo lo ignora.
                "use_fake_hardware": "true" if mode == "fake" else "false",
            })

        os.makedirs(USER_CONFIG_DIR, exist_ok=True)
        paths = {}
        for launch, robots in grupos.items():
            paths[launch] = os.path.join(USER_CONFIG_DIR, f"robots_{launch}.json")
            with open(paths[launch], "w") as f:
                json.dump(robots, f, indent=4)
        return paths

    def lanzar(self, r1_config, r2_config, on_ready=None):
        """Escribe las configs y lanza los launch necesarios segun el modo
        de cada robot. 'on_ready', si se pasa, se invoca (sin argumentos)
        cuando ambos robots aparecen en /ros2 topic list; por defecto los
        agrega directo al rviz_widget."""
        robot_configs = {"r1": r1_config, "r2": r2_config}
        paths = self._escribir_configs(robot_configs)

        log_dir = os.path.join(USER_CONFIG_DIR, "logs")
        os.makedirs(log_dir, exist_ok=True)
        for launch, config_path in paths.items():
            launch_file, extra_args = LAUNCHES[launch]
            command = ['ros2', 'launch', 'ur5e_bringup', launch_file,
                       f'config:={config_path}', *extra_args]
            # La salida va a un archivo: con stdout=PIPE sin leerlo, el
            # launch se bloquea al llenarse el buffer del pipe (~64 KB),
            # y Gazebo lo llena en segundos.
            log_path = os.path.join(log_dir, f"robots_{launch}.log")
            try:
                print(f"[Robots] Iniciando {launch_file} (config: {config_path})")
                print(f"[Robots] Comando: {' '.join(command)}")
                print(f"[Robots] Salida en {log_path}")
                with open(log_path, "w") as log:
                    self.processes[launch] = subprocess.Popen(
                        command,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        preexec_fn=os.setsid
                    )
                print(f"[Robots] Proceso lanzado (PID: {self.processes[launch].pid})")
            except Exception as e:
                print(f"[Robots] Error al lanzar {launch_file}: {e}")
                continue
            for name, cfg in robot_configs.items():
                mode = cfg.get("mode", DEFAULT_MODE)
                if _launch_de(mode) == launch:
                    self.robots[name] = {"launch": launch, "mode": mode}

        self.running = bool(self.processes)
        if self.running:
            self._armar_verificacion(on_ready)

    def estado_proceso(self, robot_id):
        """Estado del launch que lanzo a 'robot_id': 'sin_proceso' (no
        lanzado o detenido con detener()), 'vivo' o 'terminado' (el proceso
        murio sin que se pidiera; ver su log en ~/.ros/ur5_panel/logs/)."""
        robot = self.robots.get(robot_id)
        process = self.processes.get(robot["launch"]) if robot else None
        if process is None:
            return "sin_proceso"
        return "vivo" if process.poll() is None else "terminado"

    def modo(self, robot_id):
        """Modo ('fake'/'real'/'gazebo') con el que se lanzo 'robot_id', o
        None si no esta lanzado."""
        robot = self.robots.get(robot_id)
        return robot["mode"] if robot else None

    def _armar_verificacion(self, on_ready):
        self._intentos = 0
        self._on_ready = on_ready
        self._timer = QTimer()
        self._timer.timeout.connect(self._verificar_topicos)
        self._timer.start(500)  # Verificar cada 500ms

    def _verificar_topicos(self):
        """Sondea 'ros2 topic list' hasta ver ambos robot_description o
        agotar el timeout (20 intentos, 10s)."""
        self._intentos += 1
        if self._intentos > 20:
            print("[Robots] Timeout esperando tópicos de robots")
            self._timer.stop()
            return

        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            topics = result.stdout.strip().split('\n')
            r1_ready = '/r1/robot_description' in topics
            r2_ready = '/r2/robot_description' in topics

            if r1_ready and r2_ready:
                print("[Robots] Tópicos detectados, agregando robots a RViz...")
                self._timer.stop()
                if self._on_ready is not None:
                    self._on_ready()
                else:
                    self.rviz_widget.add_robot("/r1/robot_description")
                    print("[Robots] Robot 1 agregado")
                    self.rviz_widget.add_robot("/r2/robot_description")
                    print("[Robots] Robot 2 agregado")
                    print("[Robots] ¡Robots cargados exitosamente en RViz!")
            else:
                print(f"[Robots] Esperando tópicos... (intento {self._intentos}/20)")
        except Exception as e:
            print(f"[Robots] Error verificando tópicos: {e}")

    def generar_robot_description(self, robot_id, robot_config):
        """Corre xacro localmente con los mismos argumentos que arma
        multi_ur5e.launch.py para 'robot_id', y devuelve el URDF resuelto
        (string). Se usa para pasarle el XML directo a controller_node vía
        '-p robot_description:=...', sin depender de que el robot ya este
        corriendo (no lee el topico /robot_description).

        IMPORTANTE: si se cambian los argumentos que recibe la macro
        ur5e_unit en multi_ur5e.launch.py (o multi_ur5e_sim.launch.py para
        robots en modo Gazebo), hay que reflejar el mismo cambio
        aca para que el controlador siga viendo el mismo robot que el
        driver/rviz.
        """
        share_dir = get_package_share_directory("ur5e_bringup")
        xacro_path = os.path.join(share_dir, "urdf", "ur5e_single.urdf.xacro")

        # Misma derivacion de puertos que _robot_group() en
        # multi_ur5e.launch.py: el unico puerto base que se persiste es
        # script_sender_port (ahi guardado como "tcp_port" en config.json).
        mode = robot_config.get("mode", DEFAULT_MODE)
        tcp_port = int(robot_config["script_sender_port"])
        reverse_port = tcp_port - 1
        script_sender_port = tcp_port
        trajectory_port = tcp_port + 1
        script_command_port = tcp_port + 2

        xacro_args = [
            f"name:={robot_id}",
            f"tf_prefix:={tf_prefix(robot_id)}",
            f"x:={robot_config['pos_x']}",
            f"y:={robot_config['pos_y']}",
            f"z:={robot_config['pos_z']}",
            f"rx:={robot_config['rot_x']}",
            f"ry:={robot_config['rot_y']}",
            f"rz:={robot_config['rot_z']}",
            f"ur_type:={robot_config['ur_type']}",
            f"robot_ip:={robot_config['robot_ip']}",
            f"use_fake_hardware:={'true' if mode == 'fake' else 'false'}",
            # Gazebo: mismo hardware que multi_ur5e_sim.launch.py. La unica
            # diferencia que queda es la ruta del yaml en el plugin de
            # Gazebo (simulation_controllers), que Pinocchio ignora.
            f"sim_ignition:={'true' if mode == 'gazebo' else 'false'}",
            "headless_mode:=true",
            f"reverse_port:={reverse_port}",
            f"script_sender_port:={script_sender_port}",
            f"trajectory_port:={trajectory_port}",
            f"script_command_port:={script_command_port}",
        ]

        result = subprocess.run(
            ["xacro", xacro_path, *xacro_args],
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode != 0:
            raise RuntimeError(f"xacro fallo para '{robot_id}': {result.stderr.strip()}")
        return result.stdout

    def escribir_params_robot_description(self, robot_id, robot_description_xml):
        """Vuelca 'robot_description_xml' a un YAML de parametros en
        ~/.ros/ur5_panel/, para pasarselo a controller_node via
        '--params-file' en vez de '-p robot_description:=...': el XML trae
        ':' y '\"' (ej. xmlns:xacro=\"...\") que el parser YAML de '-p k:=v'
        de la CLI de ROS 2 puede interpretar mal. Un archivo YAML, volcado
        con PyYAML, escapa el string correctamente sin ese riesgo.
        """
        params_dir = os.path.join(os.path.expanduser('~'), '.ros', 'ur5_panel')
        os.makedirs(params_dir, exist_ok=True)
        params_path = os.path.join(params_dir, f'{robot_id}_controller_params.yaml')

        params = {'/**': {'ros__parameters': {'robot_description': robot_description_xml}}}
        with open(params_path, 'w') as f:
            yaml.safe_dump(params, f, default_flow_style=False)
        return params_path

    def detener(self):
        if self._timer is not None:
            self._timer.stop()
        # Cada launch corre en su propio grupo de procesos (setsid):
        # terminar_proceso_gracefully manda la senal al grupo entero, lo que
        # incluye a Gazebo (sh -> ruby ign gazebo) en el de simulacion.
        for launch, process in self.processes.items():
            terminar_proceso_gracefully(process, f'robots ({launch})')
        self.processes = {}
        self.robots = {}
        self.running = False
