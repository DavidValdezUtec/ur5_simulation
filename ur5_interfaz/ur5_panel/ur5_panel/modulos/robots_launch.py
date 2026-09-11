"""Lanzamiento de 'ur5e_bringup multi_ur5e.launch.py': traduce la config
de la UI (dicts r1_config/r2_config del panel) al config.json que lee ese
launch, lo lanza, y sondea hasta que ambos topicos /r{1,2}/robot_description
existen para agregar los robots al rviz_widget."""
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

from .procesos import terminar_proceso_gracefully


class RobotsLaunchModule:
    def __init__(self, rviz_widget):
        self.rviz_widget = rviz_widget
        self.process = None
        self.running = False
        self._timer = None
        self._intentos = 0
        self._on_ready = None

    def _escribir_config(self, r1_config, r2_config):
        def _a_robot_json(cfg, name):
            return {
                "name": name,
                "tf_prefix": cfg["tf_prefix"],
                "xyz": [cfg["pos_x"], cfg["pos_y"], cfg["pos_z"]],
                "rpy": [cfg["rot_x"], cfg["rot_y"], cfg["rot_z"]],
                "ur_type": cfg["ur_type"],
                "robot_ip": cfg["robot_ip"],
                "tcp_port": cfg["script_sender_port"],
                "use_fake_hardware": cfg["use_fake_hardware"],
            }

        robots = [_a_robot_json(r1_config, "r1"), _a_robot_json(r2_config, "r2")]

        share_dir = get_package_share_directory("ur5e_bringup")
        config_path = os.path.join(share_dir, "config", "config.json")
        with open(config_path, "w") as f:
            json.dump(robots, f, indent=4)
        return config_path

    def lanzar(self, r1_config, r2_config, on_ready=None):
        """Escribe la config y lanza multi_ur5e.launch.py. 'on_ready', si se
        pasa, se invoca (sin argumentos) cuando ambos robots aparecen en
        /ros2 topic list; por defecto los agrega directo al rviz_widget."""
        config_path = self._escribir_config(r1_config, r2_config)
        print(f"[Robots] Config de robots escrita en {config_path}")

        command = ['ros2', 'launch', 'ur5e_bringup', 'multi_ur5e.launch.py']
        try:
            print("[Robots] Iniciando launch de robots...")
            print(f"[Robots] Comando: {' '.join(command)}")
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.running = True
            print(f"[Robots] Proceso lanzado (PID: {self.process.pid})")
            self._armar_verificacion(on_ready)
        except Exception as e:
            print(f"[Robots] Error al lanzar robots: {e}")
            self.process = None
            self.running = False

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
        ur5e_unit en multi_ur5e.launch.py, hay que reflejar el mismo cambio
        aca para que el controlador siga viendo el mismo robot que el
        driver/rviz.
        """
        share_dir = get_package_share_directory("ur5e_bringup")
        xacro_path = os.path.join(share_dir, "urdf", "ur5e_single.urdf.xacro")

        # Misma derivacion de puertos que _robot_group() en
        # multi_ur5e.launch.py: el unico puerto base que se persiste es
        # script_sender_port (ahi guardado como "tcp_port" en config.json).
        tcp_port = int(robot_config["script_sender_port"])
        reverse_port = tcp_port - 1
        script_sender_port = tcp_port
        trajectory_port = tcp_port + 1
        script_command_port = tcp_port + 2

        xacro_args = [
            f"name:={robot_id}",
            f"tf_prefix:={robot_config['tf_prefix']}",
            f"x:={robot_config['pos_x']}",
            f"y:={robot_config['pos_y']}",
            f"z:={robot_config['pos_z']}",
            f"rx:={robot_config['rot_x']}",
            f"ry:={robot_config['rot_y']}",
            f"rz:={robot_config['rot_z']}",
            f"ur_type:={robot_config['ur_type']}",
            f"robot_ip:={robot_config['robot_ip']}",
            f"use_fake_hardware:={robot_config['use_fake_hardware']}",
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
        if self.process is not None:
            terminar_proceso_gracefully(self.process, 'robots')
            self.process = None
        self.running = False
