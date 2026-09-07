"""Lanzamiento de 'ur5e_bringup multi_ur5e.launch.py': traduce la config
de la UI (dicts r1_config/r2_config del panel) al config.json que lee ese
launch, lo lanza, y sondea hasta que ambos topicos /r{1,2}/robot_description
existen para agregar los robots al rviz_widget."""
import json
import os
import subprocess

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

    def detener(self):
        if self._timer is not None:
            self._timer.stop()
        if self.process is not None:
            terminar_proceso_gracefully(self.process, 'robots')
            self.process = None
        self.running = False
