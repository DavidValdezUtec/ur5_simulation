#!/usr/bin/env python3
import os
import sys
import atexit
import subprocess
import signal
import rclpy

from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QIcon, QPainter, QColor
from PyQt5.QtSvg import QSvgRenderer
from PyQt5 import QtCore

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except Exception:
    PackageNotFoundError = Exception
    get_package_share_directory = None

# Import funciones from the same package
from ur5_panel.funciones import *
from ur5_panel.ui_mixins import UIMixin
from ur5_panel.modulos.camera import CameraModule
from ur5_panel.modulos.haptic import HapticModule
from ur5_panel.modulos.robots_launch import RobotsLaunchModule

# Import RVizQtWidget from the installed ur5_interfaz_library package
try:
    from ur5_interfaz_library.RvizWrapper import RVizQtWidget
except ImportError as e:
    print(f"Error: Could not import RVizQtWidget from ur5_interfaz_library")
    print(f"Details: {e}")
    print("\nMake sure you have:")
    print("  1. Built ur5_interfaz_library: colcon build --packages-select ur5_interfaz_library")
    print("  2. Sourced the workspace: source install/setup.bash")
    sys.exit(1)

class InterfazRviz(QMainWindow, UIMixin):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("INTERFAZ")
        self.resize(1600, 800)
        
        # Modulos de composicion: cada uno maneja su propio proceso externo.
        # self.camera se crea sin video_label todavia (set_devices_menu, mas
        # adelante en setup_ui, ya llama a buscar_dispositivos() y necesita
        # poder detener/lanzar el launch de camara); set_camara_widget le
        # asigna el label real y arranca la suscripcion ROS2 cuando existe.
        # self.robots se crea en setup_ui, una vez existe el rviz_widget.
        self.camera = CameraModule(video_label=None)
        self.haptic = HapticModule()
        self.robots_running = False
        
        # Inicializar ROS2 para el nodo de la cámara
        if not rclpy.ok():
            rclpy.init()
        
        # Configurar widget central y layout principal
        try:
            self.setup_ui()
        except Exception as e:
            print(f"[Error] Fallo durante la inicialización: {e}")
            import traceback
            traceback.print_exc()
            # Asegurar limpieza antes de salir
            self.shutdown()
            raise
        
        # Configurar limpieza al salir
        atexit.register(self.shutdown)

    def cargar_y_colorear_svg(self, file_path, color):
        """
        Carga un archivo SVG, lo colorea y devuelve un QIcon.

        :param file_path: Ruta al archivo .svg.
        :param color: El nuevo color (p. ej., QColor(Qt.white), "#FF0000").
        :return: QIcon coloreado.
        """
        # 1. Renderizar el SVG original en un QPixmap
        renderer = QSvgRenderer(file_path)
        pixmap = QPixmap(renderer.defaultSize())
        pixmap.fill(Qt.transparent)  # Empezar con un fondo transparente

        painter = QPainter(pixmap)
        renderer.render(painter)
        painter.end()

        # 2. Crear una máscara a partir del pixmap renderizado
        #    La máscara usa el canal alfa del SVG
        mask = pixmap.createMaskFromColor(Qt.transparent)

        # 3. Crear un pixmap de resultado y rellenarlo con el color deseado
        result_pixmap = QPixmap(pixmap.size())
        result_pixmap.fill(QColor(color))

        # 4. Aplicar la máscara
        result_pixmap.setMask(mask)

        return QIcon(result_pixmap)
    
    def rotar_icon(self, icon, angle):
        pix = icon.pixmap(icon.actualSize(QSize(64, 64))) # Obtener pixmap del QIcon
        transform = QTransform().rotate(angle)
        rotated_pixmap = pix.transformed(transform, Qt.SmoothTransformation)
        return QIcon(rotated_pixmap)

    def _get_package_paths(self):
        """Obtiene rutas a recursos del paquete (share/config y share/resource)."""
        share_dir = None
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory('ur5_panel')
            except PackageNotFoundError:
                share_dir = None

        if share_dir is None:
            package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            share_dir = package_root

        icons_dir = os.path.join(share_dir, 'resource', 'icons')
        qss_path = os.path.join(share_dir, 'config', 'style.qss')
        return share_dir, icons_dir, qss_path
    
    def cargar_iconos(self):
        _, self.icon_path, _ = self._get_package_paths()
        
        self.icon_menu1 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu1.svg"), "#FFFFFF"), 0)
        self.icon_menu2 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu2.svg"), "#FFFFFF"), 90)
        self.icon_menu3 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu3.svg"), "#FFFFFF"), 90)
        self.icon_menu4 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu4.svg"), "#FFFFFF"), 90)
        self.icon_reload = self.cargar_y_colorear_svg(os.path.join(self.icon_path, "reload.svg"), "#FFFFFF")
        pass

    def setup_ui(self):
        # Widget principal
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QGridLayout()
        self.main_widget.setLayout(self.main_layout)

        # 2. Initialize RViz in Passive Mode (empty, without robots)
        print("Launching RViz in Passive Mode (empty - robots will be added on demand)...")
        # urdf_path="" tells the wrapper NOT to start its own state publishers
        # description_topic="" means no initial robot subscription
        try:
            self.rviz_widget = RVizQtWidget(
                urdf_path="", 
                description_topic="",  # Sin tópico inicial - robots se agregan dinámicamente
                fixed_frame="world"
            )
            self.rviz_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            # NO agregar robots aquí - se agregarán cuando se presione "Iniciar Robots"
        except Exception as e:
            print(f"[Error] No se pudo inicializar RViz widget: {e}")
            # Crear un widget placeholder en caso de error
            self.rviz_widget = QLabel("Error: No se pudo cargar RViz")
            self.rviz_widget.setStyleSheet("background-color: #2b2b2b; color: #ff6b6b; font-size: 16px;")
            self.rviz_widget.setAlignment(Qt.AlignCenter)
            raise

        self.robots = RobotsLaunchModule(self.rviz_widget)

        # Configurar menú lateral
        self.cargar_iconos()
        self.setup_menu()
        self.set_devices_menu()
        self.set_robot_menu()
        self.set_controller_menu()
        self.set_joint_control()
        
        # Configurar widget de cámara
        self.set_camara_widget()

        # Añadir dock widget a la ventana principal
        self.addDockWidget(Qt.RightDockWidgetArea, self.video_widget)
        
        # Añadir widgets al layout principal
        self.main_layout.addWidget(self.menu_general, 0, 0)
        self.main_layout.addWidget(self.rviz_widget, 0, 1)
        self.main_layout.addWidget(self.boton_salir, 1, 0)  # Botón Salir fuera del scroll

        '''# Configurar stretch1.8
        # Columna 0 (menú): tamaño mínimo
        # Columna 1 (RViz): se expande
        # Video: dock widget flotante/acoplable
        '''
        self.main_layout.setColumnStretch(0, 0)
        self.main_layout.setColumnStretch(1, 1)
        self.main_layout.setRowStretch(0, 1)

    
    def set_camara_widget(self):
        """Configura el widget de cámara como QDockWidget"""
        # Crear widget interno para el contenido
        video_content_widget = QWidget()
        self.video_layout = QVBoxLayout()
        video_content_widget.setLayout(self.video_layout)
        
        # Label para mostrar el video
        self.video_label = QLabel("Esperando video de cámara...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.video_label.setMinimumSize(640, 480)
        self.video_layout.addWidget(self.video_label)
        
        # Crear QDockWidget y asignarle el widget interno
        self.video_widget = QDockWidget("Cámara", self)
        self.video_widget.setWidget(video_content_widget)
        #self.video_widget.setWindowFlags(self.video_widget.windowFlags() | Qt.Tool)
        self.video_widget.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
        self.video_widget.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        
        # Configurar tamaño del dock widget (cuando está flotando)
        self.video_widget.resize(800, 600)  # Ancho x Alto cuando está flotante
        
        # self.camera ya existe (creado en __init__ sin video_label); ahora
        # que el label real existe, se lo asignamos y arrancamos la
        # suscripcion ROS2 + el timer que la bombea.
        self.camera.video_label = self.video_label
        self.camera.iniciar_suscripcion()
    
    
    
    

    '''
ros2 run ur5_controller controller_node --ros-args \
  -p geomagic:=true \
  -p ur:="ur5e" \
  -p nmspace:="ur5e" \
  -p urdf_path:="/ruta/al/robot.urdf" \
  -p robot_description:="" \
  -p geomagic_topic:="/phantom/state" \
  -p geomagic_button_topic:="/phantom/button" \
  -p use_ur5_pos_init:=true \
  -p q_target:="[0.0, -1.57, 1.57, 0.0, 1.57, 0.0]" \
  -p q_target_time:=3.0 \
  -p csv_log_enable:=true \
  -p csv_log_dir:="/tmp/ur5_logs" \
  -p csv_log_prefix:="run" \
  -p traj_A:="[0.1, 0.1, 0.1]" \
  -p traj_wn:=1.0 \
  -p traj_c0:=0.5 \
  -p traj_mode:=1 \
  -p controller_type:="QP" \
  -p Kp:="[1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 5000.0]" \
  -p Kd:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]" \
  -p lambda:="[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]" \
  -p k:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]" \
  -p k2:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]" \
  -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" \
  -p alpha:=0.01 \
  -p damping_factor:=0.01 \
  -p dt:=0.01 \
  -p ctrl_hz:=500.0 \
  -p max_joint_step_rad:=0.05 \
  -p large_error_threshold_rad:=0.15 \
  -p map_x:=2.0 -p map_y:=0.0 -p map_z:=1.0 \
  -p sign_x:=-1.0 -p sign_y:=-1.0 -p sign_z:=1.0 \
  -p map_roll:=2.0 -p map_pitch:=0.0 -p map_yaw:=1.0 \
  -p sign_roll:=1.0 -p sign_pitch:=1.0 -p sign_yaw:=1.0



    '''    
    def start_controller(self, robot_id):
        """Inicia el nodo controlador con los parámetros de la interfaz"""
        print(f"[R{robot_id} Controller] Iniciando nodo controlador para Robot {robot_id}...")

        control_config = getattr(self, f"{robot_id}_control_config")
        robot_config = getattr(self, f"{robot_id}_config")

        # El controlador necesita el URDF real del robot (con su herramienta,
        # definida en ur5e_single.urdf.xacro) para que la cinemática/dinámica
        # de Pinocchio coincida con lo que ya está corriendo en ur5e_bringup.
        # Se re-genera localmente con xacro (mismos argumentos que usa
        # multi_ur5e.launch.py) en vez de leerlo del tópico /robot_description,
        # para no depender de que el robot ya esté lanzado.
        params_file = None
        try:
            robot_description_xml = self.robots.generar_robot_description(robot_id, robot_config)
            params_file = self.robots.escribir_params_robot_description(robot_id, robot_description_xml)
        except Exception as e:
            print(f"[{robot_id} Controller] Error generando robot_description: {e}")
            print(f"[{robot_id} Controller] El controlador arrancará con el URDF genérico "
                  f"de respaldo (sin herramienta) en vez del real del robot.")

        # Construir comando con parámetros desde la interfaz
        # IMPORTANTE: -p y el parámetro deben ser argumentos separados
        command = [
            'ros2', 'run', 'ur5_controller', 'controller_node',
            '--ros-args',
            '-p', 'control_topic:=/forward_position_controller/commands',
            '-p', f'ur:={control_config["ur"]}',
            '-p', f'nmspace:={robot_id}',
            '-p', f'geomagic:={control_config["geomagic"]}',
            '-p', f'geomagic_topic:={"phantom1" if robot_id == "r1" else "phantom2"}/state',
            '-p', f'geomagic_button_topic:={"/phantom1/button" if robot_id == "r1" else "/phantom2/button"}',
            '-p', 'csv_log_enable:=true',
            '-p', f'traj_mode:={int(control_config["traj_mode"])}',
            '-p', f'q_target:=[{getattr(self, f"{robot_id}_q_target").text()}]',
            '-p', f'map_x:={float(control_config["map_x"])}',
            '-p', f'map_y:={float(control_config["map_y"])}',
            '-p', f'map_z:={float(control_config["map_z"])}',
            '-p', f'sign_x:={float(control_config["sign_x"])}',
            '-p', f'sign_y:={float(control_config["sign_y"])}',
            '-p', f'sign_z:={float(control_config["sign_z"])}',
            '-p', f'map_roll:={float(control_config["map_roll"])}',
            '-p', f'map_pitch:={float(control_config["map_pitch"])}',
            '-p', f'map_yaw:={float(control_config["map_yaw"])}',
            '-p', f'sign_roll:={float(control_config["sign_roll"])}',
            '-p', f'sign_pitch:={float(control_config["sign_pitch"])}',
            '-p', f'sign_yaw:={float(control_config["sign_yaw"])}',
            '-p', f'controller_type:={control_config["controller_type"]}',
            '-p', f'lambda:={control_config["lambda"]}',
            '-p', f'k:={control_config["k"]}', 
            '-p', f'alpha:={control_config["alpha"]}',
            '-p', f'traj_A:=[0.1,0.1,0.2]',

        ]
        if params_file is not None:
            command += ['--params-file', params_file]

        try:
            print(f"[{robot_id} Controller] Comando: {' '.join(command)}")
            # Lanzar el proceso del controlador en su propio grupo
            controller_process = subprocess.Popen(
                command,
                preexec_fn=os.setsid
            )
            setattr(self, f'{robot_id}_controller_process', controller_process)
        except Exception as e:
            print(f"[{robot_id} Controller] Error al iniciar el nodo controlador: {e}")
    
    
    def stop_controller(self, robot_id):
        """Detiene el nodo controlador del robot especificado"""
        controller_process = getattr(self, f'{robot_id}_controller_process', None)
        if controller_process is None:
            print(f"[{robot_id} Controller] No hay proceso de controlador activo para detener.")
            return

        print(f"[{robot_id} Controller] Deteniendo nodo controlador...")
        try:
            pgid = os.getpgid(controller_process.pid)

            # Intento 1: SIGINT (Ctrl+C)
            os.killpg(pgid, signal.SIGINT)
            try:
                controller_process.wait(timeout=5)
                print(f"[{robot_id} Controller] Nodo controlador detenido correctamente.")
                setattr(self, f'{robot_id}_controller_process', None)
                return
            except subprocess.TimeoutExpired:
                print(f"[{robot_id} Controller] No respondió a SIGINT, enviando SIGTERM...")

            # Intento 2: SIGTERM
            os.killpg(pgid, signal.SIGTERM)
            try:
                controller_process.wait(timeout=5)
                print(f"[{robot_id} Controller] Nodo controlador detenido con SIGTERM.")
                setattr(self, f'{robot_id}_controller_process', None)
                return
            except subprocess.TimeoutExpired:
                print(f"[{robot_id} Controller] No respondió a SIGTERM, forzando cierre...")

            # Intento 3: SIGKILL
            os.killpg(pgid, signal.SIGKILL)
            controller_process.wait(timeout=2)
            print(f"[{robot_id} Controller] Nodo controlador terminado forzosamente.")
        except ProcessLookupError:
            print(f"[{robot_id} Controller] El proceso ya no existe.")
        except Exception as e:
            print(f"[{robot_id} Controller] Error al detener el nodo controlador: {e}")
        finally:
            setattr(self, f'{robot_id}_controller_process', None)
    
    def on_r1_controller_tab_changed(self, index):
        """Se ejecuta cuando el usuario cambia de pestaña en el controlador del robot 1"""
        tab_names = ["Controller", "Joints", "Cartesian"]
        print(f"R1 Controller - Cambió a pestaña: {tab_names[index]} (índice {index})")
        
        # Aquí puedes agregar lógica específica según la pestaña
        if index == 0:
            print("  → Modo Controller activo")
        elif index == 1:
            print("  → Modo Joints activo")
        elif index == 2:
            print("  → Modo Cartesian activo")
        
    def cambiar_controller_topic(self, robot_id):
        # NOTA pre-existente: este proceso no es un dispositivo haptico, pero
        # ya se guardaba bajo la clave 'single_haptic'; se mantiene el mismo
        # comportamiento para no cambiar la logica de detener_todos_los_launches.
        process = subprocess.Popen(
                    ['ros2', 'control', 'switch_controllers', '--controller-manager', f'/{robot_id}/controller_manager',
                     '--deactivate', f'/{robot_id}/forward_position_controller',
                     '--activate', f'/{robot_id}/scaled_joint_trajectory_controller'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Crear nuevo grupo de procesos
                )
        self.haptic.single_process = process
        print(f"Haptic launch iniciado (PID: {process.pid})")

    def buscar_dispositivos(self):
        #apagar nodos hápticos antes de buscar
        print("[Main] Stopping any active haptic nodes before searching...")
        self.camera.detener_launch()
        self.haptic.detener_launch()

        print("[Main] Searching for haptic devices...")
        resultado = self.haptic.buscar()
        resultado_camara = buscar_camara()
        print(f"[Main] Search result: {resultado}")
        print("Numero de dispositivos hapticos encontrados:", resultado["num_dispositivos"])

        if self.haptic.haptic1_ready and self.haptic.haptic2_ready:
            self.led_haptic1.setStyleSheet("background-color: green; border-radius: 10px;")
            self.led_haptic2.setStyleSheet("background-color: green; border-radius: 10px;")
        elif self.haptic.haptic1_ready:
            self.led_haptic1.setStyleSheet("background-color: green; border-radius: 10px;")
            self.led_haptic2.setStyleSheet("background-color: red; border-radius: 10px;")
        else:
            self.led_haptic1.setStyleSheet("background-color: red; border-radius: 10px;")
            self.led_haptic2.setStyleSheet("background-color: red; border-radius: 10px;")
            self.camera.detener_launch()

        self.haptic.lanzar_launch()
        if resultado_camara["num_dispositivos"] > 1: #no se contará camara de la laptop
            self.camera_ready = True
            print("Camara encontrada")
            self.camera.lanzar_launch()

    def detener_todos_los_launches(self):
        """Detiene todos los launches activos"""
        print("[Shutdown] Deteniendo todos los procesos launch...")
        self.camera.detener_launch()
        self.haptic.detener_launch()
        self.robots.detener()
        print("[Shutdown] Todos los launches detenidos")
    
    def iniciar_robots(self):
        """Reinicia los robots: detiene si están corriendo y luego lanza"""
        print("[Robots] Reiniciando robots...")
        self.robots.detener()
        self.lanzar_robots()

    def lanzar_robots(self):
        """Lanza el launch de ambos robots (ur5e_bringup multi_ur5e.launch.py)"""
        launch_feedback = "true"
        if hasattr(self, 'feedback_checkbox'):
            launch_feedback = "true" if self.feedback_checkbox.isChecked() else "false"

        # NOTA: launch_feedback / initial_controller por robot ya no tienen
        # equivalente en ur5e_bringup (multi_ur5e.launch.py no los soporta
        # todavia); quedan pendientes de migrar, ver aviso en consola.
        if launch_feedback == "true":
            print("[Robots] Aviso: el feedback de fuerza (ur5_torque) no se "
                  "lanza con ur5e_bringup todavia; el checkbox no tiene efecto.")

        self.robots.lanzar(self.r1_config, self.r2_config)
        self.robots_running = self.robots.running

    def shutdown(self):
        print("[Main] Application closing...")
        
        try:
            # Detener timer de ROS y destruir nodo de cámara
            if hasattr(self, 'camera'):
                self.camera.detener_todo()
        except Exception as e:
            print(f"[Shutdown] Error deteniendo cámara: {e}")

        try:
            self.detener_todos_los_launches()
        except Exception as e:
            print(f"[Shutdown] Error deteniendo launches: {e}")
        
        try:
            if hasattr(self, 'rviz_widget') and hasattr(self.rviz_widget, 'shutdown'):
                self.rviz_widget.shutdown()
        except Exception as e:
            print(f"[Shutdown] Error cerrando RViz: {e}")
        
        try:
            # Shutdown ROS2
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[Shutdown] Error cerrando ROS2: {e}")
        
        print("[Shutdown] Limpieza completada")

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

def main():
    """Entry point for ros2 run command"""
    app = QApplication(sys.argv)
    window = None

    # Cargar y aplicar style.qss desde el share directory del paquete (compatible con install/)
    qss_path = None
    if get_package_share_directory is not None:
        try:
            qss_path = os.path.join(get_package_share_directory('ur5_panel'), 'config', 'style.qss')
        except PackageNotFoundError:
            qss_path = None
    if qss_path is None:
        qss_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/style.qss'))
    if os.path.exists(qss_path):
        print("Style cargado desde:", qss_path)
        with open(qss_path, 'r') as f:
            app.setStyleSheet(f.read())
    else:
        print(f"[Warning] No se encontró style.qss en: {qss_path}")

    # Manejar excepciones no capturadas
    def exception_hook(exctype, value, traceback_obj):
        """Asegurar limpieza en caso de excepción no manejada"""
        print(f"[Fatal Error] {exctype.__name__}: {value}")
        import traceback
        traceback.print_exception(exctype, value, traceback_obj)
        if window is not None:
            window.shutdown()
        sys.__excepthook__(exctype, value, traceback_obj)

    sys.excepthook = exception_hook

    try:
        window = InterfazRviz()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"[Fatal] Error durante la ejecución: {e}")
        if window is not None:
            window.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()