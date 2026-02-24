#!/usr/bin/env python3
import os
import sys
import atexit
import subprocess
import signal
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QPixmap, QImage, QTransform
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QRadioButton,
                             QVBoxLayout, QGridLayout, QSizePolicy, 
                             QPushButton, QLabel, QTabWidget, QLineEdit,
                             QComboBox, QHBoxLayout, QGroupBox, QCheckBox, QDockWidget,
                             QSlider, QScrollArea,QButtonGroup)
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

class CameraSubscriber(Node):
    """Nodo ROS2 para suscribirse al tópico de la cámara"""
    def __init__(self, callback):
        super().__init__('camera_subscriber_node')
        self.bridge = CvBridge()
        self.callback = callback
        self.subscription = self.create_subscription(
            Image,
            '/camera/usb/image_raw',  # Ajusta este tópico según tu configuración
            self.image_callback,
            10
        )
        self.get_logger().info('Camera subscriber initialized')
    
    def image_callback(self, msg):
        try:
            # Convertir imagen ROS a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Llamar al callback con la imagen
            self.callback(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')




class InterfazRviz(QMainWindow, UIMixin):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("INTERFAZ")
        self.resize(1600, 800)
        
        #Diccionario de estados:
        self.launch_processes = {
            'multi_haptic': None,
            'single_haptic': None,
            'camera': None,
            'robots': None
        }
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

    
    def update_video(self, cv_image):
        """Actualiza el widget de video con una nueva imagen de OpenCV"""
        try:
            # Convertir de BGR (OpenCV) a RGB (Qt)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Convertir a QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Escalar imagen manteniendo aspect ratio
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.video_label.width(), 
                self.video_label.height(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            
            # Mostrar en el QLabel
            self.video_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Error updating video: {e}")
    
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
        
        # Crear nodo suscriptor de cámara
        self.camera_node = CameraSubscriber(self.update_video)
        
        # Timer para procesar callbacks de ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.camera_node, timeout_sec=0.01))
        self.ros_timer.start(30)  # 30ms (~33 fps)
    
    
    
    

    '''
    ros2 run ur5_controller controller_node --ros-args \
  -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" \
  -p ur:="ur5e" \
  -p nmspace:="r1" \
  -p geomagic:="true" \
  -p geomagic_topic:="/phantom1/pose" \
  -p geomagic_button_topic:="/phantom1/button" \
  -p csv_log_enable:="true" \
  -p traj_mode:=1 \
  -p q_target:="[-1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0]" \
  -p map_x:=1 \
  -p map_y:=0 \
  -p map_z:=2 \
  -p sign_x:=-1.0 \
  -p sign_y:=1.0 \
  -p sign_z:=1.0 \
  -p map_roll:=0 \
  -p map_pitch:=1 \
  -p map_yaw:=2 \
  -p sign_roll:=-1.0 \
  -p sign_pitch:=1.0 \
  -p sign_yaw:=1.0


    '''    
    def start_controller(self, robot_id):
        """Inicia el nodo controlador con los parámetros de la interfaz"""
        print(f"[R{robot_id} Controller] Iniciando nodo controlador para Robot {robot_id}...")
        
        control_config = getattr(self, f"{robot_id}_control_config")
        
        # Construir comando con parámetros desde la interfaz
        # IMPORTANTE: -p y el parámetro deben ser argumentos separados
        command = [
            'ros2', 'run', 'ur5_controller', 'controller_node',
            '--ros-args',
            '-p', 'control_topic:=/forward_position_controller/commands',
            '-p', f'ur:={control_config["ur"]}',
            '-p', f'nmspace:={robot_id}',
            '-p', f'geomagic:={control_config["geomagic"]}',
            '-p', f'geomagic_topic:={"phantom1" if robot_id == "r1" else "phantom2"}/pose',
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
            '-p', f'sign_yaw:={float(control_config["sign_yaw"])}'
        ]
        
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
        
        process = subprocess.Popen(
                    ['ros2', 'control', 'switch_controllers', '--controller-manager', f'/{robot_id}/controller_manager', 
                     '--deactivate', f'/{robot_id}/forward_position_controller', 
                     '--activate', f'/{robot_id}/scaled_joint_trajectory_controller'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Crear nuevo grupo de procesos
                )
        self.launch_processes['single_haptic'] = process
        print(f"Haptic launch iniciado (PID: {process.pid})")
        
    def buscar_dispositivos(self):
        #apagar nodos hápticos antes de buscar
        
        print("[Main] Stopping any active haptic nodes before searching...")
        self.detener_camera_launch()
        self.detener_haptic_launch()
        
        print("[Main] Searching for haptic devices...")
        resultado = buscar_dispositivos()
        resultado_camara = buscar_camara()
        print(f"[Main] Search result: {resultado}")
        print("Numero de dispositivos hapticos encontrados:", resultado["num_dispositivos"])
        if resultado["num_dispositivos"] == 1:
            self.haptic1_ready = True
            self.led_haptic1.setStyleSheet("background-color: green; border-radius: 10px;")
            self.haptic2_ready = False
            self.led_haptic2.setStyleSheet("background-color: red; border-radius: 10px;")
            
        elif resultado["num_dispositivos"] == 2:
            self.haptic1_ready = True
            self.led_haptic1.setStyleSheet("background-color: green; border-radius: 10px;")
            self.haptic2_ready = True
            self.led_haptic2.setStyleSheet("background-color: green; border-radius: 10px;")
            #lanzar_nodos_haptico(2)
        else:
            self.haptic1_ready = False
            self.led_haptic1.setStyleSheet("background-color: red; border-radius: 10px;")
            self.haptic2_ready = False
            self.led_haptic2.setStyleSheet("background-color: red; border-radius: 10px;")
            self.detener_camera_launch()
            
        self.lanzar_haptic_launch()
        if resultado_camara["num_dispositivos"] > 1: #no se contará camara de la laptop
            self.camera_ready = True
            print("Camara encontrada")
            self.lanzar_camera_launch()
           
    def lanzar_haptic_launch(self):
        """Lanza el archivo launch de dispositivos hápticos"""
        if self.haptic1_ready and self.haptic2_ready:
            
            if self.launch_processes['multi_haptic'] is not None:
                print("Haptic launch ya está corriendo")
                return
            
            try:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'omni_common', 'ddual.launch.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Crear nuevo grupo de procesos
                )
                self.launch_processes['multi_haptic'] = process
                print(f"Haptic launch iniciado (PID: {process.pid})")
            except Exception as e:
                print(f"Error al lanzar haptic: {e}")
        elif self.haptic1_ready:
            
            if self.launch_processes['single_haptic'] is not None:
                print("Haptic launch ya está corriendo")
                return
            
            try:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'omni_common', 'single_omni_state.launch.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Crear nuevo grupo de procesos
                )
                self.launch_processes['single_haptic'] = process
                print(f"Haptic launch iniciado (PID: {process.pid})")
            except Exception as e:
                print(f"Error al lanzar haptic: {e}")
        else:
            print("No hay dispositivos hápticos disponibles para lanzar")
            
    def detener_haptic_launch(self):
        """Detiene el launch de dispositivos hápticos"""
        if self.launch_processes['multi_haptic'] is not None:
            self._terminar_proceso_gracefully(self.launch_processes['multi_haptic'], 'multi_haptic')
            self.launch_processes['multi_haptic'] = None
        
        if self.launch_processes['single_haptic'] is not None:
            self._terminar_proceso_gracefully(self.launch_processes['single_haptic'], 'single_haptic')
            self.launch_processes['single_haptic'] = None
    
    def lanzar_camera_launch(self):
        """Lanza el archivo launch de la cámara"""
        if self.launch_processes['camera'] is not None:
            print("Camera launch ya está corriendo")
            return
        
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'ur5_bringup', 'launch_camera.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Crear nuevo grupo de procesos
            )
            self.launch_processes['camera'] = process
            print(f"Camera launch iniciado (PID: {process.pid})")
        except Exception as e:
            print(f"Error al lanzar camera: {e}")
            
    def detener_camera_launch(self):
        """Detiene el launch de la cámara"""
        if self.launch_processes['camera'] is not None:
            self._terminar_proceso_gracefully(self.launch_processes['camera'], 'camera')
            self.launch_processes['camera'] = None
    
    def _terminar_proceso_gracefully(self, proceso, nombre):
        """Termina un proceso de forma gradual: SIGINT -> SIGTERM -> SIGKILL"""
        if proceso is None:
            return
        
        try:
            pgid = os.getpgid(proceso.pid)
            
            # Intento 1: SIGINT (Ctrl+C) - permite cleanup limpio
            print(f"[Shutdown] Enviando SIGINT a {nombre}...")
            os.killpg(pgid, signal.SIGINT)
            try:
                proceso.wait(timeout=8)
                print(f"[Shutdown] {nombre} cerrado correctamente")
                return
            except subprocess.TimeoutExpired:
                print(f"[Shutdown] {nombre} no respondió a SIGINT, escalando...")
            
            # Intento 2: SIGTERM - terminación estándar
            print(f"[Shutdown] Enviando SIGTERM a {nombre}...")
            os.killpg(pgid, signal.SIGTERM)
            try:
                proceso.wait(timeout=5)
                print(f"[Shutdown] {nombre} cerrado con SIGTERM")
                return
            except subprocess.TimeoutExpired:
                print(f"[Shutdown] {nombre} no respondió a SIGTERM, forzando cierre...")
            
            # Intento 3: SIGKILL - forzar cierre inmediato
            print(f"[Shutdown] Enviando SIGKILL a {nombre}...")
            os.killpg(pgid, signal.SIGKILL)
            proceso.wait(timeout=2)
            print(f"[Shutdown] {nombre} terminado forzosamente")
            
        except ProcessLookupError:
            print(f"[Shutdown] {nombre} ya no existe")
        except Exception as e:
            print(f"[Shutdown] Error al detener {nombre}: {e}")
    
    def detener_todos_los_launches(self):
        """Detiene todos los launches activos"""
        print("[Shutdown] Deteniendo todos los procesos launch...")
        for nombre, proceso in self.launch_processes.items():
            if proceso is not None:
                self._terminar_proceso_gracefully(proceso, nombre)
        
        # Resetear todos los procesos
        self.launch_processes = {k: None for k in self.launch_processes}
        print("[Shutdown] Todos los launches detenidos")
    
    def iniciar_robots(self):
        """Reinicia los robots: detiene si están corriendo y luego lanza"""
        print("[Robots] Reiniciando robots...")
        self.detener_robots()
        self.lanzar_robots()
    
    def lanzar_robots(self):
        """Lanza los launches de ambos robots simultáneamente"""
        # Construir comando con argumentos desde la interfaz
        command = [
            'ros2', 'launch', 'ur5_bringup', 'dual_control.launch.py',
            f'r1_type:={self.r1_config["ur_type"]}',
            f'r2_type:={self.r2_config["ur_type"]}',
            f'use_fake_hardware_r1:={self.r1_config["use_fake_hardware"]}',
            f'use_fake_hardware_r2:={self.r2_config["use_fake_hardware"]}',
            f'r1_IP:={self.r1_config["robot_ip"]}',
            f'r2_IP:={self.r2_config["robot_ip"]}',
            f'r1_TCP_port:={self.r1_config["script_sender_port"]}',
            f'r2_TCP_port:={self.r2_config["script_sender_port"]}',
            # Posiciones y orientaciones desde inputs   
            f'r1_x_pos:={self.r1_config["pos_x"]}',
            f'r1_y_pos:={self.r1_config["pos_y"]}',
            f'r1_z_pos:={self.r1_config["pos_z"]}',
            f'r1_rot_x:={self.r1_config["rot_x"]}',
            f'r1_rot_y:={self.r1_config["rot_y"]}',
            f'r1_rot_z:={self.r1_config["rot_z"]}',
            f'r2_x_pos:={self.r2_config["pos_x"]}',
            f'r2_y_pos:={self.r2_config["pos_y"]}',
            f'r2_z_pos:={self.r2_config["pos_z"]}',
            f'r2_rot_x:={self.r2_config["rot_x"]}',
            f'r2_rot_y:={self.r2_config["rot_y"]}',
            f'r2_rot_z:={self.r2_config["rot_z"]}',
            # Agrega más argumentos según necesites
        ]
            
        try:
            print("[Robots] Iniciando launch de robots...")
            print(f"[Robots] Comando: {' '.join(command)}")
            
            # Lanzar el proceso
            self.launch_processes['robots'] = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.robots_running = True
            print(f"[Robots] Proceso lanzado (PID: {self.launch_processes['robots'].pid})")
            
            # Iniciar timer para verificar tópicos periódicamente
            self.robot_topics_attempts = 0
            self.robot_topics_timer = QTimer()
            self.robot_topics_timer.timeout.connect(self.verificar_y_agregar_robots)
            self.robot_topics_timer.start(500)  # Verificar cada 500ms
            
        except Exception as e:
            print(f"[Robots] Error al lanzar robots: {e}")
            self.launch_processes['robots'] = None
    
    def verificar_y_agregar_robots(self):
        """Verifica si los tópicos de robots están disponibles y los agrega a RViz"""
        self.robot_topics_attempts += 1
        
        # Timeout después de 20 intentos (10 segundos)
        if self.robot_topics_attempts > 20:
            print("[Robots] Timeout esperando tópicos de robots")
            self.robot_topics_timer.stop()
            return
        
        try:
            # Verificar si los tópicos existen
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
                self.robot_topics_timer.stop()
                
                # Agregar robots dinámicamente
                self.rviz_widget.add_robot("/r1/robot_description")
                print("[Robots] Robot 1 agregado")
                self.rviz_widget.add_robot("/r2/robot_description")
                print("[Robots] Robot 2 agregado")
                print("[Robots] ¡Robots cargados exitosamente en RViz!")
            else:
                print(f"[Robots] Esperando tópicos... (intento {self.robot_topics_attempts}/20)")
                
        except Exception as e:
            print(f"[Robots] Error verificando tópicos: {e}")
    
    def detener_robots(self):
        """Detiene los launches de ambos robots"""
        if self.launch_processes['robots'] is not None:
            self._terminar_proceso_gracefully(self.launch_processes['robots'], 'robots')
            self.launch_processes['robots'] = None    
                
    def shutdown(self):
        print("[Main] Application closing...")
        
        try:
            # Detener timer de ROS
            if hasattr(self, 'ros_timer'):
                self.ros_timer.stop()
        except Exception as e:
            print(f"[Shutdown] Error deteniendo timer: {e}")
        
        try:
            # Destruir nodo de cámara
            if hasattr(self, 'camera_node'):
                self.camera_node.destroy_node()
        except Exception as e:
            print(f"[Shutdown] Error destruyendo nodo de cámara: {e}")
        
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