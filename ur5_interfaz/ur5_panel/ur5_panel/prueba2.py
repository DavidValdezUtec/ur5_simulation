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

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                             QVBoxLayout, QGridLayout, QSizePolicy, 
                             QPushButton, QLabel, QTabWidget)

# Import funciones from the same package
from ur5_panel.funciones import *

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

class InterfazRviz(QMainWindow):
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
        self.setup_menu()
        self.set_devices_menu()
        self.set_robot_menu()

        # Añadir widgets al layout principal
        self.main_layout.addWidget(self.menu_widget, 0, 0)
        self.main_layout.addWidget(self.video_widget, 0, 1)
        self.main_layout.addWidget(self.rviz_widget, 0, 2)

        # Configurar stretch
        # Columna 0 (menú): tamaño mínimo
        # Columna 1 (video): tamaño fijo/medio
        # Columna 2 (RViz): se expande
        self.main_layout.setColumnStretch(0, 0)
        self.main_layout.setColumnStretch(1, 1)
        self.main_layout.setColumnStretch(2, 2)
        self.main_layout.setRowStretch(0, 1)

    def setup_menu(self):
        #variables del menú
        self.haptic1_ready = False
        self.haptic2_ready = False
        self.camera_ready = False
        self.r1_config = {"ur_type": "ur5e",
                    "robot_ip": "192.168.10.104",
                    "description_package": "ur5_description",
                    "tf_prefix": "r1_",
                    "runtime_config_package": "ur5_bringup",
                    "controllers_file": ["ur_controllers_", "r1", ".yaml"],
                    "kinematics_params_file": ["/home/david/my_robot_calibration_", "ur5e", ".yaml"],
                    "use_fake_hardware": "true", # Empezamos en modo simulación
                    "launch_dashboard_client": "true",
                    "launch_rviz": "false", 
                    "reverse_port": "50001",
                    "script_sender_port": "50002",
                    "trajectory_port": "50003",
                    "script_command_port": "50004",
                    "pos_x": "0.0",
                    "pos_y": "0.9",
                    "pos_z": "0.0",
                    "rot_x": "0.0",
                    "rot_y": "0.0",
                    "rot_z": "0.0",}
        self.r2_config = {}
        
        
        
        self.menu_widget = QWidget()
        self.menu_layout = QVBoxLayout()
        self.menu_widget.setLayout(self.menu_layout)
        
        #layouts
        
        self.robots_layout = QVBoxLayout()
        
        # Widgets del menú
        self.label_menu = QLabel("Menu")
        self.boton_salir = QPushButton("Salir")
        self.boton_salir.clicked.connect(self.close)
        
        # Configurar widget de video
        self.video_widget = QWidget()
        self.video_layout = QVBoxLayout()
        self.video_widget.setLayout(self.video_layout)
        
        self.video_label = QLabel("Esperando video de cámara...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.video_label.setMinimumSize(640, 480)
        self.video_layout.addWidget(self.video_label)
        
        # Crear nodo suscriptor de cámara
        self.camera_node = CameraSubscriber(self.update_video)
        
        # Timer para procesar callbacks de ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.camera_node, timeout_sec=0.01))
        self.ros_timer.start(30)  # 30ms (~33 fps)



        # Placeholders
        self.device_widget = QWidget()
        self.robots_widget = QWidget(); self.robots_widget.setLayout(self.robots_layout)
        self.r1_widget = QWidget()
        self.r1_adv_widget = QWidget()
        self.r2_widget = QWidget()
        self.r2_adv_widget = QWidget()
        self.controller_widget = QWidget()
        self.r1_layout = QTabWidget()
        self.r2_layout = QTabWidget()
        self.button_haptic = QPushButton("Buscar Dispositivos")
        self.boton_iniciar_robots = QPushButton("Iniciar Robots")

        # Construir menú
        self.menu_layout.addWidget(self.label_menu)
        self.menu_layout.addWidget(self.device_widget)
        self.menu_layout.addWidget(self.robots_widget)
        self.menu_layout.addWidget(self.controller_widget)
        self.menu_layout.addStretch()
        self.menu_layout.addWidget(self.boton_salir)
        
    def set_devices_menu(self):
        self.device_layout = QGridLayout()
        self.device_widget.setLayout(self.device_layout)
        self.device_layout.addWidget(QLabel("Haptic Device Menu Placeholder"))
        self.device_layout.addWidget(self.button_haptic, 0,0,1,2)
        self.button_haptic.clicked.connect(self.buscar_dispositivos)
        
        #leds de estado 1/0 de los dispositivos hápticos
        self.led_haptic1 = QLabel()
        self.led_haptic1.setFixedSize(20, 20)
        self.led_haptic1.setStyleSheet("background-color: red; border-radius: 10px;")
        
        self.device_layout.addWidget(QLabel("Haptic Device 1:"), 1, 0)
        self.device_layout.addWidget(self.led_haptic1, 1, 1)
        self.led_haptic2 = QLabel()
        self.led_haptic2.setFixedSize(20, 20)
        self.led_haptic2.setStyleSheet("background-color: red; border-radius: 10px;")
        
        self.device_layout.addWidget(QLabel("Haptic Device 2:"), 2, 0)
        self.device_layout.addWidget(self.led_haptic2, 2, 1)
        
        self.buscar_dispositivos()

    
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
        pass
    
    
    
    def set_r1_menu(self):        
        
        self.r1_layout.addTab(self.r1_widget, "Robot 1")
        r1_buttons_layout = QVBoxLayout()
        r1_buttons_layout.addWidget(QLabel("Type:"))
        
        
        self.r1_widget.setLayout(r1_buttons_layout)
        
        self.r1_layout.addTab(self.r1_adv_widget, "Robot 1 Advanced")
        r1_adv_buttons_layout = QVBoxLayout()
        r1_adv_buttons_layout.addWidget(QLabel("Robot 1 Advanced Menu Placeholder"))
        
        self.r1_adv_widget.setLayout(r1_adv_buttons_layout)
           
    def set_r2_menu(self):        
        
        self.r2_layout.addTab(self.r2_widget, "Robot 2")
        r2_buttons_layout = QVBoxLayout()
        r2_buttons_layout.addWidget(QLabel("Robot 2 Menu Placeholder"))
        self.r2_widget.setLayout(r2_buttons_layout)
        
        self.r2_layout.addTab(self.r2_adv_widget, "Robot 2 Advanced")
        r2_adv_buttons_layout = QVBoxLayout()
        r2_adv_buttons_layout.addWidget(QLabel("Robot 2 Advanced Menu Placeholder"))
        self.r2_adv_widget.setLayout(r2_adv_buttons_layout)    
        
 
    def set_robot_menu(self):
        self.robots_layout.addWidget(self.r1_layout)
        self.robots_layout.addWidget(self.r2_layout)
        self.set_r1_menu()
        self.set_r2_menu()
        self.boton_iniciar_robots.clicked.connect(self.lanzar_robots)
        self.robots_layout.addWidget(self.boton_iniciar_robots)
        
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
    
    def lanzar_robots(self):
        """Lanza los launches de ambos robots simultáneamente"""
        if self.launch_processes['robots'] is not None:
            print("[Robots] Robots ya están corriendo")
            return
            
        try:
            print("[Robots] Iniciando launch de robots...")
            # Lanzar el proceso
            self.launch_processes['robots'] = subprocess.Popen(
                ['ros2', 'launch', 'ur5_bringup', 'dual_control.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
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