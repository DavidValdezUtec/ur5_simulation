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
from PyQt5.QtGui import QPixmap, QImage, QTransform
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QRadioButton,
                             QVBoxLayout, QGridLayout, QSizePolicy, 
                             QPushButton, QLabel, QTabWidget, QLineEdit,
                             QComboBox, QHBoxLayout, QGroupBox, QCheckBox, QDockWidget,
                             QSlider, QScrollArea,QButtonGroup)
from PyQt5.QtGui import QIcon
from PyQt5 import QtCore

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

    def rotar_icon(self, path_icon, angle):
        pix = QPixmap(path_icon)
        transform = QTransform().rotate(angle)
        rotated_pixmap = pix.transformed(transform, Qt.SmoothTransformation)
        return QIcon(rotated_pixmap)
    
    def cargar_iconos(self):
        self.icon_path = "/home/david/tesis_ws/src/ur5_simulation/ur5_interfaz/ur5_panel/resource/icons/"
        
        self.icon_menu1 = self.rotar_icon(self.icon_path + "menu1.svg", 0)
        self.icon_menu2 = self.rotar_icon(self.icon_path + "menu2.svg", 90)
        self.icon_menu3 = self.rotar_icon(self.icon_path + "menu3.svg", 90)
        self.icon_menu4 = self.rotar_icon(self.icon_path + "menu4.svg", 90)
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

        '''# Configurar stretch
        # Columna 0 (menú): tamaño mínimo
        # Columna 1 (RViz): se expande
        # Video: dock widget flotante/acoplable
        '''
        self.main_layout.setColumnStretch(0, 0)
        self.main_layout.setColumnStretch(1, 1)
        self.main_layout.setRowStretch(0, 1)

    def setup_menu(self):
        """Configura el menú lateral de la interfaz"""
        self.initialize_config_variables()
        self.create_menu_structure()
        self.initialize_menu_widgets()
        self.build_menu1_layout()
        self.build_menu2_layout()
    
    def initialize_config_variables(self):
        """Inicializa variables de configuración y estado"""
        # Estados de dispositivos
        self.haptic1_ready = False
        self.haptic2_ready = False
        self.camera_ready = False
        
        # Configuración Robot 1
        self.r1_config = {
            "ur_type": "ur5e",
            "robot_ip": "192.168.10.104",
            "description_package": "ur5_description",
            "tf_prefix": "r1_",
            "runtime_config_package": "ur5_bringup",
            "controllers_file": ["ur_controllers_", "r1", ".yaml"],
            "kinematics_params_file": ["/home/david/my_robot_calibration_", "ur5e", ".yaml"],
            "use_fake_hardware": "true",  # Empezamos en modo simulación
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
            "rot_z": "0.0",
        }
        
        # Configuración Robot 2
        self.r2_config = {}
        
        # Modos de control disponibles
        self.control_mode_r1 = ["Teleoperation", "Trayectoria"]
        self.control_mode_r2 = ["Teleoperation", "Trayectoria"]
    
    def create_menu_structure(self):
        """ Crea menu general"""
        self.menu_general = QTabWidget()
        self.menu_general.setObjectName("menu_general")
        self.menu_general.setTabPosition(QTabWidget.West)
        self.create_menu1_structure()
        self.create_menu2_structure()
        self.create_menu3_structure()
        self.create_menu4_structure()
        pass
    
    
    def create_menu1_structure(self):
                
        """Crea la estructura básica del menú"""
        # Widget interno del menú con scroll
        self.menu1_widget = QWidget()
        self.menu1_layout = QVBoxLayout()
        self.menu1_widget.setLayout(self.menu1_layout)
        
        # Scroll area para el menú
        self.menu_scroll = QScrollArea()
        self.menu_scroll.setWidget(self.menu1_widget)
        self.menu_scroll.setWidgetResizable(True)
        self.menu_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.menu_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.menu_scroll.setMinimumWidth(300)
        
        self.menu_general.addTab(self.menu_scroll, self.icon_menu1,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        
        # Layouts para secciones
        self.robots_layout = QVBoxLayout()
        self.robots_controller_layout = QVBoxLayout()
        
    def create_menu2_structure(self):
        self.menu2_layout = QVBoxLayout()
        self.menu2_widget = QWidget()
        self.menu2_widget.setLayout(self.menu2_layout)
        self.menu_general.addTab(self.menu2_widget, self.icon_menu2,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass
    
    def create_menu3_structure(self):
        self.menu3_layout = QVBoxLayout()
        self.menu3_widget = QWidget()
        self.menu3_widget.setLayout(self.menu3_layout)
        self.menu_general.addTab(self.menu3_widget, self.icon_menu3,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass
        
    def create_menu4_structure(self):
        self.menu4_layout = QVBoxLayout()
        self.menu4_widget = QWidget()
        self.menu4_widget.setLayout(self.menu4_layout)
        self.menu_general.addTab(self.menu4_widget, self.icon_menu4,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass
    
    
    
    def initialize_menu_widgets(self):
        """Inicializa todos los widgets del menú"""
        # Widgets principales
        self.label_menu = QLabel("Menu")
        self.boton_salir = QPushButton("Salir")
        self.boton_salir.clicked.connect(self.close)
        
        # Widgets para botones principales
        self.button_haptic = QPushButton("Buscar Dispositivos")
        self.boton_iniciar_robots = QPushButton("Iniciar Robots")
        
        # Widgets contenedores de secciones
        self.device_widget = QWidget()
        self.robots_widget = QWidget()
        self.robots_widget.setLayout(self.robots_layout)
        self.controller_widget = QWidget()
        self.controller_widget.setLayout(self.robots_controller_layout)
        # Widgets para configuración de robots
        self.r1_widget = QWidget()
        self.r1_adv_widget = QWidget()
        self.r2_widget = QWidget()
        self.r2_adv_widget = QWidget()
        # Widgets para controladores de robots
        self.r1_controller_widget = QWidget()
        self.r1_CD_widget = QWidget()
        self.r1_IK_widget = QWidget()
        self.r2_controller_widget = QWidget()
        self.r2_CD_widget = QWidget()
        self.r2_IK_widget = QWidget()
        
        # Layouts con pestañas
        self.r1_layout = QTabWidget()
        self.r2_layout = QTabWidget()
        self.r1_controller_layout = QTabWidget()
        self.r2_controller_layout = QTabWidget()
    
    def build_menu1_layout(self):
        """Construye el layout final del menú"""
        self.menu1_layout.addWidget(self.label_menu)
        self.menu1_layout.addWidget(self.device_widget)
        self.menu1_layout.addWidget(self.robots_widget)
        self.menu1_layout.addStretch()
        
    def build_menu2_layout(self):
        self.menu2_layout.addWidget(self.controller_widget)
        self.menu2_layout.addStretch()
        
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
        self.video_widget.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
        self.video_widget.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        
        # Configurar tamaño del dock widget (cuando está flotando)
        self.video_widget.resize(800, 600)  # Ancho x Alto cuando está flotante
        # También puedes establecer tamaño mínimo/máximo:
        # self.video_widget.setMinimumSize(400, 300)
        # self.video_widget.setMaximumSize(1200, 900)
        
        # Crear nodo suscriptor de cámara
        self.camera_node = CameraSubscriber(self.update_video)
        
        # Timer para procesar callbacks de ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.camera_node, timeout_sec=0.01))
        self.ros_timer.start(30)  # 30ms (~33 fps)
    
    
    
    def set_r1_menu(self):        
        
        self.r1_layout.addTab(self.r1_widget, "Robot 1")
        r1_buttons_layout = QGridLayout()
        r1_buttons2_widget = QWidget()
        r1_buttons2_layout = QHBoxLayout()
        r1_buttons2_widget.setLayout(r1_buttons2_layout)
        self.r1_type_input = QComboBox()
        self.r1_type_input.addItems(["ur5e", "ur5"])
        self.r1_mode_input = QComboBox()
        self.r1_mode_input.addItems(["Simulation","Real"])
        n = 40
        self.r1_x_input = QLineEdit(); self.r1_x_input.setText("0.0"); self.r1_x_input.setFixedWidth(n)
        self.r1_y_input = QLineEdit(); self.r1_y_input.setText("0.9"); self.r1_y_input.setFixedWidth(n)
        self.r1_z_input = QLineEdit(); self.r1_z_input.setText("0.0"); self.r1_z_input.setFixedWidth(n)
        self.r1_rx_input = QLineEdit(); self.r1_rx_input.setText("0.0"); self.r1_rx_input.setFixedWidth(n)
        self.r1_ry_input = QLineEdit(); self.r1_ry_input.setText("0.0"); self.r1_ry_input.setFixedWidth(n)
        self.r1_rz_input = QLineEdit(); self.r1_rz_input.setText("0.0"); self.r1_rz_input.setFixedWidth(n)
        
        r1_buttons2_layout.addWidget(QLabel("Type:"))
        r1_buttons2_layout.addWidget(self.r1_type_input)
        r1_buttons2_layout.addWidget(QLabel("Mode:"))
        r1_buttons2_layout.addWidget(self.r1_mode_input)
        r1_buttons_layout.addWidget(r1_buttons2_widget, 0, 0, 1, 6)
        
        # Labels alineados a la derecha para que parezcan estar junto a los inputs
        label_x = QLabel("X  "); label_x.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_x, 1, 0)
        r1_buttons_layout.addWidget(self.r1_x_input, 1, 1)
        
        label_y = QLabel("Y  "); label_y.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_y, 1, 2)
        r1_buttons_layout.addWidget(self.r1_y_input, 1, 3)
        
        label_z = QLabel("Z  "); label_z.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_z, 1, 4)
        r1_buttons_layout.addWidget(self.r1_z_input, 1, 5)
        
        label_rx = QLabel("RX"); label_rx.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_rx, 2, 0)
        r1_buttons_layout.addWidget(self.r1_rx_input, 2, 1)
        
        label_ry = QLabel("RY"); label_ry.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_ry, 2, 2)
        r1_buttons_layout.addWidget(self.r1_ry_input, 2, 3)
        
        label_rz = QLabel("RZ"); label_rz.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r1_buttons_layout.addWidget(label_rz, 2, 4)
        r1_buttons_layout.addWidget(self.r1_rz_input, 2, 5)
        
        self.r1_widget.setLayout(r1_buttons_layout)
        
        self.r1_layout.addTab(self.r1_adv_widget, "Robot 1 Advanced")
        r1_adv_buttons_layout = QVBoxLayout()
        r1_adv_buttons_layout.addWidget(QLabel("Robot IP"))
        self.r1_IP_input = QLineEdit(); self.r1_IP_input.setText("192.168.1.1")
        r1_adv_buttons_layout.addWidget(self.r1_IP_input)
        
        self.r1_adv_widget.setLayout(r1_adv_buttons_layout)
           
    def set_r2_menu(self):        
        
        self.r2_layout.addTab(self.r2_widget, "Robot 2")
        r2_buttons_layout = QGridLayout()
        r2_buttons2_widget = QWidget()
        r2_buttons2_layout = QHBoxLayout()
        r2_buttons2_widget.setLayout(r2_buttons2_layout)
        
        self.r2_type_input = QComboBox()
        self.r2_type_input.addItems(["ur5e", "ur5"])
        self.r2_mode_input = QComboBox()
        self.r2_mode_input.addItems(["Simulation","Real"])
        n = 40
        self.r2_x_input = QLineEdit(); self.r2_x_input.setText("0.0"); self.r2_x_input.setFixedWidth(n)
        self.r2_y_input = QLineEdit(); self.r2_y_input.setText("-0.9"); self.r2_y_input.setFixedWidth(n)
        self.r2_z_input = QLineEdit(); self.r2_z_input.setText("0.0"); self.r2_z_input.setFixedWidth(n)
        self.r2_rx_input = QLineEdit(); self.r2_rx_input.setText("0.0"); self.r2_rx_input.setFixedWidth(n)
        self.r2_ry_input = QLineEdit(); self.r2_ry_input.setText("0.0"); self.r2_ry_input.setFixedWidth(n)
        self.r2_rz_input = QLineEdit(); self.r2_rz_input.setText("0.0"); self.r2_rz_input.setFixedWidth(n)
        
        r2_buttons2_layout.addWidget(QLabel("Type:"))
        r2_buttons2_layout.addWidget(self.r2_type_input)
        r2_buttons2_layout.addWidget(QLabel("Mode:"))
        r2_buttons2_layout.addWidget(self.r2_mode_input)
        
        r2_buttons_layout.addWidget(r2_buttons2_widget, 0, 0, 1, 6)
        
        label_x = QLabel("X  "); label_x.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_x, 1, 0)
        r2_buttons_layout.addWidget(self.r2_x_input, 1, 1)
        
        label_y = QLabel("Y  "); label_y.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_y, 1, 2)
        r2_buttons_layout.addWidget(self.r2_y_input, 1, 3)
        
        label_z = QLabel("Z  "); label_z.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_z, 1, 4)
        r2_buttons_layout.addWidget(self.r2_z_input, 1, 5)
        
        label_rx = QLabel("RX"); label_rx.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_rx, 2, 0)
        r2_buttons_layout.addWidget(self.r2_rx_input, 2, 1)
        
        label_ry = QLabel("RY"); label_ry.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_ry, 2, 2)
        r2_buttons_layout.addWidget(self.r2_ry_input, 2, 3)
        
        label_rz = QLabel("RZ"); label_rz.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        r2_buttons_layout.addWidget(label_rz, 2, 4)
        r2_buttons_layout.addWidget(self.r2_rz_input, 2, 5)
        
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
        self.boton_iniciar_robots.clicked.connect(self.iniciar_robots)
        self.robots_layout.addWidget(self.boton_iniciar_robots)
    

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

    def _ensure_mapping_storage(self):
        if not hasattr(self, "_mapping_matrices"):
            self._mapping_matrices = {}

    def _create_mapping_matrix(
        self,
        *,
        key: str,
        title: str,
        row_labels,
        col_labels,
        initial_selection=None,
        invert_label: str = "+/-1",
        parent=None,
    ) -> QGroupBox:
        """Crea una matriz 3x3 (o NxM) de QRadioButton con selección única por columnas.

        La regla es la misma que en a.py: cada fila debe tener una columna distinta.
        Se usa `clicked` para evitar bucles al hacer `setChecked()` programáticamente.
        """
        self._ensure_mapping_storage()

        widget = QGroupBox(title, parent)
        layout = QGridLayout()
        widget.setLayout(layout)

        row_labels = list(row_labels)
        col_labels = list(col_labels)

        # Cabeceras
        layout.addWidget(QLabel(), 0, 0)
        layout.addWidget(QLabel(invert_label), 0, 1)
        for j, label in enumerate(col_labels):
            layout.addWidget(QLabel(label), 0, 2 + j)
        for i, label in enumerate(row_labels):
            layout.addWidget(QLabel(label), 2 + i, 0)

        invert_checks = [QCheckBox(widget) for _ in row_labels]
        for i, cb in enumerate(invert_checks):
            layout.addWidget(cb, 2 + i, 1)

        groups = [QButtonGroup(widget) for _ in row_labels]
        radios = [[QRadioButton(widget) for _ in col_labels] for _ in row_labels]

        if initial_selection is None:
            if len(col_labels) == 0:
                selection = [0 for _ in row_labels]
            else:
                selection = [i % len(col_labels) for i in range(len(row_labels))]
        else:
            selection = list(initial_selection)

        for row_idx in range(len(row_labels)):
            for col_idx in range(len(col_labels)):
                rb = radios[row_idx][col_idx]
                groups[row_idx].addButton(rb, col_idx)
                layout.addWidget(rb, 2 + row_idx, 2 + col_idx)
                if selection[row_idx] == col_idx:
                    rb.setChecked(True)
                rb.clicked.connect(
                    lambda checked, r=row_idx, c=col_idx, k=key: self._on_mapping_clicked(k, r, c)
                )

        self._mapping_matrices[key] = {
            "widget": widget,
            "layout": layout,
            "invert_checks": invert_checks,
            "groups": groups,
            "radios": radios,
            "selection": selection,
            "row_labels": row_labels,
            "col_labels": col_labels,
        }
        return widget

    def _on_mapping_clicked(self, key: str, fila_clicada: int, nueva_columna: int) -> None:
        data = self._mapping_matrices.get(key)
        if not data:
            return

        selection = data["selection"]
        radios = data["radios"]

        columna_antigua = selection[fila_clicada]
        if columna_antigua == nueva_columna:
            return

        fila_en_conflicto = -1
        for r, col in enumerate(selection):
            if r != fila_clicada and col == nueva_columna:
                fila_en_conflicto = r
                break

        if fila_en_conflicto != -1:
            # setChecked() NO dispara 'clicked' => sin recursión.
            radios[fila_en_conflicto][columna_antigua].setChecked(True)
            selection[fila_en_conflicto] = columna_antigua

        selection[fila_clicada] = nueva_columna
    
    def set_r1_controller(self):
        
        self.r1_controller_layout.addTab(self.r1_controller_widget,"Control")
        
        
        # Conectar señal para detectar cambios de pestaña
        self.r1_controller_layout.currentChanged.connect(self.on_r1_controller_tab_changed)
        
        #inputs de Controller
        self.r1_control_mode_input = QComboBox()
        self.r1_control_mode_input.addItems(self.control_mode_r1)
        r1_mode_layout = QGridLayout()
        r1_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r1_mode_layout.addWidget(self.r1_control_mode_input, 0, 1)
        self.r1_checkbox_safe_trayectory = QCheckBox("Save Trajectory")
        r1_mode_layout.addWidget(self.r1_checkbox_safe_trayectory, 1, 0)

        linear_widget = self._create_mapping_matrix(
            key="r1_linear",
            title="Movimiento Lineal",
            row_labels=["Joy X", "Joy Y", "Joy Z"],
            col_labels=["Robot X", "Robot Y", "Robot Z"],
            initial_selection=[0, 1, 2],
            parent=self.r1_controller_widget,
        )
        # Mantener nombres antiguos por compatibilidad
        self.r1_linear_invert_checks = self._mapping_matrices["r1_linear"]["invert_checks"]
        self.r1_linear_map_groups = self._mapping_matrices["r1_linear"]["groups"]
        self.r1_linear_map_radios = self._mapping_matrices["r1_linear"]["radios"]
        self.r1_linear_seleccion_actual = self._mapping_matrices["r1_linear"]["selection"]
        r1_mode_layout.addWidget(linear_widget, 5, 0, 1, 2)

        rot_widget = self._create_mapping_matrix(
            key="r1_rot",
            title="Movimiento Rotacional",
            row_labels=["Joy RX", "Joy RY", "Joy RZ"],
            col_labels=["Robot RX", "Robot RY", "Robot RZ"],
            initial_selection=[0, 1, 2],
            parent=self.r1_controller_widget,
        )
        self.r1_rot_invert_checks = self._mapping_matrices["r1_rot"]["invert_checks"]
        self.r1_rot_map_groups = self._mapping_matrices["r1_rot"]["groups"]
        self.r1_rot_map_radios = self._mapping_matrices["r1_rot"]["radios"]
        self.r1_rot_seleccion_actual = self._mapping_matrices["r1_rot"]["selection"]
        r1_mode_layout.addWidget(rot_widget, 6, 0, 1, 2)
        
        
        
        self.r1_controller_widget.setLayout(r1_mode_layout)
        
        
        
        
        #inputs de Cartesian

    def set_r2_controller(self):
        self.r2_controller_layout.addTab(self.r2_controller_widget, "Controller")
        #inputs de Controller
        self.r2_control_mode_input = QComboBox()
        self.r2_control_mode_input.addItems(self.control_mode_r2)
        r2_mode_layout = QGridLayout()
        r2_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r2_mode_layout.addWidget(self.r2_control_mode_input, 0, 1)

        linear_widget = self._create_mapping_matrix(
            key="r2_linear",
            title="Movimiento Lineal",
            row_labels=["Joy X", "Joy Y", "Joy Z"],
            col_labels=["Robot X", "Robot Y", "Robot Z"],
            initial_selection=[0, 1, 2],
            parent=self.r2_controller_widget,
        )
        self.r2_linear_invert_checks = self._mapping_matrices["r2_linear"]["invert_checks"]
        self.r2_linear_map_groups = self._mapping_matrices["r2_linear"]["groups"]
        self.r2_linear_map_radios = self._mapping_matrices["r2_linear"]["radios"]
        self.r2_linear_seleccion_actual = self._mapping_matrices["r2_linear"]["selection"]
        r2_mode_layout.addWidget(linear_widget, 5, 0, 1, 2)

        rot_widget = self._create_mapping_matrix(
            key="r2_rot",
            title="Movimiento Rotacional",
            row_labels=["Joy RX", "Joy RY", "Joy RZ"],
            col_labels=["Robot RX", "Robot RY", "Robot RZ"],
            initial_selection=[0, 1, 2],
            parent=self.r2_controller_widget,
        )
        self.r2_rot_invert_checks = self._mapping_matrices["r2_rot"]["invert_checks"]
        self.r2_rot_map_groups = self._mapping_matrices["r2_rot"]["groups"]
        self.r2_rot_map_radios = self._mapping_matrices["r2_rot"]["radios"]
        self.r2_rot_seleccion_actual = self._mapping_matrices["r2_rot"]["selection"]
        r2_mode_layout.addWidget(rot_widget, 6, 0, 1, 2)

        self.r2_controller_widget.setLayout(r2_mode_layout)
        
        
    
    
    def set_controller_menu(self):
        self.robots_controller_layout.addWidget(self.r1_controller_layout)
        self.robots_controller_layout.addWidget(self.r2_controller_layout)
        self.set_r1_controller()
        self.set_r2_controller()
    
    
    def set_r1_joint_control(self):
        self.r1_CD_widget_layout = QGridLayout()
        self.r1_CD_widget.setLayout(self.r1_CD_widget_layout)
        self.r1_CD_widget_layout.addWidget(QLabel("Robot 1 Joints Control Placeholder"), 0, 0,1, 2)
        self.r1_q0 = QSlider(Qt.Horizontal); self.r1_q0.setMinimum(-180); self.r1_q0.setMaximum(180); self.r1_q0.setValue(0)
        self.r1_q1 = QSlider(Qt.Horizontal); self.r1_q1.setMinimum(-180); self.r1_q1.setMaximum(180); self.r1_q1.setValue(0)
        self.r1_q2 = QSlider(Qt.Horizontal); self.r1_q2.setMinimum(-180); self.r1_q2.setMaximum(180); self.r1_q2.setValue(0)
        self.r1_q3 = QSlider(Qt.Horizontal); self.r1_q3.setMinimum(-180); self.r1_q3.setMaximum(180); self.r1_q3.setValue(0)
        self.r1_q4 = QSlider(Qt.Horizontal); self.r1_q4.setMinimum(-180); self.r1_q4.setMaximum(180); self.r1_q4.setValue(0)
        self.r1_q5 = QSlider(Qt.Horizontal); self.r1_q5.setMinimum(-180); self.r1_q5.setMaximum(180); self.r1_q5.setValue(0)
        
        #inputs de Joints
        
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 1"), 1, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q0, 1, 1)
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 2"), 2, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q1, 2, 1)
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 3"), 3, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q2, 3, 1)
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 4"), 4, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q3, 4, 1)
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 5"), 5, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q4, 5, 1)
        self.r1_CD_widget_layout.addWidget(QLabel("Joint 6"), 6, 0)
        self.r1_CD_widget_layout.addWidget(self.r1_q5, 6, 1)
        
    def set_r2_joint_control(self):
        self.r2_CD_widget_layout = QGridLayout()
        self.r2_CD_widget.setLayout(self.r2_CD_widget_layout)
        self.r2_CD_widget_layout.addWidget(QLabel("Robot 1 Joints Control Placeholder"), 0, 0,1, 2)
        self.r2_q0 = QSlider(Qt.Horizontal); self.r2_q0.setMinimum(-180); self.r2_q0.setMaximum(180); self.r2_q0.setValue(0)
        self.r2_q1 = QSlider(Qt.Horizontal); self.r2_q1.setMinimum(-180); self.r2_q1.setMaximum(180); self.r2_q1.setValue(0)
        self.r2_q2 = QSlider(Qt.Horizontal); self.r2_q2.setMinimum(-180); self.r2_q2.setMaximum(180); self.r2_q2.setValue(0)
        self.r2_q3 = QSlider(Qt.Horizontal); self.r2_q3.setMinimum(-180); self.r2_q3.setMaximum(180); self.r2_q3.setValue(0)
        self.r2_q4 = QSlider(Qt.Horizontal); self.r2_q4.setMinimum(-180); self.r2_q4.setMaximum(180); self.r2_q4.setValue(0)
        self.r2_q5 = QSlider(Qt.Horizontal); self.r2_q5.setMinimum(-180); self.r2_q5.setMaximum(180); self.r2_q5.setValue(0)
        
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 1"), 1, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q0, 1, 1)
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 2"), 2, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q1, 2, 1)
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 3"), 3, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q2, 3, 1)
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 4"), 4, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q3, 4, 1)
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 5"), 5, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q4, 5, 1)
        self.r2_CD_widget_layout.addWidget(QLabel("Joint 6"), 6, 0)
        self.r2_CD_widget_layout.addWidget(self.r2_q5, 6, 1)
     
    
    def set_joint_control(self):
        self.set_r1_joint_control()
        self.set_r2_joint_control()        
        self.menu3_layout.addWidget(self.r1_CD_widget)
        self.menu3_layout.addWidget(self.r2_CD_widget)
        self.menu3_layout.addStretch()
        

    def set_r1_ik_control(self):
        pass
    def set_r2_ik_control(self):
        pass
    def set_ik_control(self):
        self.set_r1_ik_control()
        self.set_r2_ik_control()
        self.menu4_layout.addWidget(self.r1_IK_widget)
        self.menu4_layout.addWidget(self.r2_IK_widget)
        self.menu4_layout.addStretch()
            
    
    
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
            f'r1_type:={self.r1_type_input.currentText()}',
            f'r2_type:={self.r2_type_input.currentText()}',
            f'use_fake_hardware_r1:={"true" if self.r1_mode_input.currentText().lower() == "simulation" else "false"}',
            f'use_fake_hardware_r2:={"true" if self.r2_mode_input.currentText().lower() == "simulation" else "false"}',
            # Posiciones y orientaciones desde inputs   
            f'r1_x_pos:={self.r1_x_input.text()}',
            f'r1_y_pos:={self.r1_y_input.text()}',
            f'r1_z_pos:={self.r1_z_input.text()}',
            f'r1_rot_x:={self.r1_rx_input.text()}',
            f'r1_rot_y:={self.r1_ry_input.text()}',
            f'r1_rot_z:={self.r1_rz_input.text()}',
            f'r2_x_pos:={self.r2_x_input.text()}',
            f'r2_y_pos:={self.r2_y_input.text()}',
            f'r2_z_pos:={self.r2_z_input.text()}',
            f'r2_rot_x:={self.r2_rx_input.text()}',
            f'r2_rot_y:={self.r2_ry_input.text()}',
            f'r2_rot_z:={self.r2_rz_input.text()}',
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

    # Cargar y aplicar style.qss desde la carpeta config del paquete ur5_panel
    # Workspace: /home/david/tesis_ws
    # Paquete: ur5_panel
    # Carpeta config: /home/david/tesis_ws/src/ur5_simulation/ur5_interfaz/ur5_panel/config/style.qss
    qss_path = os.path.join(os.path.dirname(__file__), '../config/style.qss')
    qss_path = os.path.abspath(qss_path)
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