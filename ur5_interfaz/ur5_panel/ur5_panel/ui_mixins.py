from PyQt5.QtCore import Qt, QSize
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QRadioButton,
                             QVBoxLayout, QGridLayout, QSizePolicy, 
                             QPushButton, QLabel, QTabWidget, QLineEdit,
                             QComboBox, QHBoxLayout, QGroupBox, QCheckBox, QDockWidget,
                             QSlider, QScrollArea,QButtonGroup)
from PyQt5 import QtCore

class UIMixin:
    """
    Mixin para la configuración de la interfaz de usuario de InterfazRviz.
    Contiene los métodos para crear y organizar los menús.
    """
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
        self.r2_config = {
            "ur_type": "ur5e",
            "robot_ip": "192.168.10.103",
            "description_package": "ur5_description",
            "tf_prefix": "r2_",
            "runtime_config_package": "ur5_bringup",
            "controllers_file": ["ur_controllers_", "r2", ".yaml"],
            "kinematics_params_file": ["/home/david/my_robot_calibration_", "ur5e", ".yaml"],
            "use_fake_hardware": "true",  # Empezamos en modo simulación
            "launch_dashboard_client": "true",
            "launch_rviz": "false",
            "reverse_port": "50011",
            "script_sender_port": "50012",
            "trajectory_port": "50013",
            "script_command_port": "50014",
            "pos_x": "0.0",
            "pos_y": "-0.9",
            "pos_z": "0.0",
            "rot_x": "0.0",
            "rot_y": "0.0",
            "rot_z": "0.0",
        }
        
        self.r1_control_config = {
            "control_topic": "/forward_position_controller/commands",
            "ur":"ur5e",
            "nmspace":"r1",
            "geomagic":"true",
            "geomagic_topic":"/phantom1/pose",
            "csv_log_enable":"true",
            "traj_mode":"1",
            "q_target":"[-1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0]",
            "map_x":"0",
            "map_y":"1",
            "map_z":"2",
            "sign_x":"1.0",
            "sign_y":"1.0",
            "sign_z":"1.0",
            "map_roll":"0.0",
            "map_pitch":"1.0",
            "map_yaw":"2.0",
            "sign_roll":"1.0",
            "sign_pitch":"1.0",
            "sign_yaw":"1.0",            
            "controller_type":"QP",
            "lambda":"[20.0,20.0,20.0,20.0,20.0,20.0]",
            "k":"[60.0,60.0,60.0,60.0,60.0,60.0,60.0]",
            "alpha":"10.0" 
        }
        self.r2_control_config = {
            "control_topic": "/forward_position_controller/commands",
            "ur":"ur5e",
            "nmspace":"r2",
            "geomagic":"true",
            "geomagic_topic":"/phantom2/pose",
            "csv_log_enable":"true",
            "traj_mode":"1",
            "q_target":"[1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0]",
            "map_x":"0",
            "map_y":"1",
            "map_z":"2",
            "sign_x":"1.0",
            "sign_y":"1.0",
            "sign_z":"1.0",
            "map_roll":"0.0",
            "map_pitch":"1.0",
            "map_yaw":"2.0",
            "sign_roll":"1.0",
            "sign_pitch":"1.0",
            "sign_yaw":"1.0",                
            "controller_type":"QP",
            "lambda":"[20.0,20.0,20.0,20.0,20.0,20.0]",
            "k":"[60.0,60.0,60.0,60.0,60.0,60.0,60.0]",
            "alpha":"10.0"     
        }
        
        
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
        self.r1_controller_adv_widget = QWidget()
        self.r1_CD_widget = QWidget()
        self.r1_IK_widget = QWidget()
        self.r2_controller_widget = QWidget()
        self.r2_controller_adv_widget = QWidget()
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
        self.r1_TCP_input = QLineEdit(); self.r1_TCP_input.setText("50002")
        r1_adv_buttons_layout.addWidget(self.r1_IP_input)
        r1_adv_buttons_layout.addWidget(self.r1_TCP_input)
        
        self.r1_adv_widget.setLayout(r1_adv_buttons_layout)
        self.setup_config_connections('r1')
    
    
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
        r2_adv_buttons_layout.addWidget(QLabel("Robot IP"))
        self.r2_IP_input = QLineEdit(); self.r2_IP_input.setText("192.168.1.2")
        self.r2_TCP_input = QLineEdit(); self.r2_TCP_input.setText("50012")
        r2_adv_buttons_layout.addWidget(self.r2_IP_input)
        r2_adv_buttons_layout.addWidget(self.r2_TCP_input)
        self.r2_adv_widget.setLayout(r2_adv_buttons_layout)
        self.setup_config_connections('r2')
           
    def setup_config_connections(self, robot_id):
        """Conecta los widgets de configuración de un robot a los métodos de actualización."""
        config = getattr(self, f"{robot_id}_config")
        
        getattr(self, f"{robot_id}_type_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'ur_type', text)
        )
        getattr(self, f"{robot_id}_mode_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'use_fake_hardware', 'true' if text == 'Simulation' else 'false')
        )
        getattr(self, f"{robot_id}_x_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'pos_x', text)
        )
        getattr(self, f"{robot_id}_y_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'pos_y', text)
        )
        getattr(self, f"{robot_id}_z_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'pos_z', text)
        )
        getattr(self, f"{robot_id}_rx_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'rot_x', text)
        )
        getattr(self, f"{robot_id}_ry_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'rot_y', text)
        )
        getattr(self, f"{robot_id}_rz_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'rot_z', text)
        )
        getattr(self, f"{robot_id}_IP_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'robot_ip', text)
        )
        getattr(self, f"{robot_id}_TCP_input").textChanged.connect(
            lambda text, r_id=robot_id: self.update_config(r_id, 'script_sender_port', text)
        )

    def update_config(self, robot_id, key, value):
        """Actualiza una clave en el diccionario de configuración del robot especificado."""
        config = getattr(self, f"{robot_id}_config")
        if key in config:
            config[key] = value
            print(f"{robot_id.upper()} Config updated: {key} = {value}") # Opcional: para depuración
        else:
            print(f"Warning: La clave '{key}' no existe en {robot_id}_config.")
    
    def set_robot_menu(self):
        self.robots_layout.addWidget(self.r1_layout)
        self.robots_layout.addWidget(self.r2_layout)
        self.set_r1_menu()
        self.set_r2_menu()
        self.boton_iniciar_robots.clicked.connect(self.iniciar_robots)
        self.robots_layout.addWidget(self.boton_iniciar_robots)

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
        invert_label: str = "-1",
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
        self.r1_control_config["ur"] = self.r1_config["ur_type"]  # Asegurar que el tipo de robot esté sincronizado
        self.r1_controller_layout.addTab(self.r1_controller_widget,"Control")
        self.r1_controller_layout.addTab(self.r1_controller_adv_widget,"Advanced")
        
        
        # Conectar señal para detectar cambios de pestaña
        self.r1_controller_layout.currentChanged.connect(self.on_r1_controller_tab_changed)
        
        #inputs de Controller
        self.r1_control_mode_input = QComboBox()
        self.r1_control_mode_input.addItems(self.control_mode_r1)
        self.r1_trayectories = QComboBox()
        self.r1_trayectories.addItems(["Curva Helicoidal", "Linea Recta", "Circunferencia"])
        self.r1_controles = QComboBox(); self.r1_controles.addItems(["Optimizador", "Sliding", "Impedancia"])
        self.r1_q_target = QLineEdit(); self.r1_q_target.setText("-1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0")
        
        
        
        r1_mode_layout = QGridLayout()
        r1_adv_mode_layout = QGridLayout()
        
        self.r1_checkbox_safe_trayectory = QCheckBox("Save Trajectory")
        self.r1_checkbox_safe_trayectory.setChecked(True)
        
        
        botones_layout = QHBoxLayout()
        botones_widget = QWidget(); botones_widget.setLayout(botones_layout)
        boton_start_controller = QPushButton("Start Controller")
        boton_detener_controller = QPushButton("Stop Controller")
        boton_start_controller.clicked.connect(lambda: self.start_controller("r1"))
        boton_detener_controller.clicked.connect(lambda: self.stop_controller("r1"))
        
        botones_layout.addWidget(boton_start_controller)
        botones_layout.addWidget(boton_detener_controller)
        
        
        r1_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r1_mode_layout.addWidget(self.r1_control_mode_input, 0, 1)        
            
        if self.r1_control_mode_input.currentText() == "Trayectoria":
            r1_mode_layout.addWidget(QLabel("Trajectory Type:"), 2, 0)
            r1_mode_layout.addWidget(self.r1_trayectories, 2, 1)
            r1_mode_layout.addWidget(QLabel("Q Target:"), 3, 0)
            r1_mode_layout.addWidget(self.r1_q_target, 3, 1)
        else:
            r1_mode_layout.addWidget(QLabel(""), 2, 0); r1_mode_layout.addWidget(QLabel(""), 2, 1) # Espaciadores para mantener el diseño
            r1_mode_layout.addWidget(QLabel(""), 3, 0); r1_mode_layout.addWidget(QLabel(""), 3, 1) # Espaciadores para mantener el diseño
            
            
        r1_mode_layout.addWidget(self.r1_checkbox_safe_trayectory, 1, 0)
        r1_mode_layout.addWidget(self.r1_controles, 1, 1)
        r1_mode_layout.addWidget(botones_widget, 5, 0, 1, 2)
        
        

        self.r1_linear_widget = self._create_mapping_matrix(
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
        
        self.r1_rot_widget = self._create_mapping_matrix(
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
        
        r1_adv_mode_layout.addWidget(self.r1_linear_widget, 5, 0, 1, 2)
        r1_adv_mode_layout.addWidget(self.r1_rot_widget, 6, 0, 1, 2)
        
        
        
        self.r1_controller_widget.setLayout(r1_mode_layout)
        self.r1_controller_adv_widget.setLayout(r1_adv_mode_layout)
        
        self.cambiar_widget_controller("r1")
        self.setup_control_config_connections("r1")
        
        
        #inputs de Cartesian

    def set_r2_controller(self):
        self.r2_control_config["ur"] = self.r2_config["ur_type"]  # Asegurar que el tipo de robot esté sincronizado
        self.r2_controller_layout.addTab(self.r2_controller_widget, "Controller")
        self.r2_controller_layout.addTab(self.r2_controller_adv_widget, "Advanced")
        #inputs de Controller
        self.r2_control_mode_input = QComboBox()
        self.r2_control_mode_input.addItems(self.control_mode_r2)
        self.r2_trayectories = QComboBox()
        self.r2_trayectories.addItems(["Curva Helicoidal", "Linea Recta", "Circunferencia"])
        self.r2_controles = QComboBox(); self.r2_controles.addItems(["Optimizador", "Sliding", "Impedancia"])
        self.r2_q_target = QLineEdit(); self.r2_q_target.setText("1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0")
        
        
        
        r2_mode_layout = QGridLayout()
        r2_adv_mode_layout = QGridLayout()
        
        self.r2_checkbox_safe_trayectory = QCheckBox("Save Trajectory")
        self.r2_checkbox_safe_trayectory.setChecked(True)
        
        botones_layout = QHBoxLayout()
        botones_widget = QWidget(); botones_widget.setLayout(botones_layout)
        boton_start_controller = QPushButton("Start Controller")
        boton_start_controller.clicked.connect(lambda: self.start_controller("r2"))
        boton_stop_controller = QPushButton("Stop Controller")
        boton_stop_controller.clicked.connect(lambda: self.stop_controller("r2"))
        
        botones_layout.addWidget(boton_start_controller)
        botones_layout.addWidget(boton_stop_controller)
        
        r2_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r2_mode_layout.addWidget(self.r2_control_mode_input, 0, 1) 
        
        if self.r2_control_mode_input.currentText() == "Trayectoria" :
            r2_mode_layout.addWidget(QLabel("Trajectory Type:"), 2, 0)
            r2_mode_layout.addWidget(self.r2_trayectories, 2, 1) 
            r2_mode_layout.addWidget(QLabel("Q Target:"), 3, 0)
            r2_mode_layout.addWidget(self.r2_q_target, 3, 1)
        else:            
            r2_mode_layout.addWidget(QLabel(""), 2, 0); r2_mode_layout.addWidget(QLabel(""), 2, 1) # Espaciadores para mantener el diseño
            r2_mode_layout.addWidget(QLabel(""), 3, 0); r2_mode_layout.addWidget(QLabel(""), 3, 1) # Espaciadores para mantener el diseño
            
        r2_mode_layout.addWidget(self.r2_checkbox_safe_trayectory, 1, 0)
        r2_mode_layout.addWidget(self.r2_controles, 1, 1)
        r2_mode_layout.addWidget(botones_widget, 5, 0, 1, 2)
        
        

        self.r2_linear_widget = self._create_mapping_matrix(
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
        

        self.r2_rot_widget = self._create_mapping_matrix(
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
            
           
        r2_adv_mode_layout.addWidget(self.r2_linear_widget, 5, 0, 1, 2)
        r2_adv_mode_layout.addWidget(self.r2_rot_widget, 6, 0, 1, 2)

        self.r2_controller_widget.setLayout(r2_mode_layout)
        self.r2_controller_adv_widget.setLayout(r2_adv_mode_layout)
        self.cambiar_widget_controller("r2")
        self.setup_control_config_connections("r2")
        
    def setup_control_config_connections(self, robot_id):
        getattr(self, f"{robot_id}_control_mode_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id, 'geomagic', "true" if text == "Teleoperation" else "false")
        )
        getattr(self, f"{robot_id}_trayectories").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id, 'traj_mode', 1.0 if text=="Curva Helicoidal" else (2.0 if text == "Linea Recta" else 3) )
        )
        getattr(self,f"{robot_id}_controles").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id,'controller_type', "QP" if text=="Optimizador" else ("SLD" if text == "Sliding" else "IMP"))
        )
        
        linear_data = self._mapping_matrices.get(f"{robot_id}_linear")
        if linear_data:
            for i, checkbox in enumerate(linear_data["invert_checks"]):
                checkbox.stateChanged.connect(
                    lambda state, r_id=robot_id, idx=i: self.on_linear_invert_changed(r_id, idx, state)
                )
            
            for i, radio in enumerate(linear_data["radios"]):
                for j, rb in enumerate(radio):
                    rb.clicked.connect(
                        lambda checked, r_id=robot_id, row=i, col=j: self.on_linear_mapping_changed(r_id, row, col)
                    )
        
        # Conectar señales de mapeo rotacional
        rot_data = self._mapping_matrices.get(f"{robot_id}_rot")
        if rot_data:
            for i, checkbox in enumerate(rot_data["invert_checks"]):
                checkbox.stateChanged.connect(
                    lambda state, r_id=robot_id, idx=i: self.on_rot_invert_changed(r_id, idx, state)
            )
            for i, radio in enumerate(rot_data["radios"]):
                for j, rb in enumerate(radio):
                    rb.clicked.connect(
                        lambda checked, r_id=robot_id, row=i, col=j: self.on_rot_mapping_changed(r_id, row, col)
                    )
        
        
        
    def update_control_config(self, robot_id, key, value):
        config = getattr(self, f"{robot_id}_control_config")
        if key in config:
            config[key] = value
            print(f"{robot_id.upper()} Control Config updated: {key} = {value}") # Opcional: para depuración
        pass
    
    def on_linear_mapping_changed(self, robot_id, row_idx, col_idx):
        config = getattr(self, f"{robot_id}_control_config")
        linear_data = self._mapping_matrices.get(f"{robot_id}_linear")
        if linear_data:
            lista = ["x","y","z"]
            # Actualizar todos los mapeos
            for i, col in enumerate(linear_data["selection"]):
                config[f"map_{lista[i]}"] = str(col)
            print(f"Linear mapping updated for {robot_id}: map_x={linear_data['selection'][0]}, map_y={linear_data['selection'][1]}, map_z={linear_data['selection'][2]}")
    
    def on_rot_mapping_changed(self, robot_id, row_idx, col_idx):
        config = getattr(self, f"{robot_id}_control_config")
        rot_data = self._mapping_matrices.get(f"{robot_id}_rot")
        if rot_data:
            lista = ["roll", "pitch", "yaw"]
            # Actualizar todos los mapeos
            for i, col in enumerate(rot_data["selection"]):
                config[f"map_{lista[i]}"] = str(col)
            print(f"Rotational mapping updated for {robot_id}: map_roll={rot_data['selection'][0]}, map_pitch={rot_data['selection'][1]}, map_yaw={rot_data['selection'][2]}")
    
    def on_linear_invert_changed(self, robot_id, idx, state):
        linear_data = self._mapping_matrices.get(f"{robot_id}_linear")
        if linear_data:
            keys = ["sign_x", "sign_y", "sign_z"]
            # Actualizar todos los signos
            for i, checkbox in enumerate(linear_data["invert_checks"]):
                sign = -1.0 if checkbox.isChecked() else 1.0
                self.update_control_config(robot_id, keys[i], str(sign))
            print(f"Linear signs updated for {robot_id}: x={getattr(self, f'{robot_id}_control_config')['sign_x']}, y={getattr(self, f'{robot_id}_control_config')['sign_y']}, z={getattr(self, f'{robot_id}_control_config')['sign_z']}")

    def on_rot_invert_changed(self, robot_id, idx, state):
        rot_data = self._mapping_matrices.get(f"{robot_id}_rot")
        if rot_data:
            keys = ["sign_roll", "sign_pitch", "sign_yaw"]
            # Actualizar todos los signos
            for i, checkbox in enumerate(rot_data["invert_checks"]):
                sign = -1.0 if checkbox.isChecked() else 1.0
                self.update_control_config(robot_id, keys[i], str(sign))
            print(f"Rotational signs updated for {robot_id}: roll={getattr(self, f'{robot_id}_control_config')['sign_roll']}, pitch={getattr(self, f'{robot_id}_control_config')['sign_pitch']}, yaw={getattr(self, f'{robot_id}_control_config')['sign_yaw']}")
    
    
    
    def cambiar_widget_controller(self, robot_id):
        getattr(self, f"{robot_id}_control_mode_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.widget_controller_changed(r_id, text) #text = Teleoperation o Trayectoria
        )
    
    def widget_controller_changed(self, robot_id, new_mode):        
        if new_mode == self.control_mode_r1[0]: #new_mode = "Teleoperation"
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(2, 1).widget().setParent(None) # Elimina el widget actual en esa posición
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 0).widget().setParent(None) # Elimina el widget de "Save Trajectory"
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 1).widget().setParent(None) # Elimina el widget de botones
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 2, 0) # Espaciador para mantener el diseño
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 2, 1) # Espaciador para mantener el diseño
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 3, 0) # Espaciador para mantener el diseño
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 3, 1) # Espaciador para mantener el diseño
            
        elif new_mode == self.control_mode_r1[1]:
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(2, 1).widget().setParent(None) # Elimina el widget actual en esa posición
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(getattr(self, f"{robot_id}_trayectories"), 2, 1)
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 0).widget().setParent(None) # Elimina el widget de "Save Trajectory"
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 1).widget().setParent(None) # Elimina el widget de botones
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel("Q Target"), 3, 0) # Espaciador para mantener el diseño
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(getattr(self, f"{robot_id}_q_target"), 3, 1)
            
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
