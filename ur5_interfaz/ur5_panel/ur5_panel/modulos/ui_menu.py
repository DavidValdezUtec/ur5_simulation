from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import QtCore

from ur5_panel import config_store


class UIMenuMixin:
    """
    Punto de entrada del menu lateral (llamado 1 vez desde InterfazRviz.setup_ui)
    y la estructura de las 4 pestañas (tabs) del QTabWidget lateral:
    dispositivos+robots, controlador, joints, IK.

    Mixin: InterfazRviz hereda de esta clase junto con las otras 4 mixins de
    ui_*.py/mapping_matrix.py. Se mantiene el patron mixin (no composicion)
    porque todo el menu es un solo arbol de widgets con referencias cruzadas
    reales entre secciones (ver docstrings de cada metodo).
    """

    def setup_menu(self):
        """Punto de entrada: arma el menú lateral completo, en orden:
        1) variables de estado/config, 
        2) las 4 pestañas vacías,
        3) los widgets que van dentro de cada pestaña, 
        4) layout de las pestañas 1 y 2 (las pestañas 3 y 4 las arma set_joint_control/
        set_ik_control, llamadas aparte desde setup_ui)."""
        self.initialize_config_variables()
        self.create_menu_structure()
        self.initialize_menu_widgets()
        self.build_menu1_layout()
        self.build_menu2_layout()

    def initialize_config_variables(self):
        """Inicializa todo el estado no-visual del panel, antes de crear
        ningún widget: flags de dispositivos, y r1_config/r2_config (pose/red/
        tipo de cada robot, mas los parametros de control anidados en
        r{id}_config["controller"]). r{id}_control_config (usado por
        ui_controller_config.py y panel.py:start_controller) apunta al MISMO
        dict que r{id}_config["controller"] -- no es una copia -- para que
        editar la config de control desde la UI persista junto con la del
        robot en el mismo ~/.ros/ur5_panel/config.json (via config_store)."""
        # Estados de dispositivos (se leen en panel.py; ver nota en
        # modulos/haptic.py: la fuente de verdad real de haptic1_ready/
        # haptic2_ready ya es self.haptic.haptic1_ready/haptic2_ready)
        self.haptic1_ready = False
        self.haptic2_ready = False
        self.camera_ready = False

        # Configuracion de robots: se carga desde ~/.ros/ur5_panel/config.json
        # (creado la primera vez a partir de la plantilla del paquete). Los
        # dicts de abajo son solo el respaldo si no hay plantilla ni archivo
        # de usuario, y completan claves nuevas que un archivo viejo no tenga.
        default_robot_configs = {
            "r1": {
                "ur_type": "ur5e",
                "robot_ip": "192.168.10.104",
                "description_package": "ur5_description",
                "runtime_config_package": "ur5_bringup",
                "controllers_file": ["ur_controllers_", "r1", ".yaml"],
                "kinematics_params_file": ["/home/david/my_robot_calibration_", "ur5e", ".yaml"],
                "mode": "fake",  # Empezamos en modo simulación (ver config_store.MODE_LABELS)
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
                # "ur" (tipo de robot) y "nmspace" (namespace) NO se guardan
                # aca: son datos del robot, no del controlador. Se derivan al
                # vuelo de r{id}_config["ur_type"] y de robot_id en
                # panel.py:start_controller, para no tener 2 copias de lo
                # mismo que puedan desincronizarse.
                "controller":{
                    "control_topic": "/forward_position_controller/commands",
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
            },
            "r2": {
                "ur_type": "ur5e",
                "robot_ip": "192.168.10.103",
                "description_package": "ur5_description",
                "runtime_config_package": "ur5_bringup",
                "controllers_file": ["ur_controllers_", "r2", ".yaml"],
                "kinematics_params_file": ["/home/david/my_robot_calibration_", "ur5e", ".yaml"],
                "mode": "fake",  # Empezamos en modo simulación (ver config_store.MODE_LABELS)
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
                "controller":{
                    "control_topic": "/forward_position_controller/commands",
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
            },
        }
        loaded_robot_configs = config_store.load_config(default_robot_configs)
        self.r1_config = loaded_robot_configs["r1"]
        self.r2_config = loaded_robot_configs["r2"]

        # r{id}_control_config: MISMO objeto que r{id}_config["controller"]
        # (no una copia) -- cualquier escritura en r{id}_control_config
        # (desde update_control_config/on_*_mapping_changed/on_*_invert_changed
        # en ui_controller_config.py) modifica directamente r{id}_config, que
        # es el dict que persiste config_store.save_config(). "ur"/"nmspace"
        # no viven aca: se leen de r{id}_config["ur_type"] y del robot_id
        # directamente donde se arma el comando en panel.py:start_controller.
        self.r1_control_config = self.r1_config["controller"]
        self.r2_control_config = self.r2_config["controller"]


        # Modos de control disponibles
        self.control_mode_r1 = ["Teleoperation", "Trayectoria"]
        self.control_mode_r2 = ["Teleoperation", "Trayectoria"]

    def create_menu_structure(self):
        """Crea el QTabWidget lateral (menu_general) y sus 4 pestañas vacías
        (una llamada a create_menuN_structure por pestaña). Cada pestaña usa
        un ícono ya cargado por panel.py:cargar_iconos() antes de setup_menu()."""
        self.menu_general = QTabWidget()
        self.menu_general.setObjectName("menu_general")
        self.menu_general.setTabPosition(QTabWidget.West)
        self.create_menu1_structure()
        self.create_menu2_structure()
        self.create_menu3_structure()
        self.create_menu4_structure()
        pass


    def create_menu1_structure(self):

        """Pestaña 1: la única con scroll. Su contenido anterior (LEDs de
        dispositivos, config de r1/r2, botón Iniciar Robots) pasó a la barra
        superior (modulos/dock.py) y a la ventana 'Configuración de robots';
        se mantiene por ahora para los futuros modos teleoperación/desarrollo.
        Deja listo robots_controller_layout, que llena set_controller_menu()
        (ui_controller_config.py)."""
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
        """Pestaña 2 (Controlador): contenedor donde set_controller_menu()
        agrega los tabs de control de r1 y r2 (Teleoperation/Trayectoria)."""
        self.menu2_layout = QVBoxLayout()
        self.menu2_widget = QWidget()
        self.menu2_widget.setLayout(self.menu2_layout)
        self.menu_general.addTab(self.menu2_widget, self.icon_menu2,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass

    def create_menu3_structure(self):
        """Pestaña 3 (Joints): contenedor donde set_joint_control() agrega
        los sliders de cada joint de r1/r2 (son placeholders, no publican
        nada todavia)."""
        self.menu3_layout = QVBoxLayout()
        self.menu3_widget = QWidget()
        self.menu3_widget.setLayout(self.menu3_layout)
        self.menu_general.addTab(self.menu3_widget, self.icon_menu3,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass

    def create_menu4_structure(self):
        """Pestaña 4 (IK): contenedor para set_ik_control(); set_r1_ik_control/
        set_r2_ik_control todavia estan vacios (pass), pestaña sin implementar."""
        self.menu4_layout = QVBoxLayout()
        self.menu4_widget = QWidget()
        self.menu4_widget.setLayout(self.menu4_layout)
        self.menu_general.addTab(self.menu4_widget, self.icon_menu4,"")
        self.menu_general.setIconSize(QtCore.QSize(40, 40))
        pass



    def initialize_menu_widgets(self):
        """Crea (sin llenar layouts todavia) los widgets/contenedores que usan
        las secciones de mas abajo: botones principales, los QWidget vacios
        que luego reciben layout en set_devices_menu/set_r{1,2}_menu/
        set_r{1,2}_controller, y los QTabWidget que agrupan "basico" vs
        "avanzado" de cada robot y su controlador."""
        # Widgets principales
        self.label_menu = QLabel("Menu")
        self.boton_salir = QPushButton("Salir")
        self.boton_salir.clicked.connect(self.close)

        # "Buscar dispositivos" e "Iniciar robots" estan en la barra
        # superior (modulos/dock.py), no en el menu.

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
        """Apila en la pestaña 1 (de arriba a abajo): título, sección de
        dispositivos (device_widget, la llena set_devices_menu) y sección de
        robots (robots_widget, la llena set_robot_menu)."""
        self.menu1_layout.addWidget(self.label_menu)
        self.menu1_layout.addWidget(self.device_widget)
        self.menu1_layout.addWidget(self.robots_widget)
        self.menu1_layout.addStretch()

    def build_menu2_layout(self):
        """Pone en la pestaña 2 el contenedor de controladores (lo llena
        set_controller_menu, llamado por separado desde setup_ui)."""
        self.menu2_layout.addWidget(self.controller_widget)
        self.menu2_layout.addStretch()
