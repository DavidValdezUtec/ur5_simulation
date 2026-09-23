from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *

from ur5_panel import config_store


class UIRobotConfigMixin:
    """
    Config de robots (pose/tipo/modo/IP/puerto) por r1 y r2, mas la seccion
    de dispositivos hapticos de la pestaña 1. Estos paneles leen/escriben
    r{id}_config (persistido en ~/.ros/ur5_panel/config.json via
    config_store), que modulos/robots_launch.py traduce a config.json de
    ur5e_bringup al lanzar. set_r1_menu/set_r2_menu son practicamente
    identicos (uno por robot); si se tocan, hay que replicar el cambio en
    el otro a mano.
    """

    def set_devices_menu(self):
        """Sección 'dispositivos' de la pestaña 1: boton de busqueda +
        2 LEDs de estado haptico. Dispara una busqueda inicial al arrancar
        (self.buscar_dispositivos() al final), ademas de la que dispara el
        boton. La logica de busqueda/lanzamiento vive en panel.py e
        internamente delega a modulos/haptic.py y modulos/camera.py."""
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
        """Panel 'Robot 1' (tab basico: tipo/modo/pose XYZ+RPY) + panel
        'Robot 1 Advanced' (IP y puerto TCP). Al final conecta cada input
        a update_config('r1', ...) via setup_config_connections."""
        self.r1_layout.addTab(self.r1_widget, "Robot 1")
        r1_buttons_layout = QGridLayout()
        r1_buttons2_widget = QWidget()
        r1_buttons2_layout = QHBoxLayout()
        r1_buttons2_widget.setLayout(r1_buttons2_layout)
        self.r1_type_input = QComboBox()
        self.r1_type_input.addItems(["ur5e", "ur5"])
        self.r1_type_input.setCurrentText(self.r1_config["ur_type"])
        self.r1_mode_input = QComboBox()
        self.r1_mode_input.addItems(["Simulation","Real"])
        self.r1_mode_input.setCurrentText("Simulation" if self.r1_config["use_fake_hardware"] == "true" else "Real")
        n = 50
        self.r1_x_input = QLineEdit(); self.r1_x_input.setText(self.r1_config["pos_x"]); self.r1_x_input.setFixedWidth(n)
        self.r1_y_input = QLineEdit(); self.r1_y_input.setText(self.r1_config["pos_y"]); self.r1_y_input.setFixedWidth(n)
        self.r1_z_input = QLineEdit(); self.r1_z_input.setText(self.r1_config["pos_z"]); self.r1_z_input.setFixedWidth(n)
        self.r1_rx_input = QLineEdit(); self.r1_rx_input.setText(self.r1_config["rot_x"]); self.r1_rx_input.setFixedWidth(n)
        self.r1_ry_input = QLineEdit(); self.r1_ry_input.setText(self.r1_config["rot_y"]); self.r1_ry_input.setFixedWidth(n)
        self.r1_rz_input = QLineEdit(); self.r1_rz_input.setText(self.r1_config["rot_z"]); self.r1_rz_input.setFixedWidth(n)

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
        self.r1_IP_input = QLineEdit(); self.r1_IP_input.setText(self.r1_config["robot_ip"])
        self.r1_TCP_input = QLineEdit(); self.r1_TCP_input.setText(self.r1_config["script_sender_port"])
        r1_adv_buttons_layout.addWidget(self.r1_IP_input)
        r1_adv_buttons_layout.addWidget(self.r1_TCP_input)

        self.r1_adv_widget.setLayout(r1_adv_buttons_layout)
        self.setup_config_connections('r1')


    def set_r2_menu(self):
        """Igual que set_r1_menu pero para Robot 2 (misma estructura de
        widgets, prefijo r2_ en vez de r1_)."""
        self.r2_layout.addTab(self.r2_widget, "Robot 2")
        r2_buttons_layout = QGridLayout()
        r2_buttons2_widget = QWidget()
        r2_buttons2_layout = QHBoxLayout()
        r2_buttons2_widget.setLayout(r2_buttons2_layout)

        self.r2_type_input = QComboBox()
        self.r2_type_input.addItems(["ur5e", "ur5"])
        self.r2_type_input.setCurrentText(self.r2_config["ur_type"])
        self.r2_mode_input = QComboBox()
        self.r2_mode_input.addItems(["Simulation","Real"])
        self.r2_mode_input.setCurrentText("Simulation" if self.r2_config["use_fake_hardware"] == "true" else "Real")
        n = 50
        self.r2_x_input = QLineEdit(); self.r2_x_input.setText(self.r2_config["pos_x"]); self.r2_x_input.setFixedWidth(n)
        self.r2_y_input = QLineEdit(); self.r2_y_input.setText(self.r2_config["pos_y"]); self.r2_y_input.setFixedWidth(n)
        self.r2_z_input = QLineEdit(); self.r2_z_input.setText(self.r2_config["pos_z"]); self.r2_z_input.setFixedWidth(n)
        self.r2_rx_input = QLineEdit(); self.r2_rx_input.setText(self.r2_config["rot_x"]); self.r2_rx_input.setFixedWidth(n)
        self.r2_ry_input = QLineEdit(); self.r2_ry_input.setText(self.r2_config["rot_y"]); self.r2_ry_input.setFixedWidth(n)
        self.r2_rz_input = QLineEdit(); self.r2_rz_input.setText(self.r2_config["rot_z"]); self.r2_rz_input.setFixedWidth(n)

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
        self.r2_IP_input = QLineEdit(); self.r2_IP_input.setText(self.r2_config["robot_ip"])
        self.r2_TCP_input = QLineEdit(); self.r2_TCP_input.setText(self.r2_config["script_sender_port"])
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
        """Actualiza una clave en el diccionario de configuración del robot especificado
        y persiste el cambio en ~/.ros/ur5_panel/config.json."""
        config = getattr(self, f"{robot_id}_config")
        if key in config:
            config[key] = value
            print(f"{robot_id.upper()} Config updated: {key} = {value}") # Opcional: para depuración
            config_store.save_config({"r1": self.r1_config, "r2": self.r2_config})
        else:
            print(f"Warning: La clave '{key}' no existe en {robot_id}_config.")

    def set_robot_menu(self):
        """Ensambla la sección 'robots' de la pestaña 1: paneles r1+r2,
        checkbox de feedback háptico (ver nota en panel.py:lanzar_robots,
        ese checkbox ya no tiene efecto real) y el botón que dispara
        panel.py:iniciar_robots() -> modulos/robots_launch.py."""
        self.robots_layout.addWidget(self.r1_layout)
        self.robots_layout.addWidget(self.r2_layout)
        self.set_r1_menu()
        self.set_r2_menu()

        self.feedback_checkbox = QCheckBox("Iniciar feedback haptico")
        self.feedback_checkbox.setChecked(True)
        self.robots_layout.addWidget(self.feedback_checkbox)

        self.boton_iniciar_robots.clicked.connect(self.iniciar_robots)
        self.robots_layout.addWidget(self.boton_iniciar_robots)
