from PyQt5.QtWidgets import *

from ur5_panel import config_store


class UIControllerConfigMixin:
    """
    Config de controlador (pestaña 2). Por robot: modo (Teleoperation/
    Trayectoria), tipo de controlador (QP/Sliding/Impedancia), las 2
    matrices de mapeo de mapping_matrix.py, y los botones Start/Stop que
    llaman a panel.py:start_controller/stop_controller (esos arman el
    'ros2 run ur5_controller controller_node' con todo r{id}_control_config).
    El layout cambia dinamicamente segun el modo elegido: ver
    widget_controller_changed mas abajo.
    """

    def set_r1_controller(self):
        """Arma el tab 'Control' (modo, tipo, mapeos, botones Start/Stop)
        y 'Advanced' (las 2 matrices de mapeo) de Robot 1. Al final llama
        cambiar_widget_controller (conecta el combo de modo a
        widget_controller_changed) y setup_control_config_connections
        (conecta todo lo demas a r1_control_config). El tipo de robot ("ur")
        ya no se guarda aca: panel.py:start_controller lo lee directo de
        r1_config["ur_type"] al armar el comando."""
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
        self.r1_q_target = QLineEdit(); self.r1_q_target.setText(self.r1_control_config["q_target"][1:-1])
        #self.r1_A_input = QLineEdit(); self.r1_A_input.setText("1.0")
        


        r1_mode_layout = QGridLayout()
        r1_adv_mode_layout = QGridLayout()

        self.r1_checkbox_safe_trayectory = QCheckBox("Save Trajectory")
        self.r1_checkbox_safe_trayectory.setChecked(True)


        botones_layout = QHBoxLayout()
        botones_widget = QWidget(); botones_widget.setLayout(botones_layout)
        boton_start_controller = QPushButton("Start Controller")
        boton_detener_controller = QPushButton("Stop Controller")
        boton_home = QPushButton("Home")
        
        boton_start_controller.clicked.connect(lambda: self.start_controller("r1"))
        boton_detener_controller.clicked.connect(lambda: self.stop_controller("r1"))
        boton_home.clicked.connect(lambda: self.home_controller("r1"))

        botones_layout.addWidget(boton_start_controller)
        botones_layout.addWidget(boton_detener_controller)


        r1_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r1_mode_layout.addWidget(self.r1_control_mode_input, 0, 1)
        r1_mode_layout.addWidget(boton_home, 3, 0)
        r1_mode_layout.addWidget(self.r1_q_target, 3, 1)

        if self.r1_control_mode_input.currentText() == "Trayectoria":
            r1_mode_layout.addWidget(self.r1_trayectories, 2, 1)
            
        else:
            r1_mode_layout.addWidget(QLabel(""), 2, 0); r1_mode_layout.addWidget(QLabel(""), 2, 1) # Espaciadores para mantener el diseño
            #r1_mode_layout.addWidget(QLabel(""), 3, 0); r1_mode_layout.addWidget(QLabel(""), 3, 1) # Espaciadores para mantener el diseño


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
        """Igual que set_r1_controller pero para Robot 2."""
        self.r2_controller_layout.addTab(self.r2_controller_widget, "Controller")
        self.r2_controller_layout.addTab(self.r2_controller_adv_widget, "Advanced")
        #inputs de Controller
        self.r2_control_mode_input = QComboBox()
        self.r2_control_mode_input.addItems(self.control_mode_r2)
        self.r2_trayectories = QComboBox()
        self.r2_trayectories.addItems(["Curva Helicoidal", "Linea Recta", "Circunferencia"])
        self.r2_controles = QComboBox(); self.r2_controles.addItems(["Optimizador", "Sliding", "Impedancia"])
        self.r2_q_target = QLineEdit(); self.r2_q_target.setText(self.r2_control_config["q_target"][1:-1])



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
        boton_home = QPushButton("Home")
        boton_home.clicked.connect(lambda: self.home_controller("r2"))

        botones_layout.addWidget(boton_start_controller)
        botones_layout.addWidget(boton_stop_controller)

        r2_mode_layout.addWidget(QLabel("Control Mode:"), 0, 0)
        r2_mode_layout.addWidget(self.r2_control_mode_input, 0, 1)
        r2_mode_layout.addWidget(boton_home, 3, 0)
        r2_mode_layout.addWidget(self.r2_q_target, 3, 1)

        if self.r2_control_mode_input.currentText() == "Trayectoria" :
            #r2_mode_layout.addWidget(QLabel("Trajectory Type:"), 2, 0)
            r2_mode_layout.addWidget(self.r2_trayectories, 2, 1)
            
        else:
            r2_mode_layout.addWidget(QLabel(""), 2, 0); r2_mode_layout.addWidget(QLabel(""), 2, 1) # Espaciadores para mantener el diseño
            #r2_mode_layout.addWidget(QLabel(""), 3, 0); r2_mode_layout.addWidget(QLabel(""), 3, 1) # Espaciadores para mantener el diseño

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
        """Conecta los combos (modo/trayectoria/tipo de controlador) y las
        2 matrices de mapeo del robot indicado a update_control_config /
        on_linear_mapping_changed / on_rot_mapping_changed / on_*_invert_changed,
        que son quienes finalmente escriben en r{id}_control_config."""
        getattr(self, f"{robot_id}_control_mode_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id, 'geomagic', "true" if text == "Teleoperation" else "false")
        )
        getattr(self, f"{robot_id}_trayectories").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id, 'traj_mode', 1.0 if text=="Curva Helicoidal" else (2.0 if text == "Linea Recta" else 3) )
        )
        getattr(self,f"{robot_id}_controles").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id,'controller_type', "QP" if text=="Optimizador" else ("SLD" if text == "Sliding" else "IMP"))
        )
        getattr(self,f"{robot_id}_q_target").textChanged.connect(
            lambda text, r_id=robot_id: self.update_control_config(r_id,'q_target', f"[{text}]")
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
        """Escribe una clave en r{id}_control_config y persiste a disco.
        r{id}_control_config ES r{id}_config["controller"] (mismo objeto,
        ver initialize_config_variables en ui_menu.py), asi que escribir aca
        ya modifica r{id}_config; solo falta volcarlo a
        ~/.ros/ur5_panel/config.json, igual que hace update_config()."""
        config = getattr(self, f"{robot_id}_control_config")
        if key in config:
            config[key] = value
            print(f"{robot_id.upper()} Control Config updated: {key} = {value}") # Opcional: para depuración
            config_store.save_config({"r1": self.r1_config, "r2": self.r2_config})
        pass

    def on_linear_mapping_changed(self, robot_id, row_idx, col_idx):
        """Tras un click en la matriz lineal (ya resuelto por _on_mapping_clicked),
        vuelca la seleccion completa [Joy X/Y/Z -> Robot X/Y/Z] a
        map_x/map_y/map_z en r{id}_control_config."""
        config = getattr(self, f"{robot_id}_control_config")
        linear_data = self._mapping_matrices.get(f"{robot_id}_linear")
        if linear_data:
            lista = ["x","y","z"]
            # Actualizar todos los mapeos
            for i, col in enumerate(linear_data["selection"]):
                config[f"map_{lista[i]}"] = str(col)
            config_store.save_config({"r1": self.r1_config, "r2": self.r2_config})
            print(f"Linear mapping updated for {robot_id}: map_x={linear_data['selection'][0]}, map_y={linear_data['selection'][1]}, map_z={linear_data['selection'][2]}")

    def on_rot_mapping_changed(self, robot_id, row_idx, col_idx):
        """Igual que on_linear_mapping_changed pero para la matriz rotacional
        (Joy RX/RY/RZ -> Robot RX/RY/RZ), escribe map_roll/map_pitch/map_yaw."""
        config = getattr(self, f"{robot_id}_control_config")
        rot_data = self._mapping_matrices.get(f"{robot_id}_rot")
        if rot_data:
            lista = ["roll", "pitch", "yaw"]
            # Actualizar todos los mapeos
            for i, col in enumerate(rot_data["selection"]):
                config[f"map_{lista[i]}"] = str(col)
            config_store.save_config({"r1": self.r1_config, "r2": self.r2_config})
            print(f"Rotational mapping updated for {robot_id}: map_roll={rot_data['selection'][0]}, map_pitch={rot_data['selection'][1]}, map_yaw={rot_data['selection'][2]}")

    def on_linear_invert_changed(self, robot_id, idx, state):
        """Checkbox de invertir signo (columna '-1' de la matriz lineal):
        recalcula sign_x/sign_y/sign_z (+1.0 o -1.0) en r{id}_control_config."""
        linear_data = self._mapping_matrices.get(f"{robot_id}_linear")
        if linear_data:
            keys = ["sign_x", "sign_y", "sign_z"]
            # Actualizar todos los signos
            for i, checkbox in enumerate(linear_data["invert_checks"]):
                sign = -1.0 if checkbox.isChecked() else 1.0
                self.update_control_config(robot_id, keys[i], str(sign))
            print(f"Linear signs updated for {robot_id}: x={getattr(self, f'{robot_id}_control_config')['sign_x']}, y={getattr(self, f'{robot_id}_control_config')['sign_y']}, z={getattr(self, f'{robot_id}_control_config')['sign_z']}")

    def on_rot_invert_changed(self, robot_id, idx, state):
        """Igual que on_linear_invert_changed pero para sign_roll/sign_pitch/sign_yaw."""
        rot_data = self._mapping_matrices.get(f"{robot_id}_rot")
        if rot_data:
            keys = ["sign_roll", "sign_pitch", "sign_yaw"]
            # Actualizar todos los signos
            for i, checkbox in enumerate(rot_data["invert_checks"]):
                sign = -1.0 if checkbox.isChecked() else 1.0
                self.update_control_config(robot_id, keys[i], str(sign))
            print(f"Rotational signs updated for {robot_id}: roll={getattr(self, f'{robot_id}_control_config')['sign_roll']}, pitch={getattr(self, f'{robot_id}_control_config')['sign_pitch']}, yaw={getattr(self, f'{robot_id}_control_config')['sign_yaw']}")



    def cambiar_widget_controller(self, robot_id):
        """Solo conecta el combo de modo a widget_controller_changed; el
        trabajo real de reacomodar widgets esta ahi."""
        getattr(self, f"{robot_id}_control_mode_input").currentTextChanged.connect(
            lambda text, r_id=robot_id: self.widget_controller_changed(r_id, text) #text = Teleoperation o Trayectoria
        )

    def widget_controller_changed(self, robot_id, new_mode):
        """Cuando cambia el modo (Teleoperation/Trayectoria), quita lo que
        haya en las filas 2-3 del grid del controlador y pone lo que
        corresponde al nuevo modo: en Teleoperation, espaciadores vacios; en
        Trayectoria, el combo de tipo de trayectoria + el campo Q Target.
        Depende de que set_r1_controller/set_r2_controller hayan puesto algo
        en esas posiciones exactas del grid (2,1)/(3,0)/(3,1) al construirlo."""
        if new_mode == self.control_mode_r1[0]: #new_mode = "Teleoperation"
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(2, 1).widget().setParent(None) # Elimina el widget actual en esa posición
            #getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 0).widget().setParent(None) # Elimina el widget de "Save Trajectory"
            #getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 1).widget().setParent(None) # Elimina el widget de botones
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 2, 0) # Espaciador para mantener el diseño
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 2, 1) # Espaciador para mantener el diseño
            # getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 3, 0) # Espaciador para mantener el diseño
            # getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QLabel(""), 3, 1) # Espaciador para mantener el diseño

        elif new_mode == self.control_mode_r1[1]:
            getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(2, 1).widget().setParent(None) # Elimina el widget actual en esa posición
            getattr(self, f"{robot_id}_controller_widget").layout().addWidget(getattr(self, f"{robot_id}_trayectories"), 2, 1)
            # getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 0).widget().setParent(None) # Elimina el widget de "Save Trajectory"
            # getattr(self, f"{robot_id}_controller_widget").layout().itemAtPosition(3, 1).widget().setParent(None) # Elimina el widget de botones
            # getattr(self, f"{robot_id}_controller_widget").layout().addWidget(QPushButton("Home"), 3, 0) # Espaciador para mantener el diseño
            # getattr(self, f"{robot_id}_controller_widget").layout().addWidget(getattr(self, f"{robot_id}_q_target"), 3, 1)

    def set_controller_menu(self):
        """Punto de entrada de la sección: arma set_r1_controller/
        set_r2_controller y los cuelga en la pestaña 2 (llamado desde
        setup_ui, después de setup_menu())."""
        self.robots_controller_layout.addWidget(self.r1_controller_layout)
        self.robots_controller_layout.addWidget(self.r2_controller_layout)
        self.set_r1_controller()
        self.set_r2_controller()

    def home_controller(self, robot_id):
        """Activa scaled_joint_trajectory_controller (desactivando el
        controlador de articulaciones activo) y publica q_target como
        trayectoria de home. Corre en un hilo para no congelar la GUI."""
        import threading
        threading.Thread(target=self._home_controller_worker, args=(robot_id,), daemon=True).start()

    def _home_controller_worker(self, robot_id):
        import subprocess
        home_controller = "scaled_joint_trajectory_controller"
        previos = self.controller_changer(robot_id, home_controller)
        if previos is None:
            return

        try:
            print(f"[{robot_id}] Enviando trayectoria de home...")
            q_target = getattr(self, f"{robot_id}_control_config")["q_target"]
            # Los joints del controlador llevan el tf_prefix del robot (p.ej. r1_shoulder_pan_joint)
            prefix = getattr(self, f"{robot_id}_config")["tf_prefix"]
            joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            joint_names = ", ".join(f'"{prefix}{j}"' for j in joints)
            goal = (f'{{trajectory: {{joint_names: [{joint_names}], '
                    f'points: [{{positions: {q_target}, time_from_start: {{sec: 5, nanosec: 0}}}}]}}}}')
            # Se usa la accion (no el topico) porque send_goal bloquea hasta que la
            # trayectoria termina: asi sabemos cuando es seguro volver al controlador inicial
            result = subprocess.run(['ros2', 'action', 'send_goal',
                                     f'/{robot_id}/{home_controller}/follow_joint_trajectory',
                                     'control_msgs/action/FollowJointTrajectory', goal],
                                    capture_output=True, text=True, timeout=60)
            if "SUCCEEDED" in result.stdout:
                print(f"[{robot_id}] Home alcanzado: {q_target}")
            else:
                print(f"[{robot_id}] Error en trayectoria de home: {result.stdout.strip()} {result.stderr.strip()}")
        except subprocess.TimeoutExpired:
            print(f"[{robot_id}] Timeout esperando la trayectoria de home")
        finally:
            # Regresar al controlador que estaba activo antes del home
            if previos and previos != [home_controller]:
                self.controller_changer(robot_id, previos[0])

    def controller_changer(self, robot_id, new_controller_type):
        """Activa new_controller_type en /{robot_id}/controller_manager,
        desactivando los controladores de articulaciones que esten activos.
        Devuelve la lista de controladores de articulaciones que estaban
        activos antes del cambio (para poder regresar a ellos), o None si
        el cambio fallo. Bloquea: llamarla desde un hilo, no desde la GUI."""
        import subprocess, re
        cm = f'/{robot_id}/controller_manager'
        joint_controllers = ["scaled_joint_trajectory_controller", "joint_trajectory_controller",
                             "forward_position_controller", "forward_velocity_controller",
                             "passthrough_trajectory_controller"]

        # Identificar controlador activo (la salida trae codigos de color ANSI)
        result = subprocess.run(['ros2', 'control', 'list_controllers', '-c', cm],
                                capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[{robot_id}] Error al listar controladores: {result.stderr.strip()}")
            return None
        estados = {}
        for linea in re.sub(r'\x1b\[[0-9;]*m', '', result.stdout).splitlines():
            campos = linea.split()
            if len(campos) >= 3:
                estados[campos[0]] = campos[-1]
        activos = [c for c in joint_controllers if estados.get(c) == "active"]
        print(f"[{robot_id}] Controladores de articulaciones activos: {activos}")

        if new_controller_type not in estados:
            print(f"[{robot_id}] {new_controller_type} no esta cargado en {cm}")
            return None

        if activos == [new_controller_type]:
            return activos

        print(f"[{robot_id}] Cambiando a {new_controller_type}...")
        # Los nombres van SIN namespace: el controller_manager ya es /{robot_id}/...
        cmd = ['ros2', 'control', 'switch_controllers', '-c', cm, '--strict',
               '--activate', new_controller_type]
        desactivar = [c for c in activos if c != new_controller_type]
        if desactivar:
            cmd += ['--deactivate'] + desactivar
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"[{robot_id}] Salida del cambio de controlador: {result.stdout.strip()}")
        if result.returncode != 0:
            print(f"[{robot_id}] Error al cambiar de controlador: {result.stderr.strip()}")
            return None
        return activos
