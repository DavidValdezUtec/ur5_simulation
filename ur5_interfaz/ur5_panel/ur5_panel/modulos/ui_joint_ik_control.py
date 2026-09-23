from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *


class UIJointIkControlMixin:
    """
    Control de Joints (pestaña 3) e IK (pestaña 4). Ambas siguen siendo
    placeholders: los sliders de joints no estan conectados a ningun
    publisher/servicio todavia, y el control IK (set_r{1,2}_ik_control)
    esta completamente vacio.
    """

    def set_r1_joint_control(self):
        """Sliders (-180..180) para los 6 joints de Robot 1. Placeholder:
        todavia no publican nada, solo mueven el slider en la UI."""
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
        """Igual que set_r1_joint_control pero para Robot 2."""
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
        """Punto de entrada de la pestaña 3 (llamado desde setup_ui, aparte
        de setup_menu()): arma los sliders de r1/r2 y los cuelga ahi."""
        self.set_r1_joint_control()
        self.set_r2_joint_control()
        self.menu3_layout.addWidget(self.r1_CD_widget)
        self.menu3_layout.addWidget(self.r2_CD_widget)
        self.menu3_layout.addStretch()


    def set_r1_ik_control(self):
        """TODO: sin implementar. Pestaña IK de Robot 1."""
        pass
    def set_r2_ik_control(self):
        """TODO: sin implementar. Pestaña IK de Robot 2."""
        pass
    def set_ik_control(self):
        """Punto de entrada de la pestaña 4. NOTA: a diferencia de las demas
        pestañas, esto no se llama desde setup_ui() todavia (buscar
        'set_ik_control' en panel.py) -- la pestaña 4 queda vacia salvo por
        el contenedor que crea create_menu4_structure."""
        self.set_r1_ik_control()
        self.set_r2_ik_control()
        self.menu4_layout.addWidget(self.r1_IK_widget)
        self.menu4_layout.addWidget(self.r2_IK_widget)
        self.menu4_layout.addStretch()
