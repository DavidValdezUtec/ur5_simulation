from PyQt5.QtWidgets import QWidget, QGridLayout, QDoubleSpinBox, QLabel, QSlider
from PyQt5.QtCore import Qt
import math

class RobotConfigWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.in_degrees = True  # True = grados, False = radianes
        self.joints = []
        self.sliders = []
        
        # Layout principal
        main_layout = QGridLayout(self)
        
        # Switch de cambio de unidad de grados a radianes
        switch_layout = QGridLayout()
        switch_layout.addWidget(QLabel("Degrees"), 0, 0)
        switch_layout.addWidget(QLabel("Radians"), 0, 1)
        self.unit_switch = QSlider()
        self.unit_switch.setOrientation(Qt.Horizontal)
        self.unit_switch.setRange(0, 1)
        self.unit_switch.setValue(0)  # Default a grados
        self.unit_switch.sliderMoved.connect(self._on_unit_switch_changed)
        switch_layout.addWidget(self.unit_switch, 1, 0, 1, 2)
        
        main_layout.addLayout(switch_layout, 0, 0, 1, 3)
        
        # Widgets de los valores articulares
        for i in range(6):
            label = QLabel(f"Joint {i+1}:")
            # Slider
            slider = QSlider()
            slider.setOrientation(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, idx=i: self._sync_spin_from_slider(idx))
            
            # SpinBox
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)  # Definir límites físicos
            spin.setSuffix(" °")           # Ayuda visual
            spin.setDecimals(2)            # Precisión
            spin.valueChanged.connect(lambda value, idx=i: self._sync_slider_from_spin(idx))
            
            main_layout.addWidget(label, i+1, 0)
            main_layout.addWidget(slider, i+1, 1)
            main_layout.addWidget(spin, i+1, 2)
            
            self.joints.append(spin)
            self.sliders.append(slider)

    def _on_unit_switch_changed(self):
        """Cambia el sistema de unidades cuando se mueve el switch"""
        self.in_degrees = self.unit_switch.value() == 0
        
        # Actualizar sufijo y rango de los spinboxes
        for spin, slider in zip(self.joints, self.sliders):
            current_value = spin.value()
            
            if self.in_degrees:
                # Convertir a grados (si estaba en radianes)
                spin.setSuffix(" °")
                spin.setRange(-180.0, 180.0)
                # No necesita conversión porque ya tiene grados guardados
            else:
                # Convertir a radianes
                spin.setSuffix(" rad")
                spin.setRange(-math.pi, math.pi)
                # Convertir valor actual a radianes
                spin.blockSignals(True)
                spin.setValue(math.radians(current_value))
                spin.blockSignals(False)

    def _sync_spin_from_slider(self, idx):
        """Sincroniza el spinbox con el slider"""
        self.joints[idx].blockSignals(True)
        self.joints[idx].setValue(self.sliders[idx].value())
        self.joints[idx].blockSignals(False)

    def _sync_slider_from_spin(self, idx):
        """Sincroniza el slider con el spinbox"""
        self.sliders[idx].blockSignals(True)
        self.sliders[idx].setValue(int(self.joints[idx].value()))
        self.sliders[idx].blockSignals(False)

    def get_values(self):
        """Retorna una lista con los valores actuales siempre en radianes"""
        values = []
        for spin in self.joints:
            value = spin.value()
            if self.in_degrees:
                # Convertir de grados a radianes
                value = math.radians(value)
            values.append(value)
        return values

class window_example(QWidget):
    def __init__(self):
        super().__init__()
        self.robot_config = RobotConfigWidget()
        layout = QGridLayout(self)
        layout.addWidget(self.robot_config, 0, 0)

if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)
    window = window_example()
    window.show()
    sys.exit(app.exec_())
        