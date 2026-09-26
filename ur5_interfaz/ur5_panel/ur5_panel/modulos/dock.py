"""Barra superior del panel (QToolBar fija arriba, siempre visible):
dispositivos, estado de los robots, configuracion/lanzamiento y STOP.

Dock solo dibuja: no conoce procesos ni ROS. panel.py le dice que mostrar
(set_haptic/set_camara/set_robot_state/set_stop_info) y conecta sus
botones (boton_buscar, boton_config, boton_iniciar, boton_stop)."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QToolBar,
    QWidget,
)

LED_SIZE = 16

# Colores de los LEDs
COLOR_OK = "#2ecc71"        # verde
COLOR_FALLA = "#e74c3c"     # rojo
COLOR_GRIS = "#7f8c8d"
COLOR_AMARILLO = "#f1c40f"

# Estado de un robot (ver robot_monitor.py) -> color del LED
ROBOT_STATE_COLORS = {
    "detenido": COLOR_GRIS,
    "lanzado": COLOR_AMARILLO,
    "listo": COLOR_OK,
    "error": COLOR_FALLA,
}

# Modo del robot (config_store.MODE_LABELS) -> texto corto junto al LED
MODE_SHORT = {"fake": "Sim", "real": "Real", "gazebo": "Gazebo"}


def _led():
    led = QLabel()
    led.setFixedSize(LED_SIZE, LED_SIZE)
    return led


def _set_led(led, color, tooltip=""):
    led.setStyleSheet(
        f"background-color: {color}; border-radius: {LED_SIZE // 2}px;")
    led.setToolTip(tooltip)


class Dock(QToolBar):
    def __init__(self, parent=None):
        super().__init__("Barra superior", parent)
        self.setObjectName("dock_superior")
        # Fija: ni movible, ni flotante, ni ocultable desde el menu
        # contextual de la ventana (el STOP tiene que estar siempre a mano).
        self.setMovable(False)
        self.setFloatable(False)
        self.toggleViewAction().setVisible(False)
        self.setContextMenuPolicy(Qt.PreventContextMenu)

        self._set_devices()
        self.addSeparator()
        self._set_robots()
        self.addSeparator()
        self._set_stop()

    # ------------------------------------------------------------ secciones
    def _grupo(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(6, 0, 6, 0)
        layout.setSpacing(6)
        self.addWidget(widget)
        return layout

    def _set_devices(self):
        """LEDs de hapticos/camara + boton de busqueda + checkbox de feedback."""
        layout = self._grupo()
        self.led_haptic1 = _led()
        self.led_haptic2 = _led()
        self.led_camara = _led()
        for texto, led in (("Háptico 1", self.led_haptic1),
                           ("Háptico 2", self.led_haptic2),
                           ("Cámara", self.led_camara)):
            layout.addWidget(QLabel(texto))
            layout.addWidget(led)
        self.set_haptic(1, False)
        self.set_haptic(2, False)
        self.set_camara(False)

        self.boton_buscar = QPushButton("Buscar dispositivos")
        layout.addWidget(self.boton_buscar)

        self.feedback_checkbox = QCheckBox("Feedback háptico")
        self.feedback_checkbox.setChecked(True)
        layout.addWidget(self.feedback_checkbox)

    def _set_robots(self):
        """LED de estado + modo de cada robot, y botones de config/lanzamiento."""
        layout = self._grupo()
        self.robot_leds = {}
        self.robot_modes = {}
        for robot_id in ("r1", "r2"):
            layout.addWidget(QLabel(robot_id.upper()))
            self.robot_leds[robot_id] = _led()
            layout.addWidget(self.robot_leds[robot_id])
            self.robot_modes[robot_id] = QLabel("")
            self.robot_modes[robot_id].setObjectName("dock_robot_mode")
            layout.addWidget(self.robot_modes[robot_id])
            self.set_robot_state(robot_id, "detenido", "Detenido")

        self.boton_config = QPushButton("Configurar robots")
        self.boton_iniciar = QPushButton("Iniciar robots")
        layout.addWidget(self.boton_config)
        layout.addWidget(self.boton_iniciar)

    def _set_stop(self):
        """STOP a la derecha, separado del resto por un espacio flexible."""
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.addWidget(spacer)

        layout = self._grupo()
        self.stop_info = QLabel("")
        self.stop_info.setObjectName("dock_stop_info")
        layout.addWidget(self.stop_info)

        self.boton_stop = QPushButton("STOP")
        self.boton_stop.setObjectName("boton_stop")
        self.boton_stop.setToolTip(
            "Detiene ambos robots: corta los controladores y los deja quietos "
            "en su posición actual (atajo: S).\n"
            "No reemplaza el paro de emergencia físico del robot.")
        layout.addWidget(self.boton_stop)

    # ------------------------------------------------------------- setters
    def set_haptic(self, numero, conectado):
        led = self.led_haptic1 if numero == 1 else self.led_haptic2
        _set_led(led, COLOR_OK if conectado else COLOR_FALLA,
                 f"Háptico {numero}: {'conectado' if conectado else 'no encontrado'}")

    def set_camara(self, conectada):
        _set_led(self.led_camara, COLOR_OK if conectada else COLOR_FALLA,
                 f"Cámara: {'conectada' if conectada else 'no encontrada'}")

    def set_robot_state(self, robot_id, estado, detalle, modo=None):
        """estado: 'detenido' | 'lanzado' | 'listo' | 'error' (gris/amarillo/
        verde/rojo). detalle va al tooltip. modo: 'fake'/'real'/'gazebo' con
        el que se lanzo el robot (None = no se muestra)."""
        _set_led(self.robot_leds[robot_id], ROBOT_STATE_COLORS[estado],
                 f"{robot_id.upper()}: {detalle}")
        self.robot_modes[robot_id].setText(f"({MODE_SHORT[modo]})" if modo in MODE_SHORT else "")

    def set_stop_info(self, texto):
        """Resultado del ultimo STOP, junto al boton."""
        self.stop_info.setText(texto)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    dock = Dock()
    dock.set_haptic(1, True)
    dock.set_robot_state("r1", "listo", "Listo", "gazebo")
    dock.set_robot_state("r2", "lanzado", "Esperando /joint_states", "real")
    dock.show()
    sys.exit(app.exec_())
