"""Graficas del modo Desarrollo (debajo de RViz, ver modos.py), con
pyqtgraph. Tres pestañas, cada una con su propio selector de robot:

  - Joint States: posicion articular actual (q_meas) y deseada (q_cmd).
  - Cartesian Pose: posicion (x, y, z) y orientacion (roll, pitch, yaw) de
    la punta de la herramienta, actual y deseada.
  - Control Effort: esfuerzo de control (u_control) por articulacion.

Fuente: /rN/controller_state (ur5_controller_msgs/ControllerState), que
publica controller_node (la misma fila que escribe en el CSV). Sin
controller_node corriendo no hay datos.

Las suscripciones van en el nodo de RobotMonitor, que se bombea con un
QTimer en el hilo de Qt: los callbacks solo guardan la muestra en un
buffer circular por robot, y un QTimer redibuja la pestaña visible a
~30 fps (no en cada mensaje).
"""
import time

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from ur5_controller_msgs.msg import ControllerState

ROBOT_IDS = ("r1", "r2")
JOINTS = ("Shoulder pan", "Shoulder lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3")

REDIBUJO_MS = 33            # ~30 fps
VENTANA_INICIAL_S = 10
VENTANA_MAX_S = 60
# Buffer por robot: VENTANA_MAX_S a la frecuencia por defecto de
# controller_node (state_pub_hz = 100), con margen.
MAX_MUESTRAS = VENTANA_MAX_S * 150
SIN_DATOS_S = 1.0           # "sin datos" si la ultima muestra es mas vieja
# Alto minimo del eje vertical por unidad: sin esto el autoescalado
# amplifica ruido numerico (p. ej. 1e-6 rad) hasta parecer una señal.
RANGO_MIN = {"rad": 0.02, "m": 0.005}

# Columnas de cada muestra en el buffer
T = 0
Q_MEAS, Q_CMD, U = range(1, 7), range(7, 13), range(13, 19)
X_MEAS, X_DES = range(19, 22), range(22, 25)
RPY_MEAS, RPY_DES = range(25, 28), range(28, 31)
COLUMNAS = 31
# Angulos de Euler: saltan entre +pi y -pi; se "desenrollan" para graficar
ANGULOS = set(RPY_MEAS) | set(RPY_DES)

# Colores (fondo y texto como config/style.qss)
FONDO = "#23272e"
TEXTO = "#e6e6e6"
PEN_ACTUAL = pg.mkPen("#00bfff", width=2)
PEN_DESEADA = pg.mkPen("#f39c12", width=2, style=Qt.DashLine)


class Buffer:
    """Buffer circular de muestras (filas de COLUMNAS valores)."""

    def __init__(self, n=MAX_MUESTRAS):
        self.data = np.zeros((n, COLUMNAS))
        self.i = 0
        self.lleno = False
        self.controller_type = ""

    def agregar(self, fila):
        self.data[self.i] = fila
        self.i = (self.i + 1) % len(self.data)
        if self.i == 0:
            self.lleno = True

    def muestras(self):
        """Muestras en orden cronologico."""
        if not self.lleno:
            return self.data[:self.i]
        return np.concatenate((self.data[self.i:], self.data[:self.i]))

    def ultima_t(self):
        if not self.lleno and self.i == 0:
            return None
        return self.data[self.i - 1, T]

    def limpiar(self):
        self.i = 0
        self.lleno = False


def fila_de(msg, t):
    fila = np.empty(COLUMNAS)
    fila[T] = t
    fila[1:7] = msg.q_meas
    fila[7:13] = msg.q_cmd
    fila[13:19] = msg.u_control
    fila[19:22] = (msg.x_meas.x, msg.x_meas.y, msg.x_meas.z)
    fila[22:25] = (msg.x_des.x, msg.x_des.y, msg.x_des.z)
    fila[25:28] = (msg.euler_meas.x, msg.euler_meas.y, msg.euler_meas.z)
    fila[28:31] = (msg.euler_des.x, msg.euler_des.y, msg.euler_des.z)
    return fila


def serie(datos, columna):
    if not len(datos):
        return np.empty(0)
    y = datos[:, columna]
    return np.unwrap(y) if columna in ANGULOS else y


class PestanaGraficas(QWidget):
    """Una pestaña: selector de robot + grilla de 3x2 graficos. graficos:
    lista de (titulo, unidad, columna_actual, columna_deseada o None), en
    orden por columnas (los 3 primeros a la izquierda)."""

    def __init__(self, graficos, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        cabecera = QHBoxLayout()
        cabecera.addWidget(QLabel("Robot:"))
        self.robot = QComboBox()
        self.robot.addItems([f"Robot {r[1:]} ({r})" for r in ROBOT_IDS])
        self.robot.setFixedWidth(130)
        cabecera.addWidget(self.robot)
        cabecera.addStretch()
        if any(deseada is not None for *_, deseada in graficos):
            leyenda = QLabel(
                "<span style='color:#00bfff'>━ actual</span>&nbsp;&nbsp;"
                "<span style='color:#f39c12'>╍ deseada</span>")
            cabecera.addWidget(leyenda)
        self.info = QLabel()
        self.info.setObjectName("graficas_info")
        cabecera.addWidget(self.info)
        layout.addLayout(cabecera)

        self.lienzo = pg.GraphicsLayoutWidget()
        self.lienzo.setBackground(FONDO)
        layout.addWidget(self.lienzo, 1)

        self.curvas = []
        self.plots = []
        primero = None
        fuente = QFont()
        fuente.setPointSize(7)
        for i, (titulo, unidad, actual, deseada) in enumerate(graficos):
            plot = self.lienzo.addPlot(row=i % 3, col=i // 3)
            # Unidad en el titulo (sin etiqueta de eje: el espacio vertical
            # es poco) y sin prefijos SI automaticos ("x1e-09").
            plot.setTitle(f"{titulo} [{unidad}]", color=TEXTO, size="8pt")
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setMenuEnabled(False)
            for eje in ("left", "bottom"):
                plot.getAxis(eje).setStyle(tickFont=fuente)
                plot.getAxis(eje).enableAutoSIPrefix(False)
            plot.getAxis("left").setWidth(45)
            # Eje de tiempo solo en la fila de abajo (estan enlazados)
            if i % 3 == 2:
                plot.setLabel("bottom", "t [s]", color=TEXTO)
            else:
                plot.getAxis("bottom").setStyle(showValues=False)
            if primero is None:
                primero = plot
            else:
                plot.setXLink(primero)
            curvas = [(plot.plot(pen=PEN_ACTUAL), actual)]
            if deseada is not None:
                curvas.append((plot.plot(pen=PEN_DESEADA), deseada))
            self.curvas += curvas
            self.plots.append((plot, curvas, RANGO_MIN.get(unidad)))

    def robot_id(self):
        return ROBOT_IDS[self.robot.currentIndex()]

    def dibujar(self, buffer, ahora, ventana_s):
        datos = buffer.muestras()
        if len(datos):
            datos = datos[datos[:, T] >= ahora - ventana_s]
        t = datos[:, T] if len(datos) else np.empty(0)
        for plot, curvas, rango_min in self.plots:
            series = [serie(datos, columna) for _, columna in curvas]
            if len(series) == 2 and len(t) and curvas[0][1] in ANGULOS:
                # Tras desenrollar, la deseada en la misma vuelta que la actual
                vueltas = np.round((series[0][0] - series[1][0]) / (2 * np.pi))
                series[1] = series[1] + vueltas * 2 * np.pi
            for (curva, _), y in zip(curvas, series):
                curva.setData(t, y)
            if rango_min and len(t):
                bajo = min(y.min() for y in series)
                alto = max(y.max() for y in series)
                if alto - bajo < rango_min:
                    centro = (alto + bajo) / 2
                    plot.setYRange(centro - rango_min / 2, centro + rango_min / 2, padding=0)
                else:
                    plot.enableAutoRange(axis="y")

        ultima = buffer.ultima_t()
        if ultima is None or ahora - ultima > SIN_DATOS_S:
            self.info.setText(f"sin datos de /{self.robot_id()}/controller_state "
                              "(¿controlador corriendo?)")
        else:
            self.info.setText(f"controlador {buffer.controller_type}")


class Graficas(QTabWidget):
    """node: nodo rclpy bombeado en el hilo de Qt (RobotMonitor.node)."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.setObjectName("graficas")
        self.setTabPosition(QTabWidget.North)
        self.setTabShape(QTabWidget.Rounded)
        pg.setConfigOptions(antialias=False, foreground=TEXTO)

        self.t0 = time.monotonic()
        self.buffers = {r: Buffer() for r in ROBOT_IDS}
        self.subs = [
            node.create_subscription(
                ControllerState, f"/{r}/controller_state",
                lambda msg, r=r: self._on_state(r, msg), 10)
            for r in ROBOT_IDS]

        self.joint_states_tab = PestanaGraficas([
            (nombre, "rad", Q_MEAS[i], Q_CMD[i]) for i, nombre in enumerate(JOINTS)])
        self.cartesian_pose_tab = PestanaGraficas(
            [(eje, "m", X_MEAS[i], X_DES[i]) for i, eje in enumerate(("x", "y", "z"))] +
            [(eje, "rad", RPY_MEAS[i], RPY_DES[i]) for i, eje in enumerate(("roll", "pitch", "yaw"))])
        self.control_effort_tab = PestanaGraficas([
            (nombre, "u", U[i], None) for i, nombre in enumerate(JOINTS)])
        self.addTab(self.joint_states_tab, "Joint States")
        self.addTab(self.cartesian_pose_tab, "Cartesian Pose")
        self.addTab(self.control_effort_tab, "Control Effort")

        # Controles comunes a las 3 pestañas, a la derecha de los tabs
        esquina = QWidget()
        controles = QHBoxLayout(esquina)
        controles.setContentsMargins(0, 0, 4, 0)
        controles.addWidget(QLabel("Ventana"))
        self.ventana = QSpinBox()
        self.ventana.setRange(2, VENTANA_MAX_S)
        self.ventana.setValue(VENTANA_INICIAL_S)
        self.ventana.setSuffix(" s")
        controles.addWidget(self.ventana)
        self.boton_pausa = QPushButton("Pausar")
        self.boton_pausa.setCheckable(True)
        self.boton_pausa.toggled.connect(
            lambda pausado: self.boton_pausa.setText("Reanudar" if pausado else "Pausar"))
        controles.addWidget(self.boton_pausa)
        self.boton_limpiar = QPushButton("Limpiar")
        self.boton_limpiar.clicked.connect(self.limpiar)
        controles.addWidget(self.boton_limpiar)
        self.setCornerWidget(esquina, Qt.TopRightCorner)

        self.currentChanged.connect(lambda _i: self.redibujar())
        for pestana in self.pestanas():
            pestana.robot.currentIndexChanged.connect(lambda _i: self.redibujar())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.redibujar)
        self.timer.start(REDIBUJO_MS)

    def pestanas(self):
        return (self.joint_states_tab, self.cartesian_pose_tab, self.control_effort_tab)

    def _on_state(self, robot_id, msg):
        buffer = self.buffers[robot_id]
        buffer.controller_type = msg.controller_type
        buffer.agregar(fila_de(msg, time.monotonic() - self.t0))

    def redibujar(self):
        """Solo la pestaña visible y si no esta en pausa (los datos se
        siguen guardando en pausa)."""
        if not self.isVisible() or self.boton_pausa.isChecked():
            return
        pestana = self.currentWidget()
        pestana.dibujar(self.buffers[pestana.robot_id()],
                        time.monotonic() - self.t0, self.ventana.value())

    def limpiar(self):
        for buffer in self.buffers.values():
            buffer.limpiar()
        self.redibujar()


if __name__ == "__main__":
    # Prueba independiente: python3 -m ur5_panel.modulos.graficas
    # (con controller_node corriendo para ver datos)
    import sys
    import rclpy
    rclpy.init()
    app = QApplication(sys.argv)
    nodo = rclpy.create_node("graficas_test")
    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(nodo, timeout_sec=0))
    spin.start(5)
    graficas = Graficas(nodo)
    graficas.resize(1000, 600)
    graficas.show()
    codigo = app.exec_()
    nodo.destroy_node()
    rclpy.try_shutdown()
    sys.exit(codigo)
