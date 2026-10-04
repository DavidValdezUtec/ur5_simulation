"""Panel de estado: posicion articular y pose cartesiana del efector final
de cada robot lanzado (r1, r2).

  - Articular: /rN/joint_states (los nombres traen el prefijo 'rN_', y el
    orden del mensaje no esta garantizado: se ordena por nombre segun
    UR_JOINTS).
  - Cartesiana: TF world -> rN_tool_tip, la punta de la herramienta (mismo
    frame base y efector que usa controller_node con Pinocchio). Cada
    robot_state_publisher publica en el /tf global, por eso basta un solo
    TransformListener para los dos robots.

El nodo ROS2 de este modulo se bombea en un hilo propio (no en el QTimer
de RobotMonitor): /tf llega a cientos de Hz por robot y no debe competir
con la UI. Los callbacks solo guardan el ultimo mensaje; la UI lo lee con
un QTimer a REFRESCO_MS (el buffer de tf2 es thread-safe).
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

ROBOT_IDS = ("r1", "r2")
# Orden de los joints de un UR (sin el prefijo 'rN_').
UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LABELS = ("Base", "Hombro", "Codo", "Muñeca 1", "Muñeca 2", "Muñeca 3")
FRAME_BASE = "world"
FRAME_EFECTOR = "tool_tip"  # se le antepone el tf_prefix: 'r1_tool_tip'

REFRESCO_MS = 100        # 10 Hz en pantalla es suficiente para leer
DATOS_VIEJOS_S = 1.0     # mas viejo que esto se muestra como "sin datos"
SIN_DATO = "—"


def quaternion_a_rpy(x, y, z, w):
    """Cuaternion -> (roll, pitch, yaw) en rad, convencion de URDF/TF
    (rotaciones fijas X, Y, Z = intrinsecas Z-Y'-X'')."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class EstadoNode:
    """Nodo ROS2 en su propio hilo: guarda el ultimo /rN/joint_states y
    escucha /tf. Solo lectura; no publica nada."""

    def __init__(self, robot_ids=ROBOT_IDS):
        self.node = rclpy.create_node("ur5_panel_estado")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        # {robot_id: (JointState, time.monotonic() de llegada)}
        self.joint_states = {r: None for r in robot_ids}
        for robot_id in robot_ids:
            self.node.create_subscription(
                JointState, f"/{robot_id}/joint_states",
                lambda msg, r=robot_id: self.joint_states.__setitem__(r, (msg, time.monotonic())),
                1)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def posiciones(self, robot_id):
        """Lista de 6 posiciones [rad] en el orden de UR_JOINTS, o None si
        no hay /joint_states reciente o le faltan joints."""
        dato = self.joint_states.get(robot_id)
        if dato is None or time.monotonic() - dato[1] > DATOS_VIEJOS_S:
            return None
        msg = dato[0]
        por_nombre = dict(zip(msg.name, msg.position))
        try:
            return [por_nombre[f"{robot_id}_{j}"] for j in UR_JOINTS]
        except KeyError:
            # Robot lanzado sin tf_prefix (nombres sin 'rN_').
            try:
                return [por_nombre[j] for j in UR_JOINTS]
            except KeyError:
                return None

    def pose_efector(self, robot_id):
        """((x, y, z) [m], (roll, pitch, yaw) [rad]) de rN_tool_tip respecto
        a world, o None si TF todavia no tiene la cadena completa."""
        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME_BASE, f"{robot_id}_{FRAME_EFECTOR}", Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), quaternion_a_rpy(q.x, q.y, q.z, q.w)

    def destroy(self):
        """Idempotente: se llama desde panel.shutdown() (closeEvent y atexit)."""
        if self.node is None:
            return
        self.executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=1.0)
        self.node.destroy_node()
        self.node = None


class RobotEstadoBox(QGroupBox):
    """Cuadro de un robot: tabla de joints (rad y grados) y pose del efector."""

    def __init__(self, robot_id, parent=None):
        super().__init__(f"Robot {robot_id[1:]} ({robot_id})", parent)
        self.robot_id = robot_id
        mono = QFont("Monospace")
        mono.setStyleHint(QFont.TypeWriter)

        def valor():
            label = QLabel(SIN_DATO)
            label.setFont(mono)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            return label

        grid = QGridLayout(self)
        fila = 0
        grid.addWidget(QLabel("<b>Articular</b>"), fila, 0)
        grid.addWidget(QLabel("rad"), fila, 1, Qt.AlignRight)
        grid.addWidget(QLabel("°"), fila, 2, Qt.AlignRight)
        self.joint_rad, self.joint_deg = [], []
        for nombre in JOINT_LABELS:
            fila += 1
            self.joint_rad.append(valor())
            self.joint_deg.append(valor())
            grid.addWidget(QLabel(nombre), fila, 0)
            grid.addWidget(self.joint_rad[-1], fila, 1)
            grid.addWidget(self.joint_deg[-1], fila, 2)

        fila += 1
        titulo = QLabel(f"<b>Efector</b> ({robot_id}_{FRAME_EFECTOR} en {FRAME_BASE})")
        titulo.setWordWrap(True)
        grid.addWidget(titulo, fila, 0, 1, 3)
        self.pos, self.rot = [], []
        for eje_pos, eje_rot in (("x", "roll"), ("y", "pitch"), ("z", "yaw")):
            fila += 1
            self.pos.append(valor())
            self.rot.append(valor())
            grid.addWidget(QLabel(f"{eje_pos} [m] / {eje_rot} [°]"), fila, 0)
            grid.addWidget(self.pos[-1], fila, 1)
            grid.addWidget(self.rot[-1], fila, 2)
        grid.setColumnStretch(0, 1)

    def set_joints(self, posiciones):
        for i, (rad, deg) in enumerate(zip(self.joint_rad, self.joint_deg)):
            if posiciones is None:
                rad.setText(SIN_DATO)
                deg.setText(SIN_DATO)
            else:
                rad.setText(f"{posiciones[i]:+.4f}")
                deg.setText(f"{math.degrees(posiciones[i]):+.2f}")

    def set_pose(self, pose):
        for i in range(3):
            if pose is None:
                self.pos[i].setText(SIN_DATO)
                self.rot[i].setText(SIN_DATO)
            else:
                self.pos[i].setText(f"{pose[0][i]:+.4f}")
                self.rot[i].setText(f"{math.degrees(pose[1][i]):+.2f}")


class Estado(QWidget):
    """Widget con la posicion articular y la pose cartesiana actuales de
    cada robot. Crea su propio EstadoNode (requiere rclpy.init() hecho);
    llamar a cerrar() al salir (no se llama destroy: taparia QWidget.destroy)."""

    def __init__(self, robot_ids=ROBOT_IDS, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Estado del Robot")
        self.ros = EstadoNode(robot_ids)

        layout = QVBoxLayout(self)
        self.boxes = {}
        for robot_id in robot_ids:
            self.boxes[robot_id] = RobotEstadoBox(robot_id)
            layout.addWidget(self.boxes[robot_id])

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.actualizar)
        self._timer.start(REFRESCO_MS)

    def actualizar(self):
        """Refresca los valores en pantalla (QTimer, cada REFRESCO_MS)."""
        if self.ros.node is None:
            return
        for robot_id, box in self.boxes.items():
            box.set_joints(self.ros.posiciones(robot_id))
            box.set_pose(self.ros.pose_efector(robot_id))

    def cerrar(self):
        self._timer.stop()
        self.ros.destroy()


def main():
    """Prueba independiente: python3 -m ur5_panel.modulos.estado"""
    rclpy.init()
    app = QApplication(sys.argv)
    estado = Estado()
    estado.show()
    codigo = app.exec_()
    estado.cerrar()
    rclpy.try_shutdown()
    sys.exit(codigo)


if __name__ == "__main__":
    main()
