"""Nodo ROS2 propio del panel para (1) saber en que estado esta cada robot
y (2) el STOP: dejar los robots quietos cambiando de controlador.

Todo va por servicios/topicos con rclpy (como ControllerSwitchCoordinator
de ur5_controller), no con 'ros2 control ...' en subprocesos: una llamada
de servicio tarda milisegundos, un subproceso de la CLI 1-2 s. El nodo se
bombea con un QTimer en el hilo de Qt (igual que camera.py), asi que los
callbacks de ROS corren en el mismo hilo que la UI y pueden tocar widgets.

Estado de un robot (ver estado()):
  detenido  gris      no hay launch activo para el robot
  lanzado   amarillo  launch vivo, pero sin /rN/joint_states recientes; en
                      modo real tambien si el programa External Control no
                      esta corriendo en el UR (robot_program_running)
  listo     verde     /rN/joint_states llego hace < 1 s (y en real, el
                      programa esta corriendo)
  error     rojo      el launch termino sin pedirlo, o el robot estuvo listo
                      y lleva > 2 s sin publicar /rN/joint_states
"""
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

ROBOT_IDS = ("r1", "r2")

JOINT_STATES_FRESCO_S = 1.0   # listo si el ultimo /joint_states es mas nuevo
JOINT_STATES_PERDIDO_S = 2.0  # error si estuvo listo y pasa esto sin datos

# Controlador en el que queda cada robot tras el STOP: es el estado inicial
# que esperan controller_node y el boton Home. Sin comandos nuevos mantiene
# la ultima consigna de posicion escrita en el hardware.
IDLE_CONTROLLER = "forward_position_controller"
# Controlador de paso para "congelar" el robot: al activarse, un
# JointTrajectoryController escribe la posicion ACTUAL como consigna y la
# mantiene. Existe (inactivo) en real, fake y Gazebo.
HOLD_CONTROLLER = "joint_trajectory_controller"
# Controladores de trayectoria: su ultima consigna ya es (casi) la posicion
# actual, asi que basta cambiarlos por IDLE_CONTROLLER de forma atomica.
TRAJECTORY_CONTROLLERS = ("scaled_joint_trajectory_controller", "joint_trajectory_controller")
# Todos los controladores que mueven el robot (los broadcasters y los de
# configuracion del UR no se tocan).
MOTION_CONTROLLERS = TRAJECTORY_CONTROLLERS + (
    IDLE_CONTROLLER,
    "forward_velocity_controller",
    "forward_effort_controller",
    "passthrough_trajectory_controller",
    "freedrive_mode_controller",
    "force_mode_controller",
)
# Pausa entre "congelar con HOLD_CONTROLLER" y "pasar a IDLE_CONTROLLER",
# para que el JTC alcance a escribir la posicion actual como consigna.
HOLD_MS = 200


class RobotMonitor:
    def __init__(self, robot_ids=ROBOT_IDS):
        self.robot_ids = robot_ids
        self.node = rclpy.create_node("ur5_panel_monitor")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self._last_joint_states = {}
        self._program_running = {}
        self._was_ready = {}
        # robot_program_running se publica solo al cambiar y "latched"
        # (transient local): hay que suscribirse con la misma durabilidad
        # para recibir el valor actual aunque se haya publicado antes.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._list_clients = {}
        self._switch_clients = {}
        for robot_id in robot_ids:
            self.reset(robot_id)
            self.node.create_subscription(
                JointState, f"/{robot_id}/joint_states",
                lambda _msg, r=robot_id: self._on_joint_states(r), 1)
            self.node.create_subscription(
                Bool, f"/{robot_id}/io_and_status_controller/robot_program_running",
                lambda msg, r=robot_id: self._program_running.__setitem__(r, msg.data), latched)
            self._list_clients[robot_id] = self.node.create_client(
                ListControllers, f"/{robot_id}/controller_manager/list_controllers")
            self._switch_clients[robot_id] = self.node.create_client(
                SwitchController, f"/{robot_id}/controller_manager/switch_controller")

        self._timer = QTimer()
        self._timer.timeout.connect(self._spin)
        self._timer.start(10)

    # ------------------------------------------------------------------ ROS
    def _spin(self):
        # Varias vueltas por tick para no atrasarse con /joint_states (hasta
        # cientos de Hz por robot); con timeout 0 ninguna bloquea la UI.
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0)

    def _on_joint_states(self, robot_id):
        self._last_joint_states[robot_id] = time.monotonic()

    def destroy(self):
        """Idempotente: panel.shutdown() corre desde closeEvent y atexit."""
        if self.node is None:
            return
        self._timer.stop()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.node = None

    # --------------------------------------------------------------- estado
    def reset(self, robot_id):
        """Olvida lo recibido del robot (llamar al (re)lanzarlo)."""
        self._last_joint_states[robot_id] = None
        self._program_running[robot_id] = None
        self._was_ready[robot_id] = False

    def estado(self, robot_id, proceso, modo):
        """Devuelve (estado, detalle) de un robot. proceso: 'sin_proceso' |
        'vivo' | 'terminado' (ver RobotsLaunchModule.estado_proceso). modo:
        'fake' | 'real' | 'gazebo' con el que se lanzo."""
        if proceso == "sin_proceso":
            self._was_ready[robot_id] = False
            return "detenido", "Detenido"
        if proceso == "terminado":
            return "error", "El launch terminó inesperadamente (ver ~/.ros/ur5_panel/logs/)"

        last = self._last_joint_states[robot_id]
        edad = None if last is None else time.monotonic() - last
        if edad is not None and edad < JOINT_STATES_FRESCO_S:
            if modo == "real" and self._program_running[robot_id] is not True:
                return "lanzado", "Conectado; el programa External Control no está corriendo en el robot"
            self._was_ready[robot_id] = True
            return "listo", "Listo"
        if self._was_ready[robot_id] and edad is not None and edad > JOINT_STATES_PERDIDO_S:
            return "error", f"Sin /{robot_id}/joint_states desde hace {edad:.0f} s"
        return "lanzado", f"Lanzado, esperando /{robot_id}/joint_states"

    # ----------------------------------------------------------------- STOP
    def detener_movimiento(self, robot_id, on_done):
        """Deja el robot quieto en su posicion actual y con IDLE_CONTROLLER
        activo. No bloquea: on_done(ok, mensaje) se llama al terminar.

          - solo IDLE_CONTROLLER activo: ya esta quieto (sin comandos nuevos
            mantiene su ultima consigna).
          - un controlador de trayectoria (p.ej. scaled_joint_trajectory_
            controller yendo a home): cambio atomico a IDLE_CONTROLLER; la
            trayectoria se corta y queda la ultima consigna del JTC.
          - cualquier otro caso (velocidad, esfuerzo, ninguno activo): primero
            HOLD_CONTROLLER, que congela la posicion actual, y despues
            IDLE_CONTROLLER. Pasar directo a IDLE_CONTROLLER no sirve: la
            consigna de posicion del hardware podria ser vieja y el robot
            saltaria a ella. En Gazebo tampoco se deja nunca el robot sin un
            controlador de posicion activo (se caeria por gravedad).
        """
        client = self._list_clients[robot_id]
        if not client.service_is_ready():
            on_done(False, "sin controller_manager (¿robot no lanzado?)")
            return
        future = client.call_async(ListControllers.Request())
        future.add_done_callback(lambda f: self._on_list(robot_id, f, on_done))

    def _on_list(self, robot_id, future, on_done):
        try:
            controllers = future.result().controller
        except Exception as e:
            on_done(False, f"list_controllers falló: {e}")
            return
        loaded = {c.name for c in controllers}
        motion = [c.name for c in controllers if c.state == "active" and c.name in MOTION_CONTROLLERS]

        if motion == [IDLE_CONTROLLER]:
            on_done(True, f"quieto ({IDLE_CONTROLLER})")
        elif motion and all(c in TRAJECTORY_CONTROLLERS for c in motion):
            self._switch(robot_id, [IDLE_CONTROLLER], motion, lambda ok, msg: on_done(
                ok, f"{', '.join(motion)} -> {IDLE_CONTROLLER}" if ok else msg))
        elif HOLD_CONTROLLER in loaded:
            def congelado(ok, msg):
                if not ok:
                    on_done(False, msg)
                    return
                QTimer.singleShot(HOLD_MS, lambda: self._switch(
                    robot_id, [IDLE_CONTROLLER], [HOLD_CONTROLLER],
                    lambda ok2, msg2: on_done(
                        ok2, f"{', '.join(motion) or 'ninguno'} -> {HOLD_CONTROLLER} -> "
                             f"{IDLE_CONTROLLER}" if ok2 else msg2)))
            # Si HOLD_CONTROLLER ya estaba activo (junto a otro), no se puede
            # pedir activarlo y desactivarlo a la vez: solo se apagan los demas.
            activar = [] if HOLD_CONTROLLER in motion else [HOLD_CONTROLLER]
            desactivar = [c for c in motion if c != HOLD_CONTROLLER]
            self._switch(robot_id, activar, desactivar, congelado)
        else:
            on_done(False, f"{HOLD_CONTROLLER} no está cargado; no se puede congelar el robot")

    def _switch(self, robot_id, activate, deactivate, on_done):
        client = self._switch_clients[robot_id]
        if not client.service_is_ready():
            on_done(False, "switch_controller no disponible")
            return
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(sec=5)

        def respuesta(f):
            try:
                ok = f.result().ok
            except Exception as e:
                on_done(False, f"switch_controller falló: {e}")
                return
            on_done(ok, "" if ok else
                    f"switch_controller rechazado (activar {activate}, desactivar {deactivate})")
        client.call_async(request).add_done_callback(respuesta)
