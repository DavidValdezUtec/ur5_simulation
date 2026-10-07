"""Nodo ROS2 propio del panel para (1) saber en que estado esta cada robot,
(2) el STOP: dejar los robots quietos cambiando de controlador, (3) el
home (ir_a_home) y (4) saber si se presiono el boton gris de cada haptico
(asistente de teleoperacion).

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

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from omni_msgs.msg import OmniState
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5_panel import config_store

ROBOT_IDS = ("r1", "r2")
# Haptico de cada robot: /phantomN/state (el mismo que usa
# panel.py:start_controller).
PHANTOM = {"r1": "phantom1", "r2": "phantom2"}
UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

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

# Home: la duracion de la trayectoria sale del joint que mas se mueve, a
# HOME_VEL_RAD_S, y nunca menos de HOME_MIN_S.
HOME_MIN_S = 3.0
HOME_VEL_RAD_S = 0.5
# Margen sobre la duracion antes de dar el home por fallido.
HOME_TIMEOUT_EXTRA_S = 10.0


class RobotMonitor:
    def __init__(self, robot_ids=ROBOT_IDS):
        self.robot_ids = robot_ids
        self.node = rclpy.create_node("ur5_panel_monitor")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self._last_joint_states = {}
        self._joint_positions = {}
        self._program_running = {}
        self._was_ready = {}
        self._boton_gris = {}
        # robot_program_running se publica solo al cambiar y "latched"
        # (transient local): hay que suscribirse con la misma durabilidad
        # para recibir el valor actual aunque se haya publicado antes.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._list_clients = {}
        self._switch_clients = {}
        self._home_clients = {}
        for robot_id in robot_ids:
            self.reset(robot_id)
            self._boton_gris[robot_id] = False
            self.node.create_subscription(
                JointState, f"/{robot_id}/joint_states",
                lambda msg, r=robot_id: self._on_joint_states(r, msg), 1)
            self.node.create_subscription(
                OmniState, f"/{PHANTOM[robot_id]}/state",
                lambda msg, r=robot_id: self._on_omni_state(r, msg), 1)
            self.node.create_subscription(
                Bool, f"/{robot_id}/io_and_status_controller/robot_program_running",
                lambda msg, r=robot_id: self._program_running.__setitem__(r, msg.data), latched)
            self._list_clients[robot_id] = self.node.create_client(
                ListControllers, f"/{robot_id}/controller_manager/list_controllers")
            self._switch_clients[robot_id] = self.node.create_client(
                SwitchController, f"/{robot_id}/controller_manager/switch_controller")
            self._home_clients[robot_id] = {
                ctrl: ActionClient(self.node, FollowJointTrajectory,
                                   f"/{robot_id}/{ctrl}/follow_joint_trajectory")
                for ctrl in TRAJECTORY_CONTROLLERS}

        self._timer = QTimer()
        self._timer.timeout.connect(self._spin)
        self._timer.start(10)

    # ------------------------------------------------------------------ ROS
    def _spin(self):
        # Varias vueltas por tick para no atrasarse con /joint_states (hasta
        # cientos de Hz por robot); con timeout 0 ninguna bloquea la UI.
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0)

    def _on_joint_states(self, robot_id, msg):
        self._last_joint_states[robot_id] = time.monotonic()
        self._joint_positions[robot_id] = dict(zip(msg.name, msg.position))

    def _on_omni_state(self, robot_id, msg):
        if msg.close_gripper:
            self._boton_gris[robot_id] = True

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
        self._joint_positions[robot_id] = None
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

    # ----------------------------------------------------------------- HOME
    def ir_a_home(self, robot_id, q_target, on_done):
        """Lleva el robot a q_target (6 joints, rad) con un controlador de
        trayectoria y al terminar deja IDLE_CONTROLLER activo (lo que espera
        controller_node). No bloquea: on_done(ok, mensaje) avisa al final,
        tambien si falla (en ese caso igual se intenta dejar el robot quieto
        en IDLE_CONTROLLER).

        Usa scaled_joint_trajectory_controller si esta cargado (real, fake),
        si no joint_trajectory_controller."""
        client = self._list_clients[robot_id]
        if not client.service_is_ready():
            on_done(False, "sin controller_manager (¿robot no lanzado?)")
            return
        if len(q_target) != len(UR_JOINTS):
            on_done(False, f"q_target debe tener {len(UR_JOINTS)} valores: {q_target}")
            return
        future = client.call_async(ListControllers.Request())
        future.add_done_callback(lambda f: self._home_list(robot_id, list(q_target), f, on_done))

    def _home_list(self, robot_id, q_target, future, on_done):
        try:
            controllers = future.result().controller
        except Exception as e:
            on_done(False, f"list_controllers falló: {e}")
            return
        loaded = {c.name for c in controllers}
        motion = [c.name for c in controllers if c.state == "active" and c.name in MOTION_CONTROLLERS]
        ctrl = next((c for c in TRAJECTORY_CONTROLLERS if c in loaded), None)
        if ctrl is None:
            on_done(False, f"ningún controlador de trayectoria cargado ({', '.join(TRAJECTORY_CONTROLLERS)})")
            return

        def activo(ok, msg):
            if not ok:
                on_done(False, msg)
                return
            self._enviar_home(robot_id, ctrl, q_target, on_done)

        if motion == [ctrl]:
            activo(True, "")
        else:
            activar = [] if ctrl in motion else [ctrl]
            self._switch(robot_id, activar, [c for c in motion if c != ctrl], activo)

    def _duracion_home(self, robot_id, q_target):
        posiciones = self._joint_positions.get(robot_id) or {}
        prefix = config_store.tf_prefix(robot_id)
        delta = max((abs(q - posiciones[prefix + j]) for j, q in zip(UR_JOINTS, q_target)
                     if prefix + j in posiciones), default=0.0)
        return max(HOME_MIN_S, delta / HOME_VEL_RAD_S)

    def _enviar_home(self, robot_id, ctrl, q_target, on_done, intentos=20):
        action = self._home_clients[robot_id][ctrl]
        if not action.server_is_ready():
            if intentos > 0:
                QTimer.singleShot(100, lambda: self._enviar_home(
                    robot_id, ctrl, q_target, on_done, intentos - 1))
            else:
                self._terminar_home(robot_id, on_done, False, f"acción de {ctrl} no disponible")
            return

        duracion = self._duracion_home(robot_id, q_target)
        prefix = config_store.tf_prefix(robot_id)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [prefix + j for j in UR_JOINTS]
        punto = JointTrajectoryPoint()
        punto.positions = [float(q) for q in q_target]
        punto.time_from_start = Duration(sec=int(duracion), nanosec=int((duracion % 1) * 1e9))
        goal.trajectory.points = [punto]
        print(f"[{robot_id}] Home con {ctrl} en {duracion:.1f} s: {q_target}")

        estado = {"terminado": False, "goal": None}

        def fin(ok, msg):
            if not estado["terminado"]:
                estado["terminado"] = True
                self._terminar_home(robot_id, on_done, ok, msg)

        def aceptado(f):
            try:
                goal_handle = f.result()
            except Exception as e:
                fin(False, f"send_goal falló: {e}")
                return
            if not goal_handle.accepted:
                fin(False, f"{ctrl} rechazó la trayectoria de home")
                return
            estado["goal"] = goal_handle
            goal_handle.get_result_async().add_done_callback(resultado)

        def resultado(f):
            try:
                res = f.result()
            except Exception as e:
                fin(False, f"resultado del home falló: {e}")
                return
            if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.error_code == 0:
                fin(True, "home alcanzado")
            else:
                fin(False, f"home no completado (estado {res.status}, "
                           f"error {res.result.error_code} {res.result.error_string})")

        def timeout():
            if not estado["terminado"]:
                if estado["goal"] is not None:
                    estado["goal"].cancel_goal_async()
                fin(False, f"timeout: el home no terminó en {duracion + HOME_TIMEOUT_EXTRA_S:.0f} s")

        action.send_goal_async(goal).add_done_callback(aceptado)
        QTimer.singleShot(int((duracion + HOME_TIMEOUT_EXTRA_S) * 1000), timeout)

    def _terminar_home(self, robot_id, on_done, ok, msg):
        """Tras el home (bien o mal) deja IDLE_CONTROLLER activo y avisa."""
        def idle(ok2, msg2):
            if ok and ok2:
                on_done(True, f"{msg}; {IDLE_CONTROLLER} activo")
            elif ok:
                on_done(False, f"{msg}, pero no se pudo activar {IDLE_CONTROLLER}: {msg2}")
            else:
                on_done(False, msg)
        self.detener_movimiento(robot_id, idle)

    # ------------------------------------------------- botones de hapticos
    def reset_boton_gris(self, robot_id):
        self._boton_gris[robot_id] = False

    def boton_gris(self, robot_id):
        """True si el boton gris del haptico del robot se presiono desde el
        ultimo reset_boton_gris (controller_node captura ahi la referencia
        y el robot empieza a seguir al haptico)."""
        return self._boton_gris[robot_id]

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
