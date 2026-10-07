#!/usr/bin/env python3
import os
import sys
import atexit
import subprocess
import signal
import rclpy
import yaml

from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QIcon, QPainter, QColor
from PyQt5.QtSvg import QSvgRenderer
from PyQt5 import QtCore

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except Exception:
    PackageNotFoundError = Exception
    get_package_share_directory = None

# Import funciones from the same package
from ur5_panel.funciones import *
from ur5_panel.modulos.ui_menu import UIMenuMixin
from ur5_panel.modulos.ui_robot_config import UIRobotConfigMixin
from ur5_panel.modulos.mapping_matrix import MappingMatrixMixin
from ur5_panel.modulos.ui_controller_config import UIControllerConfigMixin
from ur5_panel.modulos.ui_joint_ik_control import UIJointIkControlMixin
from ur5_panel.modulos.camera import CameraModule
from ur5_panel.modulos.camera_view import CamaraPanel, VentanaCamara
from ur5_panel.modulos.haptic import HapticModule
from ur5_panel.modulos.robots_launch import RobotsLaunchModule
from ur5_panel.modulos.dock import Dock
from ur5_panel.modulos.robot_monitor import RobotMonitor
from ur5_panel.modulos.estado import Estado
from ur5_panel.modulos.modos import MODO_DESARROLLO, MODO_TELEOP, ModosPanel
from ur5_panel.modulos.asistente_teleop import AsistenteTeleop

# Import RVizQtWidget from the installed ur5_interfaz_library package
try:
    from ur5_interfaz_library.RvizWrapper import RVizQtWidget
except ImportError as e:
    print(f"Error: Could not import RVizQtWidget from ur5_interfaz_library")
    print(f"Details: {e}")
    print("\nMake sure you have:")
    print("  1. Built ur5_interfaz_library: colcon build --packages-select ur5_interfaz_library")
    print("  2. Sourced the workspace: source install/setup.bash")
    sys.exit(1)

class InterfazRviz(
    QMainWindow,
    UIMenuMixin,
    UIRobotConfigMixin,
    MappingMatrixMixin,
    UIControllerConfigMixin,
    UIJointIkControlMixin,
):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("INTERFAZ")
        #self.resize(1600, 800)
        
        # Modulos de composicion: cada uno maneja su propio proceso externo.
        # self.camera se crea sin labels todavia (set_devices_menu, mas
        # adelante en setup_ui, ya llama a buscar_dispositivos() y necesita
        # poder detener/lanzar el launch de camara); set_camara_widget le
        # registra el label real y arranca la suscripcion ROS2.
        # self.robots se crea en setup_ui, una vez existe el rviz_widget.
        self.camera = CameraModule()
        self.ventana_camara = None
        self.asistente = None
        self.haptic = HapticModule()
        self.robots_running = False
        
        # Inicializar ROS2 para el nodo de la cámara
        if not rclpy.ok():
            rclpy.init()
        
        # Configurar widget central y layout principal
        try:
            self.setup_ui()
        except Exception as e:
            print(f"[Error] Fallo durante la inicialización: {e}")
            import traceback
            traceback.print_exc()
            # Asegurar limpieza antes de salir
            self.shutdown()
            raise
        
        # Configurar limpieza al salir
        atexit.register(self.shutdown)

    def cargar_y_colorear_svg(self, file_path, color):
        """
        Carga un archivo SVG, lo colorea y devuelve un QIcon.

        :param file_path: Ruta al archivo .svg.
        :param color: El nuevo color (p. ej., QColor(Qt.white), "#FF0000").
        :return: QIcon coloreado.
        """
        # 1. Renderizar el SVG original en un QPixmap
        renderer = QSvgRenderer(file_path)
        pixmap = QPixmap(renderer.defaultSize())
        pixmap.fill(Qt.transparent)  # Empezar con un fondo transparente

        painter = QPainter(pixmap)
        renderer.render(painter)
        painter.end()

        # 2. Crear una máscara a partir del pixmap renderizado
        #    La máscara usa el canal alfa del SVG
        mask = pixmap.createMaskFromColor(Qt.transparent)

        # 3. Crear un pixmap de resultado y rellenarlo con el color deseado
        result_pixmap = QPixmap(pixmap.size())
        result_pixmap.fill(QColor(color))

        # 4. Aplicar la máscara
        result_pixmap.setMask(mask)

        return QIcon(result_pixmap)
    
    def rotar_icon(self, icon, angle):
        pix = icon.pixmap(icon.actualSize(QSize(64, 64))) # Obtener pixmap del QIcon
        transform = QTransform().rotate(angle)
        rotated_pixmap = pix.transformed(transform, Qt.SmoothTransformation)
        return QIcon(rotated_pixmap)

    def _get_package_paths(self):
        """Obtiene rutas a recursos del paquete (share/config y share/resource)."""
        share_dir = None
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory('ur5_panel')
            except PackageNotFoundError:
                share_dir = None

        if share_dir is None:
            package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            share_dir = package_root

        icons_dir = os.path.join(share_dir, 'resource', 'icons')
        qss_path = os.path.join(share_dir, 'config', 'style.qss')
        return share_dir, icons_dir, qss_path
    
    def cargar_iconos(self):
        _, self.icon_path, _ = self._get_package_paths()
        
        self.icon_menu1 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu1.svg"), "#FFFFFF"), 0)
        self.icon_menu2 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu2.svg"), "#FFFFFF"), 90)
        self.icon_menu3 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu3.svg"), "#FFFFFF"), 90)
        self.icon_menu4 = self.rotar_icon(self.cargar_y_colorear_svg(os.path.join(self.icon_path, "menu4.svg"), "#FFFFFF"), 90)
        self.icon_reload = self.cargar_y_colorear_svg(os.path.join(self.icon_path, "reload.svg"), "#FFFFFF")
        pass

    def setup_ui(self):
        # 2. Initialize RViz in Passive Mode (empty, without robots)
        print("Launching RViz in Passive Mode (empty - robots will be added on demand)...")
        # urdf_path="" tells the wrapper NOT to start its own state publishers
        # description_topic="" means no initial robot subscription
        try:
            self.rviz_widget = RVizQtWidget(
                urdf_path="", 
                description_topic="",  # Sin tópico inicial - robots se agregan dinámicamente
                fixed_frame="world"
            )
            self.rviz_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            # NO agregar robots aquí - se agregarán cuando se presione "Iniciar Robots"
        except Exception as e:
            print(f"[Error] No se pudo inicializar RViz widget: {e}")
            # Crear un widget placeholder en caso de error
            self.rviz_widget = QLabel("Error: No se pudo cargar RViz")
            self.rviz_widget.setStyleSheet("background-color: #2b2b2b; color: #ff6b6b; font-size: 16px;")
            self.rviz_widget.setAlignment(Qt.AlignCenter)
            raise

        self.robots = RobotsLaunchModule(self.rviz_widget)

        # Barra superior (dispositivos, estado de robots, config, STOP).
        # Va antes del menu: set_devices_menu/set_robot_menu conectan sus
        # botones. RobotMonitor es el nodo ROS2 del panel para el estado de
        # los robots y el STOP.
        self.monitor = RobotMonitor()
        
        self.dock = Dock(self)
        self.addToolBar(Qt.TopToolBarArea, self.dock)
        
        self.dock.boton_stop.clicked.connect(self.stop_robots)
        # Atajo s / S para el STOP en toda la aplicacion (tambien con la
        # ventana de configuracion al frente). Mientras se escribe en un
        # campo de texto la tecla es del campo y el atajo no se dispara.
        self.stop_shortcuts = []
        for secuencia in ("S", "Shift+S"):
            shortcut = QShortcut(QKeySequence(secuencia), self)
            shortcut.setContext(Qt.ApplicationShortcut)
            shortcut.activated.connect(self.stop_robots)
            self.stop_shortcuts.append(shortcut)
        self.estado_timer = QTimer()
        self.estado_timer.timeout.connect(self.actualizar_estado_robots)
        self.estado_timer.start(500)

        # Configurar menú lateral
        self.cargar_iconos()
        self.setup_menu()
        self.set_devices_menu()
        self.set_robot_menu()
        self.set_controller_menu()
        self.set_joint_control()

        # Configurar widget de cámara y estado de los robots
        self.set_camara_widget()
        self.estado = Estado()

        # Ventana central y modos Teleoperacion/Desarrollo (modulos/modos.py).
        # Siempre arranca en Teleoperacion.
        self.modos = ModosPanel(
            menu=self.menu_general,
            rviz=self.rviz_widget,
            camara=self.camara_panel,
            estado=self.estado,
            config_robots=self.robots_config_widget,
            config_dialogo=self.robots_config_dialog,
            slot_config_dialogo=self.slot_config_dialog,
            slot_config_menu=self.slot_config_menu,
            dock=self.dock,
        )
        self.setCentralWidget(self.modos.central)
        self.modos.aplicar(MODO_TELEOP)
        self.dock.set_modo(MODO_TELEOP)
        self.dock.modo_cambiado.connect(self.cambiar_modo)
        # lambda: clicked(bool) pasaria False como 'confirmar'
        self.dock.boton_teleop.clicked.connect(lambda: self.abrir_asistente_teleop())
        # Asistente de teleoperacion al arrancar, ya con la ventana visible
        QTimer.singleShot(0, lambda: self.abrir_asistente_teleop(confirmar=False))

    def set_camara_widget(self):
        """Vista de la camara en la ventana principal (modos.py la ubica
        segun el modo) + boton para abrir un duplicado flotante."""
        self.camara_panel = CamaraPanel()
        self.camara_panel.boton_ventana.clicked.connect(self.abrir_ventana_camara)
        self.video_label = self.camara_panel.video_label

        # self.camera ya existe (creado en __init__ sin labels); ahora que
        # el label real existe, se lo registramos y arrancamos la
        # suscripcion ROS2 + el timer que la bombea.
        self.camera.agregar_label(self.video_label)
        self.camera.iniciar_suscripcion()

    def abrir_ventana_camara(self):
        """Abre (o trae al frente) el duplicado flotante de la camara."""
        if self.ventana_camara is None:
            self.ventana_camara = VentanaCamara()
            self.camera.agregar_label(self.ventana_camara.video_label)
            self.ventana_camara.cerrada.connect(self._ventana_camara_cerrada)
        self.ventana_camara.show()
        self.ventana_camara.raise_()
        self.ventana_camara.activateWindow()

    def _ventana_camara_cerrada(self):
        if self.ventana_camara is not None:
            self.camera.quitar_label(self.ventana_camara.video_label)
            self.ventana_camara.deleteLater()
            self.ventana_camara = None

    def cambiar_modo(self, modo):
        """Toggle del Dock. Si hay controller_node corriendo pide
        confirmacion; al pasar a Teleoperacion ademas ejecuta el STOP
        (no queda una prueba de Desarrollo moviendo el robot). Si se
        cancela, el toggle vuelve a su posicion."""
        corriendo = self._controladores_corriendo()
        if corriendo:
            nombre = "Teleoperación" if modo == MODO_TELEOP else "Desarrollo"
            texto = f"Hay controladores corriendo ({', '.join(corriendo)})."
            if modo == MODO_TELEOP:
                texto += ("\n\nAl pasar a Teleoperación se ejecuta el STOP: se "
                          "detienen los controladores y los robots quedan quietos.")
            respuesta = QMessageBox.question(
                self, "Cambiar de modo", f"{texto}\n\n¿Cambiar a {nombre}?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if respuesta != QMessageBox.Yes:
                self.dock.set_modo(self.modos.modo)
                return
            if modo == MODO_TELEOP:
                self.stop_robots()
        self.modos.aplicar(modo)
        if modo == MODO_TELEOP:
            self.abrir_asistente_teleop(confirmar=False)
        elif self.asistente is not None:
            self.asistente.close()

    def _controladores_corriendo(self):
        """Robots ('R1', 'R2') con un controller_node vivo."""
        corriendo = []
        for robot_id in ("r1", "r2"):
            process = getattr(self, f'{robot_id}_controller_process', None)
            if process is not None and process.poll() is None:
                corriendo.append(robot_id.upper())
        return corriendo

    def abrir_asistente_teleop(self, confirmar=True):
        """Abre (o trae al frente) el asistente de teleoperacion
        (modulos/asistente_teleop.py). La pagina de configuracion solo
        aparece si algun robot no esta lanzado. confirmar: si hay
        controller_node corriendo, pide confirmar y hace STOP antes (al
        llegar desde cambiar_modo eso ya se hizo)."""
        if self.asistente is not None:
            self.asistente.show()
            self.asistente.raise_()
            self.asistente.activateWindow()
            return
        if confirmar:
            corriendo = self._controladores_corriendo()
            if corriendo:
                respuesta = QMessageBox.question(
                    self, "Iniciar teleoperación",
                    f"Hay controladores corriendo ({', '.join(corriendo)}).\n\n"
                    "Para iniciar la teleoperación se ejecuta el STOP: se detienen "
                    "los controladores y los robots quedan quietos. ¿Continuar?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if respuesta != QMessageBox.Yes:
                    return
                self.stop_robots()
        mostrar_config = any(self.robots.estado_proceso(r) != "vivo" for r in ("r1", "r2"))
        self.asistente = AsistenteTeleop(self, mostrar_config)
        self.asistente.show()

    '''
ros2 run ur5_controller controller_node --ros-args \
  -p geomagic:=true \
  -p ur:="ur5e" \
  -p nmspace:="ur5e" \
  -p urdf_path:="/ruta/al/robot.urdf" \
  -p robot_description:="" \
  -p geomagic_topic:="/phantom/state" \
  -p geomagic_button_topic:="/phantom/button" \
  -p use_ur5_pos_init:=true \
  -p q_target:="[0.0, -1.57, 1.57, 0.0, 1.57, 0.0]" \
  -p q_target_time:=3.0 \
  -p csv_log_enable:=true \
  -p csv_log_dir:="/tmp/ur5_logs" \
  -p csv_log_prefix:="run" \
  -p traj_A:="[0.1, 0.1, 0.1]" \
  -p traj_wn:=1.0 \
  -p traj_c0:=0.5 \
  -p traj_mode:=1 \
  -p controller_type:="QP" \
  -p Kp:="[1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 5000.0]" \
  -p Kd:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]" \
  -p lambda:="[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]" \
  -p k:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]" \
  -p k2:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]" \
  -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" \
  -p alpha:=0.01 \
  -p damping_factor:=0.01 \
  -p dt:=0.01 \
  -p ctrl_hz:=500.0 \
  -p max_joint_step_rad:=0.05 \
  -p large_error_threshold_rad:=0.15 \
  -p map_x:=2.0 -p map_y:=0.0 -p map_z:=1.0 \
  -p sign_x:=-1.0 -p sign_y:=-1.0 -p sign_z:=1.0 \
  -p map_roll:=2.0 -p map_pitch:=0.0 -p map_yaw:=1.0 \
  -p sign_roll:=1.0 -p sign_pitch:=1.0 -p sign_yaw:=1.0



    '''    
    def start_controller(self, robot_id, forzar_geomagic=False):
        """Inicia el nodo controlador con los parámetros de la interfaz.
        forzar_geomagic: arranca en teleoperacion (geomagic=true) sin
        importar el modo guardado en el config, y sin modificarlo (lo usa
        el asistente de teleoperacion)."""
        print(f"[R{robot_id} Controller] Iniciando nodo controlador para Robot {robot_id}...")

        control_config = getattr(self, f"{robot_id}_control_config")
        robot_config = getattr(self, f"{robot_id}_config")

        # El controlador necesita el URDF real del robot (con su herramienta,
        # definida en ur5e_single.urdf.xacro) para que la cinemática/dinámica
        # de Pinocchio coincida con lo que ya está corriendo en ur5e_bringup.
        # Se re-genera localmente con xacro (mismos argumentos que usa
        # multi_ur5e.launch.py) en vez de leerlo del tópico /robot_description,
        # para no depender de que el robot ya esté lanzado.
        # Parametros desde la interfaz, con los mismos tipos que antes daba
        # el parser YAML de '-p k:=v' (yaml.safe_load sobre el mismo texto).
        # 'nmspace' lo fija controller.launch.py a partir de robot_id.
        phantom = "phantom1" if robot_id == "r1" else "phantom2"
        params = {
            'control_topic': '/forward_position_controller/commands',
            'ur': robot_config["ur_type"],
            'geomagic': True if forzar_geomagic else yaml.safe_load(str(control_config["geomagic"])),
            # Absolutos: con el nodo en /rN, un topico relativo quedaria
            # como /rN/phantomX/state
            'geomagic_topic': f'/{phantom}/state',
            'geomagic_button_topic': f'/{phantom}/button',
            'csv_log_enable': True,
            'csv_log_prefix': f'ur5_log_{control_config["controller_type"]}',
            'traj_mode': int(control_config["traj_mode"]),
            'q_target': yaml.safe_load(f'[{getattr(self, f"{robot_id}_q_target").text()}]'),
            'controller_type': control_config["controller_type"],
            'lambda': yaml.safe_load(str(control_config["lambda"])),
            'k': yaml.safe_load(str(control_config["k"])),
            'alpha': float(control_config["alpha"]),
            'traj_A': [0.1, 0.1, 0.2],
            'ctrl_hz': float(250),
        }
        for eje in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            params[f'map_{eje}'] = float(control_config[f"map_{eje}"])
            params[f'sign_{eje}'] = float(control_config[f"sign_{eje}"])
        # En Gazebo el joint_trajectory_controller usa el reloj simulado
        # (/clock, arranca en 0): con reloj real, el header.stamp de la
        # trayectoria inicial (InitialMotionPublisher, node->now()) caia ~56
        # años en el futuro y el robot nunca iba a home. Con use_sim_time,
        # now() y el seguimiento de trayectoria usan el tiempo simulado.
        if robot_config.get("mode") == "gazebo":
            params['use_sim_time'] = True

        # El controlador necesita el URDF real del robot (con su herramienta,
        # definida en ur5e_single.urdf.xacro) para que la cinemática/dinámica
        # de Pinocchio coincida con lo que ya está corriendo en ur5e_bringup.
        # Se re-genera localmente con xacro (mismos argumentos que usa
        # multi_ur5e.launch.py) en vez de leerlo del tópico /robot_description,
        # para no depender de que el robot ya esté lanzado.
        try:
            params['robot_description'] = self.robots.generar_robot_description(robot_id, robot_config)
        except Exception as e:
            print(f"[{robot_id} Controller] Error generando robot_description: {e}")
            print(f"[{robot_id} Controller] El controlador arrancará con el URDF genérico "
                  f"de respaldo (sin herramienta) en vez del real del robot.")

        # controller.launch.py pone el nodo en el namespace del robot
        # (/rN/ur5_ik_node): sin eso los dos controladores se llamaban
        # /ur5_ik_node y rqt_graph / ros2 param los veian como uno solo.
        params_file = self.robots.escribir_params_controller(robot_id, params)
        command = [
            'ros2', 'launch', 'ur5e_bringup', 'controller.launch.py',
            f'robot_id:={robot_id}',
            f'params_file:={params_file}',
        ]

        try:
            print(f"[{robot_id} Controller] Comando: {' '.join(command)}")
            # Lanzar el proceso del controlador en su propio grupo
            controller_process = subprocess.Popen(
                command,
                preexec_fn=os.setsid
            )
            setattr(self, f'{robot_id}_controller_process', controller_process)
        except Exception as e:
            print(f"[{robot_id} Controller] Error al iniciar el nodo controlador: {e}")
    
    
    def stop_controller(self, robot_id):
        """Detiene el nodo controlador del robot especificado"""
        controller_process = getattr(self, f'{robot_id}_controller_process', None)
        if controller_process is None:
            print(f"[{robot_id} Controller] No hay proceso de controlador activo para detener.")
            return

        print(f"[{robot_id} Controller] Deteniendo nodo controlador...")
        try:
            pgid = os.getpgid(controller_process.pid)

            # Intento 1: SIGINT (Ctrl+C)
            os.killpg(pgid, signal.SIGINT)
            try:
                controller_process.wait(timeout=5)
                print(f"[{robot_id} Controller] Nodo controlador detenido correctamente.")
                setattr(self, f'{robot_id}_controller_process', None)
                return
            except subprocess.TimeoutExpired:
                print(f"[{robot_id} Controller] No respondió a SIGINT, enviando SIGTERM...")

            # Intento 2: SIGTERM
            os.killpg(pgid, signal.SIGTERM)
            try:
                controller_process.wait(timeout=5)
                print(f"[{robot_id} Controller] Nodo controlador detenido con SIGTERM.")
                setattr(self, f'{robot_id}_controller_process', None)
                return
            except subprocess.TimeoutExpired:
                print(f"[{robot_id} Controller] No respondió a SIGTERM, forzando cierre...")

            # Intento 3: SIGKILL
            os.killpg(pgid, signal.SIGKILL)
            controller_process.wait(timeout=2)
            print(f"[{robot_id} Controller] Nodo controlador terminado forzosamente.")
        except ProcessLookupError:
            print(f"[{robot_id} Controller] El proceso ya no existe.")
        except Exception as e:
            print(f"[{robot_id} Controller] Error al detener el nodo controlador: {e}")
        finally:
            setattr(self, f'{robot_id}_controller_process', None)
    
    def on_r1_controller_tab_changed(self, index):
        """Se ejecuta cuando el usuario cambia de pestaña en el controlador del robot 1"""
        tab_names = ["Controller", "Joints", "Cartesian"]
        print(f"R1 Controller - Cambió a pestaña: {tab_names[index]} (índice {index})")
        
        # Aquí puedes agregar lógica específica según la pestaña
        if index == 0:
            print("  → Modo Controller activo")
        elif index == 1:
            print("  → Modo Joints activo")
        elif index == 2:
            print("  → Modo Cartesian activo")
        
    def cambiar_controller_topic(self, robot_id):
        """Cambia el controlador activo del robot (forward_position_controller <-> scaled_joint_trajectory_controller)"""
        print(f"[{robot_id}] Cambiando controlador activo...")
        process = subprocess.Popen(
                    ['ros2', 'control', 'switch_controllers', '--controller-manager', f'/{robot_id}/controller_manager',
                     '--deactivate', f'/{robot_id}/forward_position_controller',
                     '--activate', f'/{robot_id}/scaled_joint_trajectory_controller'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Crear nuevo grupo de procesos
                )
        self.haptic.single_process = process
        print(f"Haptic launch iniciado (PID: {process.pid})")

    def buscar_dispositivos(self):
        #apagar nodos hápticos antes de buscar
        print("[Main] Stopping any active haptic nodes before searching...")
        self.camera.detener_launch()
        self.haptic.detener_launch()

        print("[Main] Searching for haptic devices...")
        resultado = self.haptic.buscar()
        resultado_camara = buscar_camara()
        print(f"[Main] Search result: {resultado}")
        print("Numero de dispositivos hapticos encontrados:", resultado["num_dispositivos"])

        self.dock.set_haptic(1, self.haptic.haptic1_ready)
        self.dock.set_haptic(2, self.haptic.haptic2_ready)
        if not self.haptic.haptic1_ready:
            self.camera.detener_launch()

        self.haptic.lanzar_launch()
        # >1: no se cuenta la camara de la laptop, >0: si se contaria
        self.camera_ready = resultado_camara["num_dispositivos"] > 1
        self.dock.set_camara(self.camera_ready)
        if self.camera_ready:
            print("Camara encontrada")
            self.camera.lanzar_launch()

    def detener_todos_los_launches(self):
        """Detiene todos los launches activos"""
        print("[Shutdown] Deteniendo todos los procesos launch...")
        # Primero los controladores: si no, un controller_node huerfano
        # sigue comandando el robot despues de cerrar/reiniciar el panel.
        for robot_id in ("r1", "r2"):
            self.stop_controller(robot_id)
        self.camera.detener_launch()
        self.haptic.detener_launch()
        self.robots.detener()
        print("[Shutdown] Todos los launches detenidos")
    
    def iniciar_robots(self):
        """Reinicia los robots: detiene si están corriendo y luego lanza"""
        print("[Robots] Reiniciando robots...")
        self.robots.detener()
        for robot_id in ("r1", "r2"):
            self.monitor.reset(robot_id)
        self.lanzar_robots()

    def actualizar_estado_robots(self):
        """LED de estado de cada robot en la barra superior (cada 0.5 s):
        gris detenido / amarillo lanzado / verde listo / rojo error (ver
        modulos/robot_monitor.py:RobotMonitor.estado)."""
        for robot_id in ("r1", "r2"):
            modo = self.robots.modo(robot_id)
            estado, detalle = self.monitor.estado(
                robot_id, self.robots.estado_proceso(robot_id), modo)
            self.dock.set_robot_state(robot_id, estado, detalle, modo)

    def stop_robots(self):
        """STOP (boton de la barra superior o tecla S): en ambos robots corta
        los controller_node y deja el robot quieto en su posicion actual con
        forward_position_controller activo (RobotMonitor.detener_movimiento).
        No bloquea la UI. No reemplaza el paro de emergencia fisico."""
        print("[STOP] Deteniendo ambos robots...")
        # El STOP corta tambien el asistente de teleoperacion a medio camino
        if self.asistente is not None:
            self.asistente.close()
        self.dock.set_stop_info("Deteniendo...")
        resultados = {}

        def terminado(robot_id, ok, mensaje):
            resultados[robot_id] = "quieto" if ok else mensaje
            print(f"[STOP] {robot_id}: {'OK' if ok else 'FALLO'} - {mensaje}")
            if len(resultados) == 2:
                self.dock.set_stop_info(
                    " | ".join(f"{r.upper()}: {resultados[r]}" for r in ("r1", "r2")))

        for robot_id in ("r1", "r2"):
            # Primero cortar la fuente de comandos, luego cambiar de
            # controlador: si no, controller_node podria volver a mover el
            # robot o pedir su propio cambio de controlador.
            if self._detener_controller_async(robot_id):
                print(f"[STOP] {robot_id}: controller_node detenido")
            self.monitor.detener_movimiento(
                robot_id, lambda ok, msg, r=robot_id: terminado(r, ok, msg))

    def _detener_controller_async(self, robot_id):
        """Version no bloqueante de stop_controller para el STOP: manda
        SIGINT al grupo del controller_node y, si no termina, escala a
        SIGTERM y SIGKILL en segundo plano (QTimer). Devuelve True si habia
        un controller_node corriendo."""
        process = getattr(self, f'{robot_id}_controller_process', None)
        setattr(self, f'{robot_id}_controller_process', None)
        if process is None or process.poll() is not None:
            return False
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return False

        def escalar(senales):
            if process.poll() is not None or not senales:
                return
            try:
                os.killpg(pgid, senales[0])
            except ProcessLookupError:
                return
            QTimer.singleShot(3000, lambda: escalar(senales[1:]))
        QTimer.singleShot(3000, lambda: escalar([signal.SIGTERM, signal.SIGKILL]))
        return True

    def lanzar_robots(self):
        """Lanza ambos robots con ur5e_bringup: multi_ur5e.launch.py para los
        que estan en Simulation/Real y multi_ur5e_sim.launch.py para los que
        estan en Gazebo (ver modulos/robots_launch.py)."""
        launch_feedback = "true"
        if hasattr(self, 'feedback_checkbox'):
            launch_feedback = "true" if self.feedback_checkbox.isChecked() else "false"

        # NOTA: launch_feedback / initial_controller por robot ya no tienen
        # equivalente en ur5e_bringup (multi_ur5e.launch.py no los soporta
        # todavia); quedan pendientes de migrar, ver aviso en consola.
        if launch_feedback == "true":
            print("[Robots] Aviso: el feedback de fuerza (ur5_torque) no se "
                  "lanza con ur5e_bringup todavia; el checkbox no tiene efecto.")

        self.robots.lanzar(self.r1_config, self.r2_config)
        self.robots_running = self.robots.running

    def shutdown(self):
        print("[Main] Application closing...")
        
        try:
            # Detener timer de ROS y destruir nodo de cámara
            if getattr(self, 'ventana_camara', None) is not None:
                self.ventana_camara.close()
            if hasattr(self, 'camera'):
                self.camera.detener_todo()
        except Exception as e:
            print(f"[Shutdown] Error deteniendo cámara: {e}")

        try:
            self.detener_todos_los_launches()
        except Exception as e:
            print(f"[Shutdown] Error deteniendo launches: {e}")
        
        try:
            if hasattr(self, 'rviz_widget') and hasattr(self.rviz_widget, 'shutdown'):
                self.rviz_widget.shutdown()
        except Exception as e:
            print(f"[Shutdown] Error cerrando RViz: {e}")
        
        try:
            if hasattr(self, 'estado_timer'):
                self.estado_timer.stop()
            if hasattr(self, 'monitor'):
                self.monitor.destroy()
        except Exception as e:
            print(f"[Shutdown] Error cerrando el monitor de robots: {e}")

        try:
            if hasattr(self, 'estado'):
                self.estado.cerrar()
        except Exception as e:
            print(f"[Shutdown] Error cerrando el panel de estado: {e}")

        try:
            # Shutdown ROS2
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[Shutdown] Error cerrando ROS2: {e}")
        
        print("[Shutdown] Limpieza completada")

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

def main():
    """Entry point for ros2 run command"""
    app = QApplication(sys.argv)
    window = None

    # Cargar y aplicar style.qss desde el share directory del paquete (compatible con install/)
    qss_path = None
    if get_package_share_directory is not None:
        try:
            qss_path = os.path.join(get_package_share_directory('ur5_panel'), 'config', 'style.qss')
        except PackageNotFoundError:
            qss_path = None
    if qss_path is None:
        qss_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/style.qss'))
    if os.path.exists(qss_path):
        print("Style cargado desde:", qss_path)
        with open(qss_path, 'r') as f:
            app.setStyleSheet(f.read())
    else:
        print(f"[Warning] No se encontró style.qss en: {qss_path}")

    # Manejar excepciones no capturadas
    def exception_hook(exctype, value, traceback_obj):
        """Asegurar limpieza en caso de excepción no manejada"""
        print(f"[Fatal Error] {exctype.__name__}: {value}")
        import traceback
        traceback.print_exception(exctype, value, traceback_obj)
        if window is not None:
            window.shutdown()
        sys.__excepthook__(exctype, value, traceback_obj)

    sys.excepthook = exception_hook

    try:
        window = InterfazRviz()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"[Fatal] Error durante la ejecución: {e}")
        if window is not None:
            window.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()