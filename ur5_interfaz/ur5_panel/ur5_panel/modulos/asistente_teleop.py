"""Asistente de teleoperacion (ventana flotante no modal: el STOP de la
barra superior sigue disponible). Lo abre panel.py al arrancar, al pasar de
Desarrollo a Teleoperacion y con el boton 'Iniciar teleoperación'.

Paginas, sin boton Atras (cada 'Siguiente' hace algo: lanzar, mover...):
  1. Configuracion: muestra la config de robots (el mismo widget de la
     ventana 'Configurar robots', prestado) y 'Lanzar robots' los lanza.
     Solo aparece si algun robot no esta lanzado.
  2. Dispositivos: hapticos, camara y estado de los robots. Si no hay al
     menos un haptico o algun robot no esta listo, ofrece seguir en modo
     Desarrollo.
  3. Home: enviar ambos robots a home u omitir. En los dos casos se
     asegura forward_position_controller activo al final.
  4. Controladores: lanza controller_node con geomagic=true (solo para este
     arranque) en cada robot con haptico detectado (haptico 1 -> r1,
     haptico 2 -> r2).
  5. Final: presionar el boton gris de cada haptico para empezar; cada
     robot empieza por su cuenta.

Usa del panel: robots, monitor, haptic, camera_ready, r{id}_control_config,
robots_config_widget/robots_config_dialog, modos, iniciar_robots,
buscar_dispositivos, home_controller, start_controller/stop_controller,
r{id}_controller_process y cambiar_modo.
"""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWizard,
    QWizardPage,
)

from ur5_panel.modulos.dock import (
    COLOR_FALLA, COLOR_GRIS, COLOR_OK, ROBOT_STATE_COLORS, _led, _set_led)
from ur5_panel.modulos.modos import MODO_DESARROLLO

ROBOT_IDS = ("r1", "r2")
HAPTICO = {"r1": 1, "r2": 2}
# Tiempo tras lanzar controller_node para revisar que siga vivo
CONTROLADOR_CHECK_MS = 3000
REFRESCO_MS = 300


def haptico_listo(panel, robot_id):
    return panel.haptic.haptic1_ready if robot_id == "r1" else panel.haptic.haptic2_ready


def robots_teleop(panel):
    """Robots que se teleoperan: los que tienen su haptico detectado."""
    return [r for r in ROBOT_IDS if haptico_listo(panel, r)]


def estado_robot(panel, robot_id):
    """(estado, detalle) como el LED de la barra superior."""
    return panel.monitor.estado(
        robot_id, panel.robots.estado_proceso(robot_id), panel.robots.modo(robot_id))


def _texto(label, texto, color=None):
    label.setText(texto)
    label.setStyleSheet(f"color: {color};" if color else "")


class _Pagina(QWizardPage):
    """Base: activa=False cuando el asistente se cierra, para que los
    callbacks pendientes (home, timers) no toquen widgets ya cerrados."""

    def __init__(self, panel, titulo, subtitulo=""):
        super().__init__()
        self.panel = panel
        self.activa = True
        self.setTitle(titulo)
        if subtitulo:
            self.setSubTitle(subtitulo)


class PaginaConfig(_Pagina):
    def __init__(self, panel):
        super().__init__(panel, "Teleoperación",
                         "Se comenzará con la teleoperación. Revise la configuración "
                         "de los robots; 'Lanzar robots' los inicia con esta configuración.")
        self.setButtonText(QWizard.NextButton, "Lanzar robots")
        layout = QVBoxLayout(self)
        self.slot = QVBoxLayout()
        layout.addLayout(self.slot)
        self.resumen = QLabel()
        self.resumen.setWordWrap(True)
        layout.addWidget(self.resumen)

    def initializePage(self):
        self.panel.robots_config_dialog.hide()
        self.slot.addWidget(self.panel.robots_config_widget)
        self.panel.robots_config_widget.show()
        lineas = []
        for r in ROBOT_IDS:
            control = getattr(self.panel, f"{r}_control_config")
            lineas.append(f"<b>{r.upper()}</b>: controlador {control['controller_type']}, "
                          f"home {control['q_target']}")
        self.resumen.setText("<br>".join(lineas))

    def validatePage(self):
        self.panel.modos.ubicar_config()
        self.panel.iniciar_robots()
        return True


class PaginaDispositivos(_Pagina):
    def __init__(self, panel):
        super().__init__(panel, "Dispositivos",
                         "Se necesita al menos un háptico y ambos robots listos. "
                         "Háptico 1 controla R1 y háptico 2 controla R2.")
        layout = QVBoxLayout(self)
        grid = QGridLayout()
        self.filas = {}
        nombres = [("haptico1", "Háptico 1"), ("haptico2", "Háptico 2"), ("camara", "Cámara")]
        nombres += [(r, f"Robot {r.upper()}") for r in ROBOT_IDS]
        for i, (clave, nombre) in enumerate(nombres):
            led, texto = _led(), QLabel()
            grid.addWidget(led, i, 0)
            grid.addWidget(QLabel(nombre), i, 1)
            grid.addWidget(texto, i, 2)
            self.filas[clave] = (led, texto)
        grid.setColumnStretch(2, 1)
        layout.addLayout(grid)

        self.boton_buscar = QPushButton("Buscar dispositivos")
        self.boton_buscar.clicked.connect(self.buscar)
        fila = QHBoxLayout()
        fila.addWidget(self.boton_buscar)
        fila.addStretch()
        layout.addLayout(fila)
        self.aviso = QLabel()
        self.aviso.setWordWrap(True)
        layout.addWidget(self.aviso)
        layout.addStretch()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refrescar)

    def initializePage(self):
        self.refrescar()
        self.timer.start(REFRESCO_MS)

    def buscar(self):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.panel.buscar_dispositivos()
        finally:
            QApplication.restoreOverrideCursor()
        self.refrescar()

    def _fila(self, clave, color, texto):
        led, label = self.filas[clave]
        _set_led(led, color)
        label.setText(texto)

    def refrescar(self):
        for n in (1, 2):
            ok = haptico_listo(self.panel, f"r{n}")
            self._fila(f"haptico{n}", COLOR_OK if ok else COLOR_FALLA,
                       "detectado" if ok else "no detectado")
        camara = getattr(self.panel, "camera_ready", False)
        self._fila("camara", COLOR_OK if camara else COLOR_GRIS,
                   "detectada" if camara else "no detectada (no es necesaria)")
        for r in ROBOT_IDS:
            estado, detalle = estado_robot(self.panel, r)
            self._fila(r, ROBOT_STATE_COLORS[estado], detalle)
        if self.lanzando():
            self.aviso.setText("Esperando a que los robots terminen de iniciar...")
        else:
            self.aviso.setText("")
        self.completeChanged.emit()

    def lanzando(self):
        return any(estado_robot(self.panel, r)[0] == "lanzado" for r in ROBOT_IDS)

    def isComplete(self):
        return not self.lanzando()

    def validatePage(self):
        faltan = []
        if not robots_teleop(self.panel):
            faltan.append("ningún háptico detectado")
        for r in ROBOT_IDS:
            estado, detalle = estado_robot(self.panel, r)
            if estado != "listo":
                faltan.append(f"{r.upper()} no está listo ({detalle})")
        if not faltan:
            self.timer.stop()
            return True
        respuesta = QMessageBox.question(
            self, "Faltan elementos",
            "Faltan elementos por activar:\n  - " + "\n  - ".join(faltan) +
            "\n\nNo se puede iniciar la teleoperación. ¿Continuar en modo Desarrollo?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if respuesta == QMessageBox.Yes:
            # Fuera de validatePage: cerrar el asistente aqui adentro
            # confunde a QWizard.
            QTimer.singleShot(0, self.wizard().continuar_en_desarrollo)
        return False


class PaginaHome(_Pagina):
    def __init__(self, panel):
        super().__init__(panel, "Home",
                         "¿Enviar ambos robots a home? Verifique que el espacio de "
                         "trabajo esté libre. Al terminar (o al omitir) cada robot queda "
                         "quieto con forward_position_controller activo.")
        layout = QVBoxLayout(self)
        self.estados = {}
        grid = QGridLayout()
        for i, r in enumerate(ROBOT_IDS):
            grid.addWidget(QLabel(f"<b>{r.upper()}</b>"), i, 0)
            self.estados[r] = QLabel()
            self.estados[r].setWordWrap(True)
            grid.addWidget(self.estados[r], i, 1)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

        fila = QHBoxLayout()
        self.boton_home = QPushButton("Enviar a home")
        self.boton_omitir = QPushButton("Omitir")
        self.boton_home.clicked.connect(lambda: self.ejecutar(True))
        self.boton_omitir.clicked.connect(lambda: self.ejecutar(False))
        fila.addWidget(self.boton_home)
        fila.addWidget(self.boton_omitir)
        fila.addStretch()
        layout.addLayout(fila)
        layout.addStretch()
        self.resultados = {}

    def initializePage(self):
        self.resultados = {}
        for r in ROBOT_IDS:
            q = getattr(self.panel, f"{r}_control_config")["q_target"]
            _texto(self.estados[r], f"home {q}")

    def ejecutar(self, home):
        self.boton_home.setEnabled(False)
        self.boton_omitir.setEnabled(False)
        self.resultados = {}
        self.completeChanged.emit()
        for r in ROBOT_IDS:
            callback = lambda ok, msg, r=r: self.terminado(r, ok, msg)
            if home:
                _texto(self.estados[r], "Enviando a home...")
                self.panel.home_controller(r, callback)
            else:
                _texto(self.estados[r], "Activando forward_position_controller...")
                self.panel.monitor.detener_movimiento(r, callback)

    def terminado(self, robot_id, ok, mensaje):
        if not self.activa:
            return
        self.resultados[robot_id] = ok
        _texto(self.estados[robot_id], ("✓ " if ok else "✗ ") + mensaje,
               None if ok else COLOR_FALLA)
        if len(self.resultados) == len(ROBOT_IDS):
            if not all(self.resultados.values()):
                # Permite reintentar u omitir
                self.boton_home.setEnabled(True)
                self.boton_omitir.setEnabled(True)
            self.completeChanged.emit()

    def isComplete(self):
        return len(self.resultados) == len(ROBOT_IDS) and all(self.resultados.values())


class PaginaControladores(_Pagina):
    def __init__(self, panel):
        super().__init__(panel, "Controladores",
                         "Se inicia el controlador en modo teleoperación (con el tipo de "
                         "controlador guardado) en cada robot con háptico detectado.")
        layout = QGridLayout(self)
        self.estados = {}
        for i, r in enumerate(ROBOT_IDS):
            layout.addWidget(QLabel(f"<b>{r.upper()}</b>"), i, 0)
            self.estados[r] = QLabel()
            self.estados[r].setWordWrap(True)
            layout.addWidget(self.estados[r], i, 1)
        layout.setColumnStretch(1, 1)
        layout.setRowStretch(len(ROBOT_IDS), 1)
        self.revisado = False
        self.corriendo = []

    def initializePage(self):
        self.revisado = False
        self.corriendo = []
        teleop = robots_teleop(self.panel)
        for r in ROBOT_IDS:
            if r not in teleop:
                _texto(self.estados[r], f"sin háptico {HAPTICO[r]}: no se inicia")
                continue
            tipo = getattr(self.panel, f"{r}_control_config")["controller_type"]
            proceso = getattr(self.panel, f"{r}_controller_process", None)
            if proceso is not None and proceso.poll() is None:
                # Uno anterior (p. ej. de Desarrollo) que no termino aun
                self.panel.stop_controller(r)
            _texto(self.estados[r], f"Iniciando controller_node ({tipo}, teleoperación)...")
            self.panel.start_controller(r, forzar_geomagic=True)
        QTimer.singleShot(CONTROLADOR_CHECK_MS, lambda: self.revisar(teleop))

    def revisar(self, teleop):
        if not self.activa:
            return
        for r in teleop:
            proceso = getattr(self.panel, f"{r}_controller_process", None)
            if proceso is not None and proceso.poll() is None:
                self.corriendo.append(r)
                _texto(self.estados[r], "✓ controller_node corriendo")
            else:
                codigo = None if proceso is None else proceso.poll()
                _texto(self.estados[r], f"✗ controller_node no está corriendo "
                                        f"(código {codigo}); ver la consola", COLOR_FALLA)
        self.revisado = True
        self.completeChanged.emit()

    def isComplete(self):
        return self.revisado and bool(self.corriendo)


class PaginaFinal(_Pagina):
    def __init__(self, panel):
        super().__init__(panel, "Comenzar teleoperación",
                         "Para comenzar, presione el botón gris de cada háptico. Cada robot "
                         "empieza a seguir a su háptico al presionar su botón.")
        layout = QGridLayout(self)
        self.estados = {}
        for i, r in enumerate(ROBOT_IDS):
            layout.addWidget(QLabel(f"<b>{r.upper()}</b> (háptico {HAPTICO[r]})"), i, 0)
            self.estados[r] = QLabel()
            layout.addWidget(self.estados[r], i, 1)
        layout.setColumnStretch(1, 1)
        layout.setRowStretch(len(ROBOT_IDS), 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refrescar)
        self.robots = []

    def initializePage(self):
        self.robots = self.wizard().pagina_controladores.corriendo
        for r in self.robots:
            self.panel.monitor.reset_boton_gris(r)
        self.refrescar()
        self.timer.start(REFRESCO_MS)

    def refrescar(self):
        for r in ROBOT_IDS:
            if r not in self.robots:
                _texto(self.estados[r], "sin controlador")
            elif self.panel.monitor.boton_gris(r):
                _texto(self.estados[r], "✓ teleoperando", COLOR_OK)
            else:
                _texto(self.estados[r], "esperando el botón gris...")


class AsistenteTeleop(QWizard):
    def __init__(self, panel, mostrar_config):
        super().__init__(panel)
        self.panel = panel
        self.setWindowTitle("Asistente de teleoperación")
        self.setModal(False)
        self.setAttribute(Qt.WA_DeleteOnClose)
        # Sin Atras: cada paso tiene efectos (lanzar, mover, arrancar).
        self.setButtonLayout([QWizard.Stretch, QWizard.NextButton,
                              QWizard.FinishButton, QWizard.CancelButton])
        self.setButtonText(QWizard.NextButton, "Siguiente")
        self.setButtonText(QWizard.FinishButton, "Finalizar")
        self.setButtonText(QWizard.CancelButton, "Cerrar")

        self.paginas = []
        if mostrar_config:
            self.paginas.append(PaginaConfig(panel))
        self.pagina_controladores = PaginaControladores(panel)
        self.paginas += [PaginaDispositivos(panel), PaginaHome(panel),
                         self.pagina_controladores, PaginaFinal(panel)]
        for pagina in self.paginas:
            self.addPage(pagina)
        self.resize(700, 600 if mostrar_config else 400)
        self.finished.connect(self._cerrado)

    def _cerrado(self):
        for pagina in self.paginas:
            pagina.activa = False
        # Por si se cerro estando en la pagina de configuracion
        self.panel.modos.ubicar_config()
        self.panel.asistente = None

    def continuar_en_desarrollo(self):
        self.reject()
        self.panel.dock.set_modo(MODO_DESARROLLO)
        self.panel.cambiar_modo(MODO_DESARROLLO)
