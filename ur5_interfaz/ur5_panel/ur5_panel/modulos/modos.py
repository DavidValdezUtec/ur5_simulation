"""Modos del panel y armado de la ventana central.

  - Teleoperacion (MODO_TELEOP): camara grande, RViz y debajo el estado de
    los robots. 'Configurar robots' abre la ventana de configuracion.
  - Desarrollo (MODO_DESARROLLO): menu lateral (la config de robots pasa a
    su tab 1), RViz con el area de graficas debajo, y a la derecha la
    camara chica con el estado debajo.

RViz (OGRE/OpenGL) nunca cambia de padre: reparentarlo puede perder el
contexto grafico o tirar la app. Por eso la estructura de splitters es
fija; al cambiar de modo solo se muestran/ocultan contenedores y se mueven
entre ellos los widgets Qt normales (camara, estado, config de robots).

    splitter (horizontal)
     ├─ menu                         Desarrollo
     ├─ caja_camara_grande           Teleop
     ├─ centro (vertical)
     │   ├─ rviz                     siempre
     │   ├─ caja_estado_teleop       Teleop
     │   └─ area_graficas            Desarrollo
     └─ derecha (vertical)           Desarrollo
         ├─ caja_camara_chica
         └─ caja_estado_dev

Este modulo no decide si se puede cambiar de modo (confirmacion/STOP con
controladores corriendo): eso lo hace panel.py:cambiar_modo antes de
llamar a aplicar().
"""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

MODO_TELEOP = "teleop"
MODO_DESARROLLO = "desarrollo"

# Proporciones iniciales de los splitters por modo (QSplitter las escala
# al tamaño real). Despues se recuerdan las que deje el usuario, solo
# mientras el panel este abierto.
TAMANOS_INICIALES = {
    MODO_TELEOP: {
        "splitter": [0, 600, 500, 0],
        "centro": [700, 300, 0],
        "derecha": [0, 0],
    },
    MODO_DESARROLLO: {
        "splitter": [350, 0, 900, 350],
        "centro": [700, 0, 300],
        "derecha": [250, 550],
    },
}


def _caja():
    """Contenedor sin margenes donde se coloca un widget movible."""
    caja = QWidget()
    layout = QVBoxLayout(caja)
    layout.setContentsMargins(0, 0, 0, 0)
    return caja


class ModosPanel:
    """Arma la ventana central (self.central) y aplica cada modo.

    menu: QTabWidget lateral. rviz: widget de RViz. camara: CamaraPanel.
    estado: Estado (modulos/estado.py). config_robots: widget con la config
    de r1/r2; vive en slot_config_dialogo (layout dentro de config_dialogo)
    en Teleop y en slot_config_menu (layout del tab 1) en Desarrollo.
    dock: barra superior (se oculta boton_config en Desarrollo).
    """

    def __init__(self, menu, rviz, camara, estado, config_robots,
                 config_dialogo, slot_config_dialogo, slot_config_menu, dock):
        self.menu = menu
        self.rviz = rviz
        self.camara = camara
        self.estado = estado
        self.config_robots = config_robots
        self.config_dialogo = config_dialogo
        self.slot_config_dialogo = slot_config_dialogo
        self.slot_config_menu = slot_config_menu
        self.dock = dock
        self.modo = None
        self._tamanos = {modo: dict(t) for modo, t in TAMANOS_INICIALES.items()}

        self.caja_camara_grande = _caja()
        self.caja_estado_teleop = _caja()
        self.caja_camara_chica = _caja()
        self.caja_estado_dev = _caja()

        # Lugar para las graficas de Desarrollo (por definir): agregar los
        # widgets a area_graficas.layout().
        self.area_graficas = _caja()
        placeholder = QLabel("Gráficas (por definir)")
        placeholder.setAlignment(Qt.AlignCenter)
        self.area_graficas.layout().addWidget(placeholder)

        self.centro = QSplitter(Qt.Vertical)
        self.centro.addWidget(self.rviz)
        self.centro.addWidget(self.caja_estado_teleop)
        self.centro.addWidget(self.area_graficas)

        self.derecha = QSplitter(Qt.Vertical)
        self.derecha.addWidget(self.caja_camara_chica)
        self.derecha.addWidget(self.caja_estado_dev)

        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.menu)
        self.splitter.addWidget(self.caja_camara_grande)
        self.splitter.addWidget(self.centro)
        self.splitter.addWidget(self.derecha)
        for splitter in (self.splitter, self.centro, self.derecha):
            splitter.setChildrenCollapsible(False)

        self.central = self.splitter

    def _splitters(self):
        return {"splitter": self.splitter, "centro": self.centro, "derecha": self.derecha}

    def aplicar(self, modo):
        """Muestra el modo pedido (MODO_TELEOP o MODO_DESARROLLO)."""
        if modo == self.modo:
            return
        if self.modo is not None:
            self._tamanos[self.modo] = {
                nombre: s.sizes() for nombre, s in self._splitters().items()}

        teleop = modo == MODO_TELEOP
        if teleop:
            self.caja_camara_grande.layout().addWidget(self.camara)
            self.caja_estado_teleop.layout().addWidget(self.estado)
            self.estado.set_orientacion(Qt.Horizontal)
            self.slot_config_dialogo.addWidget(self.config_robots)
        else:
            self.config_dialogo.hide()
            self.caja_camara_chica.layout().addWidget(self.camara)
            self.caja_estado_dev.layout().addWidget(self.estado)
            self.estado.set_orientacion(Qt.Vertical)
            self.slot_config_menu.addWidget(self.config_robots)
        # addWidget lo saca del contenedor anterior; show() por si estaba
        # oculto con su padre anterior.
        for widget in (self.camara, self.estado, self.config_robots):
            widget.show()

        self.menu.setVisible(not teleop)
        self.caja_camara_grande.setVisible(teleop)
        self.caja_estado_teleop.setVisible(teleop)
        self.area_graficas.setVisible(not teleop)
        self.derecha.setVisible(not teleop)
        self.dock.boton_config.setVisible(teleop)

        for nombre, splitter in self._splitters().items():
            splitter.setSizes(self._tamanos[modo][nombre])
        self.modo = modo
        print(f"[Modo] {'Teleoperación' if teleop else 'Desarrollo'}")
