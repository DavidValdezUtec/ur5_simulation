"""Widgets de la camara (los frames los pinta CameraModule, camera.py):

  - CamaraPanel: vista dentro de la ventana principal, con el boton
    'Ventana aparte'. modos.py la mueve entre la columna grande
    (Teleoperacion) y la chica (Desarrollo).
  - VentanaCamara: duplicado flotante, una ventana normal (sin Qt.Tool ni
    siempre-encima) para llevarla a otro monitor. F11 alterna pantalla
    completa, Esc la deja.
"""
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QShortcut,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtGui import QKeySequence


def crear_video_label():
    """QLabel negro donde CameraModule pinta los frames. Con politica
    Ignored el pixmap no impone su tamaño al label: si no, al reescalar el
    frame al tamaño del label en cada cuadro, el label crece sin parar."""
    label = QLabel("Esperando video de cámara...")
    label.setAlignment(Qt.AlignCenter)
    label.setStyleSheet("background-color: black; color: white; font-size: 14px;")
    label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
    label.setMinimumSize(160, 120)
    return label


class CamaraPanel(QWidget):
    """Vista de la camara en la ventana principal. panel.py conecta
    boton_ventana para abrir el duplicado flotante."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        barra = QHBoxLayout()
        barra.addWidget(QLabel("<b>Cámara</b>"))
        barra.addStretch()
        self.boton_ventana = QPushButton("Ventana aparte")
        self.boton_ventana.setToolTip(
            "Abre un duplicado de la cámara en una ventana propia, para "
            "llevarla a otro monitor (F11: pantalla completa).")
        barra.addWidget(self.boton_ventana)
        layout.addLayout(barra)

        self.video_label = crear_video_label()
        layout.addWidget(self.video_label, 1)


class VentanaCamara(QWidget):
    """Duplicado flotante de la camara. Sin padre, para que sea una ventana
    independiente; panel.py la cierra al salir. Emite cerrada al cerrarse
    para que se deje de pintar en su label."""

    cerrada = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cámara")
        self.resize(800, 600)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.video_label = crear_video_label()
        layout.addWidget(self.video_label)

        QShortcut(QKeySequence(Qt.Key_F11), self, self.alternar_pantalla_completa)
        QShortcut(QKeySequence(Qt.Key_Escape), self, self.showNormal)

    def alternar_pantalla_completa(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def closeEvent(self, event):
        self.cerrada.emit()
        event.accept()


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    panel = CamaraPanel()
    panel.resize(480, 400)
    panel.show()
    ventana = VentanaCamara()
    panel.boton_ventana.clicked.connect(ventana.show)
    sys.exit(app.exec_())
