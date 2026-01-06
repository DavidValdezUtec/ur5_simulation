import sys

from PySide6.QtCore import Slot
from PySide6.QtWidgets import (
	QApplication,
	QMainWindow,
	QWidget,
	QVBoxLayout,
	QPushButton,
)
from pathlib import Path


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("UR5 Control Panel")
        self.resize(800, 800)

        central = QWidget(self)
        layout_exit = QVBoxLayout(central)

        self.exit_button = QPushButton("Salir", central)
        self.exit_button.setObjectName("exit_button")
        self.exit_button.clicked.connect(self.on_exit_clicked)
        layout_exit.addWidget(self.exit_button)

        self.setCentralWidget(central)

    @Slot()
    def on_exit_clicked(self) -> None:
        app = QApplication.instance()
        if app is not None:
            app.quit()


def main() -> int:
    app = QApplication(sys.argv)
    apply_stylesheet(app)
    window = MainWindow()
    window.show()
    return app.exec()


def apply_stylesheet(app: QApplication) -> None:
    """Carga el stylesheet desde resource/style.qss del paquete."""
    try:
        # Ruta al archivo style.qss en la carpeta resource del paquete
        qss_path = Path(__file__).resolve().parent.parent / "resource" / "style.qss"
        if qss_path.exists():
            app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"Warning: No se pudo cargar style.qss: {e}")


if __name__ == "__main__":
    sys.exit(main())


