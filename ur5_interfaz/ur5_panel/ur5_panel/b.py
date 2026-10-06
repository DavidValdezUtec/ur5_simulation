"""Visor de camara USB con PyQt5 + OpenCV (sin QtMultimedia, que no esta
instalado). Un QTimer lee frames con cv2.VideoCapture y los pinta en un
QLabel. Ejecutar con: /usr/bin/python3 b.py"""
import glob
import os
import sys
import time

import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QAction, QApplication, QComboBox, QFileDialog,
                             QLabel, QMainWindow, QMessageBox, QStatusBar,
                             QToolBar)


def buscar_camaras():
    """Indices de /dev/videoN que realmente entregan imagen (cada camara UVC
    suele crear dos nodos, uno de ellos solo de metadatos)."""
    indices = []
    for dev in sorted(glob.glob('/dev/video*')):
        i = int(dev.replace('/dev/video', ''))
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened() and cap.read()[0]:
            indices.append(i)
        cap.release()
    return indices


# Candidatas a probar; solo se muestran las que la camara acepta.
RESOLUCIONES = [(1920, 1080), (1280, 720), (1024, 576), (960, 540),
                (800, 600), (640, 480), (640, 360), (320, 240)]


def abrir_camara(index, w=None, h=None):
    """Abre /dev/videoN en MJPG (en YUYV muchas webcams USB no pasan de
    640x480 por ancho de banda). El FOURCC debe fijarse ANTES que el tamano."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if w is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


def resoluciones_soportadas(cap):
    """Pide cada resolucion candidata y se queda con las que el driver
    devuelve tal cual (V4L2 ajusta a la mas cercana si no la soporta)."""
    soportadas = []
    for w, h in RESOLUCIONES:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) == (w, h):
            soportadas.append((w, h))
    return soportadas


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle("PyQt5 Cam")

        self.cap = None
        self.frame = None          # ultimo frame BGR, para guardar fotos
        self.save_path = ""
        self.save_seq = 0

        self.available_cameras = buscar_camaras()
        if not self.available_cameras:
            QMessageBox.critical(self, "Cámara", "No se encontró ninguna cámara")
            sys.exit(1)

        self.status = QStatusBar()
        self.setStatusBar(self.status)

        self.video_label = QLabel("Esperando video de cámara...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setMinimumSize(320, 240)
        self.setCentralWidget(self.video_label)

        toolbar = QToolBar("Camera Tool Bar")
        self.addToolBar(toolbar)

        click_action = QAction("Tomar foto", self)
        click_action.setStatusTip("Guarda el frame actual como imagen")
        click_action.triggered.connect(self.click_photo)
        toolbar.addAction(click_action)

        change_folder_action = QAction("Cambiar carpeta", self)
        change_folder_action.setStatusTip("Carpeta donde se guardan las fotos")
        change_folder_action.triggered.connect(self.change_folder)
        toolbar.addAction(change_folder_action)

        camera_selector = QComboBox()
        camera_selector.setToolTip("Seleccionar cámara")
        camera_selector.addItems([f"/dev/video{i}" for i in self.available_cameras])
        camera_selector.currentIndexChanged.connect(self.select_camera)
        toolbar.addWidget(camera_selector)

        self.resolution_selector = QComboBox()
        self.resolution_selector.setToolTip("Resolución")
        self.resolution_selector.currentIndexChanged.connect(self.select_resolution)
        toolbar.addWidget(self.resolution_selector)

        # Timer que lee y pinta frames (~30 fps)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        self.select_camera(0)
        self.show()

    def select_camera(self, i):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()

        self.camera_index = self.available_cameras[i]
        self.cap = abrir_camara(self.camera_index)
        if not self.cap.isOpened():
            self.alert(f"No se pudo abrir /dev/video{self.camera_index}")
            return

        self.current_camera_name = f"video{self.camera_index}"
        self.save_seq = 0

        # Se rellena el combo sin señales y luego select_resolution reabre la camara
        # con la resolucion elegida (por defecto la mayor) y arranca el timer.
        self.resoluciones = resoluciones_soportadas(self.cap)
        self.cap.release()
        self.cap = None
        self.resolution_selector.blockSignals(True)
        self.resolution_selector.clear()
        self.resolution_selector.addItems([f"{w}x{h}" for w, h in self.resoluciones])
        self.resolution_selector.blockSignals(False)
        self.select_resolution(0)

    def select_resolution(self, i):
        if i < 0 or i >= len(self.resoluciones):
            return
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()

        w, h = self.resoluciones[i]
        self.cap = abrir_camara(self.camera_index, w, h)
        if not self.cap.isOpened():
            self.alert(f"No se pudo abrir /dev/video{self.camera_index}")
            return
        self.status.showMessage(f"/dev/video{self.camera_index} a {w}x{h}")
        self.timer.start(30)

    def update_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.status.showMessage("No se recibe imagen de la cámara")
            return
        self.frame = frame

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.video_label.width(), self.video_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_label.setPixmap(pixmap)

    def click_photo(self):
        if self.frame is None:
            return
        timestamp = time.strftime("%d-%b-%Y-%H_%M_%S")
        path = os.path.join(self.save_path, "%s-%04d-%s.jpg" % (
            self.current_camera_name, self.save_seq, timestamp))
        if cv2.imwrite(path, self.frame):
            self.status.showMessage("Imagen guardada: " + path)
            self.save_seq += 1
        else:
            self.alert("No se pudo guardar " + path)

    def change_folder(self):
        path = QFileDialog.getExistingDirectory(self, "Carpeta de fotos", "")
        if path:
            self.save_path = path
            self.save_seq = 0

    def alert(self, msg):
        QMessageBox.warning(self, "Cámara", msg)

    def closeEvent(self, event):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())
