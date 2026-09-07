"""Todo lo relacionado a la camara: el nodo ROS2 suscriptor, su volcado al
video_label de la UI, y el proceso 'ros2 launch ur5_bringup
launch_camera.launch.py'. CameraModule es un objeto de composicion: no
hereda de InterfazRviz, solo recibe el widget que debe actualizar."""
import os
import subprocess

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

from .procesos import terminar_proceso_gracefully


class CameraSubscriber(Node):
    """Nodo ROS2 para suscribirse al tópico de la cámara"""
    def __init__(self, callback):
        super().__init__('camera_subscriber_node')
        self.bridge = CvBridge()
        self.callback = callback
        self.subscription = self.create_subscription(
            Image,
            '/camera/usb/image_raw',  # Ajusta este tópico según tu configuración
            self.image_callback,
            10
        )
        self.get_logger().info('Camera subscriber initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.callback(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')


class CameraModule:
    """Encapsula el nodo suscriptor de camara y el proceso de su launch.

    video_label: el QLabel donde se pinta cada frame (creado por la UI,
    inyectado aqui en vez de que este modulo construya su propio widget).
    """

    def __init__(self, video_label):
        self.video_label = video_label
        self.node = None
        self.ros_timer = None
        self.process = None

    def iniciar_suscripcion(self):
        """Crea el nodo suscriptor y el timer que bombea rclpy.spin_once."""
        self.node = CameraSubscriber(self.update_video)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self.ros_timer.start(30)  # 30ms (~33 fps)

    def update_video(self, cv_image):
        """Actualiza el widget de video con una nueva imagen de OpenCV."""
        try:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.video_label.width(),
                self.video_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.video_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Error updating video: {e}")

    def lanzar_launch(self):
        """Lanza el archivo launch de la cámara."""
        if self.process is not None:
            print("Camera launch ya está corriendo")
            return
        try:
            self.process = subprocess.Popen(
                ['ros2', 'launch', 'ur5_bringup', 'launch_camera.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            print(f"Camera launch iniciado (PID: {self.process.pid})")
        except Exception as e:
            print(f"Error al lanzar camera: {e}")

    def detener_launch(self):
        """Detiene el launch de la cámara."""
        if self.process is not None:
            terminar_proceso_gracefully(self.process, 'camera')
            self.process = None

    def detener_todo(self):
        """Limpieza completa para el shutdown de la app: timer, nodo y launch."""
        if self.ros_timer is not None:
            self.ros_timer.stop()
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception as e:
                print(f"[Shutdown] Error destruyendo nodo de cámara: {e}")
        self.detener_launch()
