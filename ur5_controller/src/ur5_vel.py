#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from functions import *  # fkine_ur5, jacobian_ur5

class UR5RealPositionZControl(Node):
    def __init__(self):
        super().__init__("ur5_z_velocity_control")

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.q_actual = np.zeros(6)
        self.ready = False
        self.initialized = False
        self.p_deseado = None
        self.K = np.diag([1.5, 1.5, 1.5])
        self.last_wait_log = 0.0

        # Cambiado a controlador de VELOCIDAD para el robot real
        self.pub = self.create_publisher(Float64MultiArray, '/joint_group_vel_controller/command', 1)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.timer = self.create_timer(0.2, self.control_step)
        rclpy.on_shutdown(self.stop_robot)

    def joint_callback(self, msg):
        pos_dict = dict(zip(msg.name, msg.position))
        try:
            self.q_actual = np.array([pos_dict[name] for name in self.joint_names])
            self.ready = True
        except KeyError:
            pass

    def control_step(self):
        if not self.ready:
            now = time.time()
            if now - self.last_wait_log > 0.5:
                self.get_logger().info("Esperando a /joint_states...")
                self.last_wait_log = now
            return

        if not self.initialized:
            T_init = fkine_ur5(self.q_actual)
            self.p_deseado = T_init[0:3, 3] + np.array([0.0, 0.0, 0.03])
            self.get_logger().info(
                f"Iniciando ascenso de 3cm. Objetivo Z: {self.p_deseado[2]:.4f}"
            )
            self.initialized = True

        T_act = fkine_ur5(self.q_actual)
        p_actual = T_act[0:3, 3]

        # Error de posición (3x1)
        error_p = p_actual - self.p_deseado

        # Jacobiano de posición (3x6)
        J_completo = jacobian_ur5(self.q_actual)
        J_v = J_completo[0:3, :]

        v = -self.K @ error_p

        dq = np.linalg.pinv(J_v) @ v

        # Limitador
        dq = np.clip(dq, -0.3, 0.3)

        # Publicar velocidad
        msg = Float64MultiArray()
        msg.data = dq.tolist()
        self.pub.publish(msg)

        if np.linalg.norm(error_p) < 0.001:
            self.get_logger().info("¡Objetivo alcanzado!")
            self.stop_robot()
            rclpy.shutdown()

    def stop_robot(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.pub.publish(msg)
        self.get_logger().info("Robot detenido.")

if __name__ == "__main__":
    try:
        rclpy.init()
        control = UR5RealPositionZControl()
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            control.stop_robot()
            control.destroy_node()
            rclpy.shutdown()