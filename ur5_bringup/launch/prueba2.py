#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Librerías estándar de geometría necesarias para rellenar los mensajes
from geometry_msgs.msg import Point, Quaternion, Pose
from shape_msgs.msg import SolidPrimitive

# Las librerías que tienes disponibles
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory

class MoveItTerminalCommander(Node):
    def __init__(self):
        super().__init__('moveit_terminal_commander')
        
        # 1. Cliente del Servicio de Planificación
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 2. Cliente de Acción para la Ejecución
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Esperar a que los servicios estén disponibles
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando al servicio /plan_kinematic_path...')
        self.execute_client.wait_for_server()

    def send_goal(self, x, y, z):
        # ==========================================
        # FASE 1: CONSTRUIR LA PETICIÓN DE PLANIFICACIÓN
        # ==========================================
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'ur_manipulator' # Asegúrate de que este es tu grupo
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.max_velocity_scaling_factor = 0.1
        req.motion_plan_request.max_acceleration_scaling_factor = 0.1

        # Crear el contenedor de restricciones
        goal_constraint = Constraints()

        # --- Restricción de Posición ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tool0' # Efector final del UR5
        pos_constraint.weight = 1.0

        # Definir una pequeña esfera de tolerancia (radio 0.01m)
        tolerance_sphere = SolidPrimitive()
        tolerance_sphere.type = SolidPrimitive.SPHERE
        tolerance_sphere.dimensions = [0.01] 
        
        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(tolerance_sphere)
        bounding_volume.primitive_poses.append(target_pose)
        pos_constraint.constraint_region = bounding_volume

        # --- Restricción de Orientación ---
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'tool0'
        # Apuntando hacia abajo/adelante por defecto (depende de tu URDF)
        ori_constraint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0

        # Añadir restricciones a la petición
        goal_constraint.position_constraints.append(pos_constraint)
        goal_constraint.orientation_constraints.append(ori_constraint)
        req.motion_plan_request.goal_constraints.append(goal_constraint)

        # ==========================================
        # FASE 2: LLAMAR AL PLANIFICADOR
        # ==========================================
        self.get_logger().info(f'Planificando trayectoria hacia x:{x}, y:{y}, z:{z}...')
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.motion_plan_response.error_code.val != 1: # 1 es SUCCESS en MoveIt
            self.get_logger().error(f'Fallo en la planificación. Código de error: {response.motion_plan_response.error_code.val}')
            return

        self.get_logger().info('Planificación exitosa. Enviando a ejecución...')
        trajectory = response.motion_plan_response.trajectory

        # ==========================================
        # FASE 3: EJECUTAR LA TRAYECTORIA
        # ==========================================
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El controlador rechazó la trayectoria.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('¡Movimiento completado en Gazebo y RViz!')

def main(args=None):
    if len(sys.argv) < 4:
        print("Uso: python3 enviar_pose_ur5.py <x> <y> <z>")
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])

    rclpy.init(args=args)
    commander = MoveItTerminalCommander()
    commander.send_goal(x, y, z)
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()