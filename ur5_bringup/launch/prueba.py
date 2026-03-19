#!/usr/bin/env python3
import argparse
import math
import socket
import sys
import time
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from builtin_interfaces.msg import Time as BuiltinTime
from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_ros import Buffer, TransformListener

TransformException = getattr(tf2_ros, "TransformException", Exception)

# ✅ TF2 helper (aplica TransformStamped correctamente a PoseStamped)
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    RobotState,
    RobotTrajectory,
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


def _unwrap_to_nearest(target: float, reference: float) -> float:
    two_pi = 2.0 * math.pi
    if two_pi <= 0.0:
        return float(target)
    k = round((reference - target) / two_pi)
    return float(target + k * two_pi)


# ----------------------------
# PlanningScene helper publisher
# ----------------------------
class ScenePublisher(Node):
    def __init__(self):
        super().__init__("scene_publisher")
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

    def add_world_box(self, obj_id: str, frame_id: str, x, y, z, sx, sy, sz):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(sx), float(sy), float(sz)]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        co.primitives = [box]
        co.primitive_poses = [pose]
        return co

    def attach_box_to_robot(
        self,
        obj_id: str,
        link_name: str,
        frame_id: str,
        x,
        y,
        z,
        sx,
        sy,
        sz,
        touch_links=None,
    ):
        aco = AttachedCollisionObject()
        aco.link_name = str(link_name)
        aco.object.id = str(obj_id)
        aco.object.header.frame_id = str(frame_id)
        aco.object.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(sx), float(sy), float(sz)]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        aco.object.primitives = [box]
        aco.object.primitive_poses = [pose]

        if touch_links:
            aco.touch_links = list(touch_links)

        return aco

    def attach_cylinder_to_robot(
        self,
        obj_id: str,
        link_name: str,
        frame_id: str,
        x,
        y,
        z,
        height,
        radius,
        touch_links=None,
    ):
        aco = AttachedCollisionObject()
        aco.link_name = str(link_name)
        aco.object.id = str(obj_id)
        aco.object.header.frame_id = str(frame_id)
        aco.object.operation = CollisionObject.ADD

        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [float(height), float(radius)]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        aco.object.primitives = [cylinder]
        aco.object.primitive_poses = [pose]

        if touch_links:
            aco.touch_links = list(touch_links)

        return aco

    def publish_scene(self, world_objects, attached_objects):
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = list(world_objects) if world_objects else []
        ps.robot_state = RobotState()
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = list(attached_objects) if attached_objects else []

        # Publica varias veces para evitar que RViz/MoveIt pierdan el diff
        for _ in range(8):
            self.pub.publish(ps)


# ----------------------------
# Orientation helpers
# ----------------------------
def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_to_rotvec(x: float, y: float, z: float, w: float):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 0.0
    x /= n
    y /= n
    z /= n
    w /= n

    # Canonical shortest-angle: ensure w >= 0.
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    w = max(-1.0, min(1.0, w))
    theta = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-12 or theta < 1e-12:
        return 0.0, 0.0, 0.0, 0.0

    ax = x / s
    ay = y / s
    az = z / s
    return ax * theta, ay * theta, az * theta, theta


def rotvec_to_quaternion(rx: float, ry: float, rz: float):
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    ax = rx / theta
    ay = ry / theta
    az = rz / theta
    s = math.sin(theta / 2.0)
    return ax * s, ay * s, az * s, math.cos(theta / 2.0)


def quat_dot_abs(q1, q2) -> float:
    return abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])


def quat_angle_error_rad(q1, q2) -> float:
    d = quat_dot_abs(q1, q2)
    d = max(-1.0, min(1.0, d))
    return 2.0 * math.acos(d)


def rotvec_alternatives(rv):
    rx, ry, rz = rv
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    v = (rx, ry, rz)
    if theta < 1e-12:
        return [v]
    v_neg = (-rx, -ry, -rz)
    scale = -((2.0 * math.pi) - theta) / theta
    v_2pi = (rx * scale, ry * scale, rz * scale)
    return [v, v_neg, v_2pi]


def closest_rotvec(rv, reference_rv):
    cands = rotvec_alternatives(rv)
    best = None
    best_d = None
    for c in cands:
        dx = c[0] - reference_rv[0]
        dy = c[1] - reference_rv[1]
        dz = c[2] - reference_rv[2]
        d2 = dx * dx + dy * dy + dz * dz
        if best is None or d2 < best_d:
            best = c
            best_d = d2
    return best


# ----------------------------
# MoveIt Plan/Execute Node
# ----------------------------
class MoveItPosePlanExecute(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_pose")

        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("eef_link", "tool0")
        self.declare_parameter("base_frame", "base_link")

        # ✅ tolerancias realistas (MoveIt, no controller)
        self.declare_parameter("position_tolerance", 0.005)      # 5 mm
        self.declare_parameter("orientation_tolerance", 0.03)    # ~1.7 deg

        self.declare_parameter("allowed_planning_time", 30.0)
        self.declare_parameter("num_planning_attempts", 20)
        self.declare_parameter("max_velocity_scaling_factor", 0.07)
        self.declare_parameter("max_acceleration_scaling_factor", 0.07)
        self.declare_parameter("cartesian_max_velocity_scaling_factor", 0.007)
        self.declare_parameter("cartesian_max_acceleration_scaling_factor", 0.007)
        self.declare_parameter("diagnostics_enable", False)
        self.declare_parameter("interrupt_dashboard_stop_enable", True)
        self.declare_parameter("dashboard_ip", "192.168.10.103")
        self.declare_parameter("dashboard_port", 29999)
        self.declare_parameter("dashboard_socket_timeout_s", 2.0)

        group_name = self.get_parameter("group_name").get_parameter_value().string_value
        self.group_candidates = [group_name, "ur_manipulator", "manipulator"]
        self.group_name = None

        self.eef_link = self.get_parameter("eef_link").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        self.pos_tol = self.get_parameter("position_tolerance").get_parameter_value().double_value
        self.ori_tol = self.get_parameter("orientation_tolerance").get_parameter_value().double_value

        self.allowed_planning_time = self.get_parameter("allowed_planning_time").get_parameter_value().double_value
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").get_parameter_value().integer_value)
        self.vel_scale = self.get_parameter("max_velocity_scaling_factor").get_parameter_value().double_value
        self.acc_scale = self.get_parameter("max_acceleration_scaling_factor").get_parameter_value().double_value
        self.diagnostics_enable = bool(self.get_parameter("diagnostics_enable").get_parameter_value().bool_value)
        self._diag_monitor_active = False
        self._diag_real_max_abs_vel = 0.0
        self._diag_real_samples = 0

        self.js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self._active_exec_goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Runtime-configurable timeouts (set from CLI)
        self.service_timeout_s: float = 10.0
        self.action_timeout_s: float = 180.0

    def _spin_future(self, fut, *, timeout_s: float, label: str):
        timeout_s = float(max(0.0, timeout_s))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done():
            raise RuntimeError(f"Timeout esperando {label} tras {timeout_s:.1f}s")
        return fut

    def _js_cb(self, msg: JointState):
        self.js = msg
        if self._diag_monitor_active and getattr(msg, "velocity", None):
            try:
                max_abs = max(abs(float(v)) for v in msg.velocity)
                if max_abs > self._diag_real_max_abs_vel:
                    self._diag_real_max_abs_vel = max_abs
                self._diag_real_samples += 1
            except Exception:
                pass

    @staticmethod
    def _duration_to_sec(dur_msg) -> float:
        if dur_msg is None:
            return 0.0
        sec = float(getattr(dur_msg, "sec", 0.0) or 0.0)
        nsec = float(getattr(dur_msg, "nanosec", 0.0) or 0.0)
        return sec + (nsec * 1e-9)

    def _traj_diagnostics_summary(self, traj: RobotTrajectory) -> tuple[int, int, float, Optional[float]]:
        jt = getattr(traj, "joint_trajectory", None)
        if jt is None:
            return 0, 0, 0.0, None

        names = list(getattr(jt, "joint_names", []) or [])
        points = list(getattr(jt, "points", []) or [])
        point_count = len(points)
        joint_count = len(names)

        duration_s = 0.0
        if point_count > 0:
            duration_s = self._duration_to_sec(getattr(points[-1], "time_from_start", None))

        planned_max_abs_vel: Optional[float] = None
        for point in points:
            velocities = list(getattr(point, "velocities", []) or [])
            if not velocities:
                continue
            try:
                local_max = max(abs(float(v)) for v in velocities)
            except Exception:
                continue
            if planned_max_abs_vel is None or local_max > planned_max_abs_vel:
                planned_max_abs_vel = local_max

        return point_count, joint_count, duration_s, planned_max_abs_vel

    def _log_traj_diagnostics(self, traj: RobotTrajectory, *, tag: str):
        if not self.diagnostics_enable:
            return
        points, joints, duration_s, planned_max_abs_vel = self._traj_diagnostics_summary(traj)
        if planned_max_abs_vel is None:
            self.get_logger().info(
                f"[diag:{tag}] points={points}, joints={joints}, duration={duration_s:.3f}s, "
                f"planned_max_abs_joint_vel=n/a (sin velocidades en trayectoria), "
                f"scale(v/a)=({self.vel_scale:.4f}/{self.acc_scale:.4f})"
            )
        else:
            self.get_logger().info(
                f"[diag:{tag}] points={points}, joints={joints}, duration={duration_s:.3f}s, "
                f"planned_max_abs_joint_vel={planned_max_abs_vel:.4f} rad/s, "
                f"scale(v/a)=({self.vel_scale:.4f}/{self.acc_scale:.4f})"
            )

    def wait_ready(self):
        self.get_logger().info("Esperando /plan_kinematic_path, /execute_trajectory y /joint_states…")
        self.plan_cli.wait_for_service()
        self.ik_cli.wait_for_service()
        if not self.cart_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/compute_cartesian_path no disponible (cartesian deshabilitado).")
        self.exec_ac.wait_for_server()

        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Listo.")

    def current_robot_state(self) -> RobotState:
        if self.js is None:
            raise RuntimeError("No se recibió /joint_states")
        rs = RobotState()
        rs.is_diff = True
        rs.joint_state = JointState()
        rs.joint_state.name = list(self.js.name)
        rs.joint_state.position = list(self.js.position)
        if len(self.js.velocity) == len(self.js.name):
            rs.joint_state.velocity = list(self.js.velocity)
        if len(self.js.effort) == len(self.js.name):
            rs.joint_state.effort = list(self.js.effort)
        return rs

    def check_ik(self, goal: PoseStamped, group_name: str, ik_link_name: Optional[str] = None) -> Tuple[bool, Optional[RobotState]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = str(group_name)
        req.ik_request.robot_state = self.current_robot_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = goal
        req.ik_request.timeout = Duration(sec=1, nanosec=0)
        if ik_link_name:
            req.ik_request.ik_link_name = str(ik_link_name)

        fut = self.ik_cli.call_async(req)
        try:
            self._spin_future(fut, timeout_s=self.service_timeout_s, label="/compute_ik")
        except Exception as e:
            self.get_logger().warn(f"/compute_ik timeout/fallo para group='{group_name}': {e}")
            return False, None
        resp = fut.result()
        if resp is None:
            self.get_logger().warn("/compute_ik sin respuesta")
            return False, None

        code = resp.error_code.val
        if code == 1:
            self.get_logger().info(f"IK OK para group='{group_name}' link='{ik_link_name or '(default)'}'.")
            return True, resp.solution

        self.get_logger().warn(f"IK falló para group='{group_name}' link='{ik_link_name or '(default)'}' (error_code={code}).")
        return False, None

    def log_current_eef_pose(self, label: str = "EEF"):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        base_candidates = [self.base_frame, "base_link", "base", "world"]
        eef_candidates = [self.eef_link, "tool0", f"{self.eef_link}_controller", "tool0_controller"]

        last_err = None
        t = None
        used = None
        for bf in base_candidates:
            for ef in eef_candidates:
                try:
                    t = self.tf_buffer.lookup_transform(bf, ef, Time())
                    used = (bf, ef)
                    break
                except TransformException as e:
                    last_err = e
            if t is not None:
                break

        if t is None or used is None:
            self.get_logger().warn(
                f"TF no disponible. Probé bases={base_candidates} y eefs={eef_candidates}. Último error: {last_err}"
            )
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        q = t.transform.rotation
        rr, pp, yy = quaternion_to_rpy(q.x, q.y, q.z, q.w)
        rvx, rvy, rvz, theta = quaternion_to_rotvec(q.x, q.y, q.z, q.w)
        bf, ef = used
        self.get_logger().info(
            f"{label} pose en {bf} (TF {bf}<-{ef}): xyz=({tx:.4f},{ty:.4f},{tz:.4f}) rpy=({rr:.3f},{pp:.3f},{yy:.3f})"
        )
        self.get_logger().info(
            f"{label} orientación (UR rotvec) en {bf}: RxRyRz=({rvx:.3f},{rvy:.3f},{rvz:.3f}) |theta|={theta:.3f}"
        )

    def cartesian_path(
        self,
        goal: PoseStamped,
        group_name: str,
        max_step: float = 0.01,
        jump_threshold: float = 0.0,
        min_fraction: float = 0.95,
    ) -> RobotTrajectory:
        if not self.cart_cli.service_is_ready():
            raise RuntimeError("/compute_cartesian_path no está disponible")

        req = GetCartesianPath.Request()
        req.start_state = self.current_robot_state()
        req.group_name = str(group_name)
        req.link_name = str(self.eef_link)
        req.header = Header()
        req.header.frame_id = str(goal.header.frame_id)
        req.waypoints = [goal.pose]
        req.max_step = float(max_step)
        req.jump_threshold = float(jump_threshold)
        req.avoid_collisions = True
        try:
            setattr(req, "max_velocity_scaling_factor", float(self.vel_scale))
        except Exception:
            pass
        try:
            setattr(req, "max_acceleration_scaling_factor", float(self.acc_scale))
        except Exception:
            pass

        fut = self.cart_cli.call_async(req)
        self._spin_future(fut, timeout_s=self.service_timeout_s, label="/compute_cartesian_path")
        resp = fut.result()
        if resp is None:
            raise RuntimeError("GetCartesianPath sin respuesta")

        frac = float(resp.fraction)
        self.get_logger().info(f"CartesianPath fraction={frac:.3f} (min={float(min_fraction):.3f})")
        if frac < float(min_fraction):
            raise RuntimeError(f"CartesianPath incompleto: fraction={frac:.3f}")
        return resp.solution

    def _build_pose_constraints(self, goal: PoseStamped, constrain_orientation: bool = True) -> Constraints:
        c = Constraints()

        pc = PositionConstraint()
        pc.header = goal.header
        pc.link_name = self.eef_link
        pc.weight = 1.0

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        side = max(self.pos_tol * 2.0, 1e-6)
        box.dimensions = [side, side, side]
        bv.primitives = [box]
        bv.primitive_poses = [goal.pose]
        pc.constraint_region = bv
        c.position_constraints = [pc]

        if constrain_orientation:
            oc = OrientationConstraint()
            oc.header = goal.header
            oc.link_name = self.eef_link
            oc.orientation = goal.pose.orientation
            oc.absolute_x_axis_tolerance = self.ori_tol
            oc.absolute_y_axis_tolerance = self.ori_tol
            oc.absolute_z_axis_tolerance = self.ori_tol
            oc.weight = 1.0
            c.orientation_constraints = [oc]

        return c

    def _build_request(self, group_name: str, goal: PoseStamped, constrain_orientation: bool = True) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.max_velocity_scaling_factor = float(self.vel_scale)
        req.max_acceleration_scaling_factor = float(self.acc_scale)
        req.start_state = self.current_robot_state()
        req.goal_constraints = [self._build_pose_constraints(goal, constrain_orientation=constrain_orientation)]
        return req

    def _build_joint_goal_constraints(self, goal_state: RobotState) -> Constraints:
        if self.js is None:
            raise RuntimeError("No se recibió /joint_states")
        c = Constraints()

        goal_names = list(getattr(goal_state.joint_state, "name", []) or [])
        goal_positions = list(getattr(goal_state.joint_state, "position", []) or [])
        goal_map = {n: float(p) for n, p in zip(goal_names, goal_positions) if n}

        cur_map = {n: float(p) for n, p in zip(self.js.name, self.js.position) if n}

        preferred_ur = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        joint_names = [n for n in preferred_ur if n in goal_map and n in cur_map]
        if not joint_names:
            joint_names = [n for n in goal_map.keys() if n in cur_map]

        tol = 0.02
        joint_constraints: List[JointConstraint] = []
        for name in joint_names:
            target = _unwrap_to_nearest(goal_map[name], cur_map[name])
            jc = JointConstraint()
            jc.joint_name = str(name)
            jc.position = float(target)
            jc.tolerance_above = float(tol)
            jc.tolerance_below = float(tol)
            jc.weight = 1.0
            joint_constraints.append(jc)

        c.joint_constraints = joint_constraints
        return c

    def _build_request_joint_goal(self, group_name: str, goal_state: RobotState) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.max_velocity_scaling_factor = float(self.vel_scale)
        req.max_acceleration_scaling_factor = float(self.acc_scale)
        req.start_state = self.current_robot_state()
        req.goal_constraints = [self._build_joint_goal_constraints(goal_state)]
        return req

    def plan_joint_goal(self, group_name: str, goal_state: RobotState):
        self.get_logger().info(f"Intentando plan articular (IK) con group_name='{group_name}'…")
        srv = GetMotionPlan.Request()
        srv.motion_plan_request = self._build_request_joint_goal(group_name, goal_state)
        future = self.plan_cli.call_async(srv)
        self._spin_future(future, timeout_s=self.service_timeout_s, label="/plan_kinematic_path (joint goal)")
        resp = future.result()
        if resp is None:
            raise RuntimeError("GetMotionPlan sin respuesta")
        code = resp.motion_plan_response.error_code.val
        self.get_logger().info(f"[{group_name}] MoveIt error_code={code}")
        if code != 1:
            raise RuntimeError(f"Plan articular fallido (error_code={code})")
        self.group_name = group_name
        return resp.motion_plan_response.trajectory

    def plan(self, goal: PoseStamped, constrain_orientation: bool = True):
        for grp in self.group_candidates:
            if not grp:
                continue
            self.get_logger().info(f"Intentando plan con group_name='{grp}'…")
            srv = GetMotionPlan.Request()
            srv.motion_plan_request = self._build_request(grp, goal, constrain_orientation=constrain_orientation)
            future = self.plan_cli.call_async(srv)
            try:
                self._spin_future(future, timeout_s=self.service_timeout_s, label=f"/plan_kinematic_path (group={grp})")
            except Exception as e:
                self.get_logger().warn(f"[{grp}] timeout/fallo de planificación: {e}")
                continue
            resp = future.result()
            if resp is None:
                self.get_logger().warn("GetMotionPlan sin respuesta")
                continue
            code = resp.motion_plan_response.error_code.val
            self.get_logger().info(f"[{grp}] MoveIt error_code={code}")
            if code == 1:
                self.group_name = grp
                return resp.motion_plan_response.trajectory
            self.get_logger().warn(f"[{grp}] Plan fallido, probando siguiente grupo…")
        raise RuntimeError("Plan fallido en todos los grupos probados")

    def execute(self, traj):
        self._log_traj_diagnostics(traj, tag="pre-exec")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        send_fut = self.exec_ac.send_goal_async(goal)
        self._spin_future(send_fut, timeout_s=self.action_timeout_s, label="send_goal /execute_trajectory")
        gh = send_fut.result()
        if not gh or not gh.accepted:
            raise RuntimeError("ExecuteTrajectory rechazado")
        self._active_exec_goal_handle = gh
        diag_t0 = 0.0
        if self.diagnostics_enable:
            self._diag_real_max_abs_vel = 0.0
            self._diag_real_samples = 0
            self._diag_monitor_active = True
            diag_t0 = time.time()
        try:
            res_fut = gh.get_result_async()
            self._spin_future(res_fut, timeout_s=self.action_timeout_s, label="result /execute_trajectory")
            res = res_fut.result()
        finally:
            if self.diagnostics_enable:
                self._diag_monitor_active = False
                elapsed_s = max(0.0, time.time() - diag_t0)
                self.get_logger().info(
                    f"[diag:exec] observed_max_abs_joint_vel={self._diag_real_max_abs_vel:.4f} rad/s, "
                    f"samples={self._diag_real_samples}, window={elapsed_s:.3f}s"
                )
            self._active_exec_goal_handle = None
        if res is None:
            raise RuntimeError("/execute_trajectory devolvió resultado vacío")

        status = int(getattr(res, "status", -1))
        result_msg = getattr(res, "result", None)
        err = getattr(result_msg, "error_code", None)
        code = int(getattr(err, "val", -999)) if err is not None else -999

        self.get_logger().info(
            f"Resultado ejecución: action_status={status}, moveit_error_code={code}"
        )

        if code != 1:
            raise RuntimeError(
                f"ExecuteTrajectory falló (action_status={status}, moveit_error_code={code})"
            )
        return res

    def cancel_active_execution(self):
        gh = self._active_exec_goal_handle
        if gh is None:
            self.get_logger().warn("No hay goal activo de /execute_trajectory para cancelar.")
            return
        try:
            self.get_logger().warn("Cancelando goal activo de /execute_trajectory por interrupción…")
            cancel_fut = gh.cancel_goal_async()
            self._spin_future(cancel_fut, timeout_s=2.0, label="cancel /execute_trajectory")
            self.get_logger().warn("Solicitud de cancel enviada a /execute_trajectory.")
        except Exception as e:
            self.get_logger().error(f"No se pudo cancelar /execute_trajectory: {e}")

    def dashboard_stop_best_effort(self):
        if not bool(self.get_parameter("interrupt_dashboard_stop_enable").get_parameter_value().bool_value):
            return
        ip = str(self.get_parameter("dashboard_ip").get_parameter_value().string_value or "192.168.10.103")
        port = int(self.get_parameter("dashboard_port").get_parameter_value().integer_value or 29999)
        timeout_s = float(self.get_parameter("dashboard_socket_timeout_s").get_parameter_value().double_value or 2.0)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(timeout_s)
            sock.connect((ip, port))
            try:
                sock.recv(1024)
            except Exception:
                pass
            sock.sendall(b"stop\n")
            out = ""
            try:
                out = sock.recv(4096).decode("utf-8", errors="replace").strip()
            except Exception:
                pass
            if out:
                self.get_logger().warn(f"UR dash stop -> {out}")
            else:
                self.get_logger().warn("UR dash stop enviado (sin respuesta).")
        except Exception as e:
            self.get_logger().warn(f"No se pudo enviar UR dash stop: {e}")
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def validate_trajectory_safety(
        self,
        traj: RobotTrajectory,
        *,
        max_single_joint_displacement_rad: float = math.pi,
        max_waypoint_jump_rad: float = 0.5,
    ) -> None:
        """Pre-execution check: reject trajectories with dangerous joint movements.

        Catches configuration flips and wild motions BEFORE the robot moves.
        """
        jt = getattr(traj, "joint_trajectory", None)
        if jt is None:
            return
        points = list(getattr(jt, "points", []) or [])
        names = list(getattr(jt, "joint_names", []) or [])
        if len(points) < 2 or not names:
            return

        n_joints = len(names)

        # 1) Check total displacement per joint (first point -> last point)
        start_pos = list(getattr(points[0], "positions", []) or [])
        end_pos = list(getattr(points[-1], "positions", []) or [])
        if len(start_pos) >= n_joints and len(end_pos) >= n_joints:
            for j in range(n_joints):
                total_disp = abs(float(end_pos[j]) - float(start_pos[j]))
                if total_disp > max_single_joint_displacement_rad:
                    raise RuntimeError(
                        f"Trayectoria rechazada (seguridad): joint '{names[j]}' desplazamiento total "
                        f"{total_disp:.3f} rad ({math.degrees(total_disp):.1f} deg) > límite "
                        f"{max_single_joint_displacement_rad:.3f} rad ({math.degrees(max_single_joint_displacement_rad):.1f} deg)"
                    )

        # 2) Check consecutive waypoint jumps (detect mid-trajectory flips)
        prev_pos = start_pos
        for i in range(1, len(points)):
            cur_pos = list(getattr(points[i], "positions", []) or [])
            if len(cur_pos) < n_joints or len(prev_pos) < n_joints:
                prev_pos = cur_pos
                continue
            for j in range(n_joints):
                jump = abs(float(cur_pos[j]) - float(prev_pos[j]))
                if jump > max_waypoint_jump_rad:
                    raise RuntimeError(
                        f"Trayectoria rechazada (seguridad): salto entre waypoints {i - 1}->{i} "
                        f"en joint '{names[j]}': {jump:.3f} rad ({math.degrees(jump):.1f} deg) > límite "
                        f"{max_waypoint_jump_rad:.3f} rad ({math.degrees(max_waypoint_jump_rad):.1f} deg)"
                    )
            prev_pos = cur_pos

        self.get_logger().info(
            f"Validación de trayectoria OK ({len(points)} puntos, {n_joints} joints, "
            f"max_disp={max_single_joint_displacement_rad:.2f} rad, max_jump={max_waypoint_jump_rad:.2f} rad)"
        )


# ----------------------------
# Main
# ----------------------------
def main(argv=None):
    rclpy.init(args=argv)

    ap = argparse.ArgumentParser(
        description="Publica PlanningScene y ejecuta un plan a una pose. Soporta RPY o UR rotvec (Rx,Ry,Rz)."
    )
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--z", type=float)

    ap.add_argument("--roll", type=float)
    ap.add_argument("--pitch", type=float)
    ap.add_argument("--yaw", type=float)
    ap.add_argument("--degrees", action="store_true", help="Interpretar roll/pitch/yaw en grados")

    ap.add_argument("--mm", action="store_true", help="Interpretar x/y/z de entrada en milímetros (teach pendant).")

    ap.add_argument("--ur-rotvec", action="store_true", help="Usar orientación tipo UR rotvec (Rx,Ry,Rz).")
    ap.add_argument("--rx", type=float, help="UR rotvec Rx (rad)")
    ap.add_argument("--ry", type=float, help="UR rotvec Ry (rad)")
    ap.add_argument("--rz", type=float, help="UR rotvec Rz (rad)")

    ap.add_argument("--position-only", action="store_true", help="Planificar solo posición (sin constraint de orientación).")
    ap.add_argument("--keep-current-orientation", action="store_true", help="Reemplaza la orientación objetivo por la orientación actual del efector (según TF).")
    ap.add_argument(
        "--cartesian",
        action="store_true",
        help="Planifica un movimiento cartesiano (más rectilíneo) usando /compute_cartesian_path.",
    )
    ap.add_argument("--eef-step", type=float, default=0.005, help="Paso cartesiano (m). Default: 0.005")
    ap.add_argument("--jump-threshold", type=float, default=1.5, help="Umbral de salto articular (factor). 0.0 deshabilita. Default: 1.5")
    ap.add_argument(
        "--cartesian-min-fraction",
        type=float,
        default=1.0,
        help="Fracción mínima aceptable del path cartesiano (0-1). Default: 1.0",
    )
    ap.add_argument(
        "--joint-goal",
        action="store_true",
        help="Si hay IK, planifica hacia el objetivo articular (reduce giros/flips innecesarios).",
    )
    ap.add_argument("--no-scene", action="store_true", help="No publicar PlanningScene (sin obstáculos/adjuntos).")
    ap.add_argument("--input-frame", default="base", help="Frame de entrada (por defecto: base)")

    ap.add_argument(
        "--service-timeout-s",
        type=float,
        default=10.0,
        help="Timeout (s) para servicios MoveIt (/compute_ik, /compute_cartesian_path, /plan_kinematic_path, etc).",
    )
    ap.add_argument(
        "--action-timeout-s",
        type=float,
        default=180.0,
        help="Timeout (s) para la acción /execute_trajectory (envío+resultado).",
    )
    ap.add_argument(
        "--max-orientation-retries",
        type=int,
        default=10,
        help="Reintentos máximos adicionales si el error angular final supera orientation_tolerance. Default: 10",
    )
    ap.add_argument(
        "--no-enforce-orientation",
        action="store_true",
        help="No fallar ni reintentar si la orientación final queda fuera de tolerancia.",
    )
    ap.add_argument(
        "--diagnostics",
        action="store_true",
        help="Imprime diagnósticos de trayectoria (duración, vel planificada y vel real observada).",
    )
    ap.add_argument(
        "--max-joint-displacement-rad",
        type=float,
        default=math.pi,
        help="Máximo desplazamiento total permitido por joint (rad). Rechaza trayectorias que excedan. Default: π",
    )
    ap.add_argument(
        "--max-waypoint-jump-rad",
        type=float,
        default=0.5,
        help="Máximo salto permitido entre waypoints consecutivos por joint (rad). Default: 0.5",
    )

    args, _ = ap.parse_known_args()

    def need(v):
        return v is None

    if args.ur_rotvec:
        if any(need(v) for v in (args.x, args.y, args.z, args.rx, args.ry, args.rz)):
            units = "milímetros" if args.mm else "metros"
            print(f"\nIngrese la pose objetivo (x/y/z en {units} + Rx/Ry/Rz en radianes):")
            args.x = float(input("x: ")) if need(args.x) else args.x
            args.y = float(input("y: ")) if need(args.y) else args.y
            args.z = float(input("z: ")) if need(args.z) else args.z
            args.rx = float(input("Rx: ")) if need(args.rx) else args.rx
            args.ry = float(input("Ry: ")) if need(args.ry) else args.ry
            args.rz = float(input("Rz: ")) if need(args.rz) else args.rz
    else:
        if any(need(v) for v in (args.x, args.y, args.z, args.roll, args.pitch, args.yaw)):
            units = "milímetros" if args.mm else "metros"
            print(f"\nIngrese la pose objetivo (x/y/z en {units} + roll/pitch/yaw en radianes):")
            args.x = float(input("x: ")) if need(args.x) else args.x
            args.y = float(input("y: ")) if need(args.y) else args.y
            args.z = float(input("z: ")) if need(args.z) else args.z
            args.roll = float(input("roll: ")) if need(args.roll) else args.roll
            args.pitch = float(input("pitch: ")) if need(args.pitch) else args.pitch
            args.yaw = float(input("yaw: ")) if need(args.yaw) else args.yaw

    raw_xyz = (float(args.x), float(args.y), float(args.z))

    # mm -> m
    if args.mm:
        args.x = float(args.x) * 0.001
        args.y = float(args.y) * 0.001
        args.z = float(args.z) * 0.001

    # orientation -> quaternion
    if args.ur_rotvec:
        qx, qy, qz, qw = rotvec_to_quaternion(float(args.rx), float(args.ry), float(args.rz))
    else:
        roll = float(args.roll)
        pitch = float(args.pitch)
        yaw = float(args.yaw)
        if args.degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

    moveit_node = MoveItPosePlanExecute()
    if args.diagnostics:
        moveit_node.diagnostics_enable = True
    if args.cartesian:
        cart_vel = float(
            moveit_node.get_parameter("cartesian_max_velocity_scaling_factor").get_parameter_value().double_value
        )
        cart_acc = float(
            moveit_node.get_parameter("cartesian_max_acceleration_scaling_factor").get_parameter_value().double_value
        )
        moveit_node.vel_scale = cart_vel
        moveit_node.acc_scale = cart_acc
    moveit_node.service_timeout_s = float(max(0.0, args.service_timeout_s))
    moveit_node.action_timeout_s = float(max(0.0, args.action_timeout_s))
    moveit_node.get_logger().info(
        f"Configuración: group_candidates={moveit_node.group_candidates}, base_frame='{moveit_node.base_frame}', eef_link='{moveit_node.eef_link}'"
    )
    moveit_node.get_logger().info(
        f"Timeouts: service_timeout_s={moveit_node.service_timeout_s:.1f}, action_timeout_s={moveit_node.action_timeout_s:.1f}"
    )
    if args.mm:
        moveit_node.get_logger().info(
            f"Entrada en mm: xyz=({raw_xyz[0]:.2f},{raw_xyz[1]:.2f},{raw_xyz[2]:.2f}) -> en m: xyz=({float(args.x):.4f},{float(args.y):.4f},{float(args.z):.4f})"
        )

    # ----------------------------
    # Publish planning scene (optional)
    # ----------------------------
    if not args.no_scene:
        scene_node = ScenePublisher()

        frame_world = moveit_node.base_frame
        table_size_x = 5.0
        table_size_y = 5.0
        table_thickness = 0.05
        table_top_z = 0.0
        table_center_z = table_top_z - (table_thickness * 0.5)

        ceiling_size_x = 5.0
        ceiling_size_y = 5.0
        ceiling_thickness = 0.05
        ceiling_bottom_z = 1.30
        ceiling_center_z = ceiling_bottom_z + (ceiling_thickness * 0.5)

        wall_thickness = 0.05
        wall_height = ceiling_bottom_z - table_top_z
        wall_center_z = table_top_z + (wall_height * 0.5)
        wall_span = 5.0

        box1 = scene_node.add_world_box(
            "table",
            frame_world,
            0.0,
            0.0,
            table_center_z,
            table_size_x,
            table_size_y,
            table_thickness,
        )

        box2 = scene_node.add_world_box(
            "ceiling",
            frame_world,
            0.0,
            0.0,
            ceiling_center_z,
            ceiling_size_x,
            ceiling_size_y,
            ceiling_thickness,
        )

        wall_back = scene_node.add_world_box(
            "wall_back",
            frame_world,
            -0.60,
            0.0,
            wall_center_z,
            wall_thickness,
            wall_span,
            wall_height,
        )

        wall_left = scene_node.add_world_box(
            "wall_left",
            frame_world,
            0.0,
            1.0,
            wall_center_z,
            wall_span,
            wall_thickness,
            wall_height,
        )

        wall_right = scene_node.add_world_box(
            "wall_right",
            frame_world,
            0.0,
            -1.0,
            wall_center_z,
            wall_span,
            wall_thickness,
            wall_height,
        )

        tool_link = moveit_node.eef_link

        touch_links = [
            tool_link,
            "wrist_3_link",
            "flange",
        ]

        gripper_box = scene_node.attach_cylinder_to_robot(
            "tool_gripper",
            tool_link, tool_link,
            0.00, 0.00, 0.0225,
            0.045,
            0.0575,
            touch_links,
        )
        camera_box = scene_node.attach_box_to_robot(
            "tool_camera",
            tool_link, tool_link,
            0.00, -0.0635, 0.01,
            0.150, 0.051, 0.11,
            touch_links,
        )
        finger_box = scene_node.attach_box_to_robot(
            "tool_finger",
            tool_link, tool_link,
            0.00, 0.00, 0.1025,
            0.148, 0.034, 0.115,
            touch_links,
        )

        scene_node.publish_scene([box1, box2, wall_back, wall_left, wall_right], [gripper_box, camera_box, finger_box])
        scene_node.get_logger().info(
            f"PlanningScene publicada: mesa 5x5m (top_z={table_top_z:.3f}) + techo 5x5m (bottom_z={ceiling_bottom_z:.3f}) + 3 paredes en '{frame_world}' + adjuntos (gripper/cámara/dedo)."
        )
        scene_node.destroy_node()

    # ----------------------------
    # Build goal pose
    # ----------------------------
    goal_in = PoseStamped()
    goal_in.header.frame_id = str(args.input_frame)
    goal_in.header.stamp = BuiltinTime()
    goal_in.pose.position.x = float(args.x)
    goal_in.pose.position.y = float(args.y)
    goal_in.pose.position.z = float(args.z)
    goal_in.pose.orientation.x = float(qx)
    goal_in.pose.orientation.y = float(qy)
    goal_in.pose.orientation.z = float(qz)
    goal_in.pose.orientation.w = float(qw)

    moveit_node.wait_ready()
    moveit_node.log_current_eef_pose("EEF actual (antes)")

    goal = goal_in
    moveit_node.get_logger().info(
        f"Goal (input) en '{goal_in.header.frame_id}': xyz=({goal_in.pose.position.x:.4f},{goal_in.pose.position.y:.4f},{goal_in.pose.position.z:.4f})"
    )

    if goal_in.header.frame_id != moveit_node.base_frame:
        try:
            tf = moveit_node.tf_buffer.lookup_transform(
                moveit_node.base_frame,
                goal_in.header.frame_id,
                Time(),
            )
            goal = do_transform_pose_stamped(goal_in, tf)
            goal.header.frame_id = moveit_node.base_frame
            moveit_node.get_logger().info(
                f"Goal (transformado) en '{goal.header.frame_id}': xyz=({goal.pose.position.x:.4f},{goal.pose.position.y:.4f},{goal.pose.position.z:.4f})"
            )
        except TransformException as e:
            moveit_node.get_logger().warn(
                f"No pude transformar goal desde '{goal_in.header.frame_id}' a '{moveit_node.base_frame}': {e}. Usando input tal cual."
            )
            goal = goal_in

    # keep-current-orientation (usa TF si está disponible)
    if args.keep_current_orientation:
        try:
            t = moveit_node.tf_buffer.lookup_transform(
                moveit_node.base_frame,
                moveit_node.eef_link,
                Time(),
            )
            q = t.transform.rotation
            goal.pose.orientation.x = float(q.x)
            goal.pose.orientation.y = float(q.y)
            goal.pose.orientation.z = float(q.z)
            goal.pose.orientation.w = float(q.w)
            moveit_node.get_logger().info(
                f"Orientación objetivo reemplazada por orientación actual (TF {moveit_node.base_frame}<-{moveit_node.eef_link})."
            )
        except Exception as e:
            moveit_node.get_logger().warn(f"No pude obtener orientación actual para override: {e}")

    # ✅ IMPORTANTE: congela la orientación objetivo REAL (copiando valores), para que el fallback NO la pise.
    q_goal_fixed = (
        float(goal.pose.orientation.x),
        float(goal.pose.orientation.y),
        float(goal.pose.orientation.z),
        float(goal.pose.orientation.w),
    )
    goal_rvx, goal_rvy, goal_rvz, goal_theta = quaternion_to_rotvec(*q_goal_fixed)

    moveit_node.get_logger().info(
        f"Goal orientación (UR rotvec) en {goal.header.frame_id}: RxRyRz=({goal_rvx:.3f},{goal_rvy:.3f},{goal_rvz:.3f}) |theta|={goal_theta:.3f}"
    )

    had_error = False
    try:
        constrain_orientation = not bool(args.position_only)
        enforce_orientation = bool(constrain_orientation and not args.no_enforce_orientation)
        max_orientation_retries = max(0, int(args.max_orientation_retries))

        if args.position_only:
            moveit_node.get_logger().info("Modo position-only: sin constraint de orientación.")
        if enforce_orientation:
            moveit_node.get_logger().info(
                f"Chequeo de orientación activado: tol={moveit_node.ori_tol:.4f} rad ({math.degrees(moveit_node.ori_tol):.2f} deg), reintentos={max_orientation_retries}."
            )

        def compute_orientation_error_and_log() -> Optional[float]:
            try:
                t = moveit_node.tf_buffer.lookup_transform(moveit_node.base_frame, moveit_node.eef_link, Time())
                q = t.transform.rotation
                q_act = (float(q.x), float(q.y), float(q.z), float(q.w))

                ang_err_local = quat_angle_error_rad(q_act, q_goal_fixed)
                rvx, rvy, rvz, theta = quaternion_to_rotvec(q_act[0], q_act[1], q_act[2], q_act[3])
                rv_aligned = closest_rotvec((rvx, rvy, rvz), (goal_rvx, goal_rvy, goal_rvz))
                moveit_node.get_logger().info(
                    f"Orientación alcanzada (UR rotvec) en {moveit_node.base_frame}: RxRyRz=({rvx:.3f},{rvy:.3f},{rvz:.3f}) |theta|={theta:.3f}"
                )
                if rv_aligned is not None:
                    moveit_node.get_logger().info(
                        f"Orientación alcanzada (alineada a goal): RxRyRz=({rv_aligned[0]:.3f},{rv_aligned[1]:.3f},{rv_aligned[2]:.3f})"
                    )
                moveit_node.get_logger().info(
                    f"Error angular orientación vs goal: {ang_err_local:.4f} rad ({math.degrees(ang_err_local):.2f} deg)"
                )
                return float(ang_err_local)
            except Exception as e:
                moveit_node.get_logger().warn(f"No pude comparar orientación alcanzada vs goal: {e}")
                return None

        def build_trajectory_once(attempt_idx: int) -> RobotTrajectory:
            ik_ok_local = False
            ik_group_local = None
            ik_solution_local = None
            for grp in ["ur_manipulator", "manipulator"]:
                ok, sol = moveit_node.check_ik(goal, grp, ik_link_name=moveit_node.eef_link)
                if ok and sol is not None:
                    ik_ok_local = True
                    ik_group_local = grp
                    ik_solution_local = sol
                    break

            if not ik_ok_local:
                moveit_node.get_logger().error(
                    "IK no encontró solución para este goal (o colisiona). "
                    "Si quieres mantener orientación+adjuntos, revisa touch_links y dimensiones/offset del gripper/cámara."
                )

            if args.cartesian:
                grp = ik_group_local or "ur_manipulator"
                moveit_node.get_logger().info(
                    f"Planificación cartesiana activada (grupo='{grp}', step={args.eef_step}, jump={args.jump_threshold})."
                )
                try:
                    return moveit_node.cartesian_path(
                        goal,
                        group_name=grp,
                        max_step=float(args.eef_step),
                        jump_threshold=float(args.jump_threshold),
                        min_fraction=float(args.cartesian_min_fraction),
                    )
                except Exception as e:
                    raise RuntimeError(
                        f"Cartesian falló con la orientación objetivo: {e}. "
                        "Se aborta para preservar orientación."
                    )

            use_joint_goal_now = bool(args.joint_goal and attempt_idx == 1)
            if use_joint_goal_now and ik_ok_local and ik_solution_local is not None and ik_group_local is not None:
                moveit_node.get_logger().info(
                    f"Usando goal articular desde IK (grupo='{ik_group_local}') para reducir giros innecesarios (solo intento 1)."
                )
                try:
                    return moveit_node.plan_joint_goal(ik_group_local, ik_solution_local)
                except Exception as e:
                    moveit_node.get_logger().warn(
                        f"Plan articular falló en intento {attempt_idx}: {e}. Fallback a plan por pose/orientación."
                    )

            return moveit_node.plan(goal, constrain_orientation=constrain_orientation)

        attempt = 0
        while True:
            attempt += 1
            try:
                traj = build_trajectory_once(attempt)
            except Exception as e:
                retries_used = attempt - 1
                retries_left = max_orientation_retries - retries_used
                if enforce_orientation and retries_left > 0:
                    moveit_node.get_logger().warn(
                        f"Planificación falló en intento {attempt}: {e}. Reintentando… (restantes={retries_left})"
                    )
                    continue
                raise

            try:
                jt = traj.joint_trajectory
                if jt.joint_names and jt.points:
                    moveit_node.get_logger().info(
                        f"Trayectoria intento {attempt}: {len(jt.points)} puntos, joints={len(jt.joint_names)}."
                    )
            except Exception:
                pass

            moveit_node.get_logger().info(f"Plan OK con group='{moveit_node.group_name}', validando seguridad…")
            try:
                moveit_node.validate_trajectory_safety(
                    traj,
                    max_single_joint_displacement_rad=float(args.max_joint_displacement_rad),
                    max_waypoint_jump_rad=float(args.max_waypoint_jump_rad),
                )
            except RuntimeError as safety_err:
                retries_used = attempt - 1
                retries_left = max_orientation_retries - retries_used
                if retries_left > 0:
                    moveit_node.get_logger().warn(
                        f"Trayectoria insegura en intento {attempt}: {safety_err}. "
                        f"Re-planificando… (restantes={retries_left})"
                    )
                    continue
                raise
            moveit_node.get_logger().info(f"Ejecutando intento {attempt}…")
            try:
                moveit_node.execute(traj)
            except Exception as e:
                retries_used = attempt - 1
                retries_left = max_orientation_retries - retries_used
                if enforce_orientation and retries_left > 0:
                    moveit_node.get_logger().warn(
                        f"Ejecución falló en intento {attempt}: {e}. Reintentando… (restantes={retries_left})"
                    )
                    continue
                raise
            moveit_node.log_current_eef_pose(f"EEF actual (después intento {attempt})")

            ang_err = compute_orientation_error_and_log()
            if not enforce_orientation:
                break

            if ang_err is None:
                raise RuntimeError("No pude verificar la orientación final para aplicar reintentos.")

            if ang_err <= float(moveit_node.ori_tol):
                moveit_node.get_logger().info("Orientación final dentro de tolerancia.")
                break

            retries_used = attempt - 1
            if retries_used >= max_orientation_retries:
                raise RuntimeError(
                    f"Orientación fuera de tolerancia tras {attempt} intento(s): "
                    f"error={ang_err:.4f} rad > tol={moveit_node.ori_tol:.4f} rad"
                )

            retries_left = max_orientation_retries - retries_used
            moveit_node.get_logger().warn(
                f"Orientación fuera de tolerancia (error={ang_err:.4f} rad > tol={moveit_node.ori_tol:.4f} rad). "
                f"Reintentando planificación/ejecución… (restantes={retries_left})"
            )

        moveit_node.get_logger().info("¡Hecho!")
    except KeyboardInterrupt:
        had_error = True
        moveit_node.get_logger().warn("Interrumpido por usuario (Ctrl+C). Intentando cancelar ejecución activa…")
        moveit_node.cancel_active_execution()
        moveit_node.dashboard_stop_best_effort()
    except Exception as e:
        had_error = True
        moveit_node.get_logger().error(f"Fallo plan/ejecución: {e}")
    finally:
        moveit_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(1 if had_error else 0)


if __name__ == "__main__":
    main()