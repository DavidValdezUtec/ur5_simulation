from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Nota: Usamos detección por tópicos; si ambos haptics publican, habilitamos teleoperación.
# Si falta uno o ambos, deshabilitamos teleop y dejamos seguimiento de trayectoria automático.

def _detect_geomagic_topics(timeout_sec: float = 1.0):
    try:
        import rclpy
        from rclpy.node import Node as RclpyNode
        rclpy.init(args=None)
        tmp = RclpyNode('geomagic_detector')
        # spin breve para asegurar que el grafo esté poblado
        end_time = tmp.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while rclpy.ok() and tmp.get_clock().now() < end_time:
            rclpy.spin_once(tmp, timeout_sec=0.1)
        topics = dict(tmp.get_topic_names_and_types())
        tmp.destroy_node()
        rclpy.shutdown()
    except Exception:
        topics = {}

    pose1 = '/phantom/pose'
    button1 = '/phantom/button'
    pose2 = '/phantom2/phantom/pose'
    button2 = '/phantom2/phantom/button'

    has_1 = pose1 in topics and button1 in topics
    has_2 = pose2 in topics and button2 in topics
    return {
        'both_present': has_1 and has_2,
        'phantom1_pose': pose1,
        'phantom1_button': button1,
        'phantom2_pose': pose2,
        'phantom2_button': button2,
    }


def _setup_actions(context):
    # Argumentos recibidos desde la CLI
    r1_simulation = LaunchConfiguration('r1_simulation').perform(context)
    r2_simulation = LaunchConfiguration('r2_simulation').perform(context)
    robot1 = LaunchConfiguration('robot1').perform(context)
    robot2 = LaunchConfiguration('robot2').perform(context)

    r1_x_pos = LaunchConfiguration('r1_x_pos').perform(context)
    r1_y_pos = LaunchConfiguration('r1_y_pos').perform(context)
    r1_z_pos = LaunchConfiguration('r1_z_pos').perform(context)
    r1_rot_x = LaunchConfiguration('r1_rot_x').perform(context)
    r1_rot_y = LaunchConfiguration('r1_rot_y').perform(context)
    r1_rot_z = LaunchConfiguration('r1_rot_z').perform(context)
    r2_x_pos = LaunchConfiguration('r2_x_pos').perform(context)
    r2_y_pos = LaunchConfiguration('r2_y_pos').perform(context)
    r2_z_pos = LaunchConfiguration('r2_z_pos').perform(context)
    r2_rot_x = LaunchConfiguration('r2_rot_x').perform(context)
    r2_rot_y = LaunchConfiguration('r2_rot_y').perform(context)
    r2_rot_z = LaunchConfiguration('r2_rot_z').perform(context)

    # Detección de haptics por tópicos
    det = _detect_geomagic_topics(timeout_sec=1.0)
    geomagic_enabled = bool(det['both_present'])

    # Launch del driver dual de UR con namespaces r1/r2
    robot_launch_path = get_package_share_directory('ur_robot_driver') + '/launch/dual_control.launch.py'
    actions = []
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path),
            launch_arguments={
                'r1_type': robot1,
                'r2_type': robot2,
                'use_fake_hardware_r1': r1_simulation,
                'use_fake_hardware_r2': r2_simulation,
                'r1_x_pos': r1_x_pos,
                'r1_y_pos': r1_y_pos,
                'r1_z_pos': r1_z_pos,
                'r1_rot_x': r1_rot_x,
                'r1_rot_y': r1_rot_y,
                'r1_rot_z': r1_rot_z,
                'r2_x_pos': r2_x_pos,
                'r2_y_pos': r2_y_pos,
                'r2_z_pos': r2_z_pos,
                'r2_rot_x': r2_rot_x,
                'r2_rot_y': r2_rot_y,
                'r2_rot_z': r2_rot_z,
            }.items()
        )
    )

    # Dos controladores, uno por robot, con geomagic on/off según detección
    # r1
    actions.append(
        Node(
            package='ur5_controller',
            executable='controller_backup',
            name='ur5_controller_r1',
            output='screen',
            parameters=[{
                'nmspace': 'r1',
                'ur': robot1,
                'control_topic': '/joint_trajectory_controller/joint_trajectory',
                'geomagic': geomagic_enabled,
                'geomagic_topic': det['phantom1_pose'],
                'geomagic_button_topic': det['phantom1_button'],
            }],
        )
    )

    # r2
    actions.append(
        Node(
            package='ur5_controller',
            executable='controller_backup',
            name='ur5_controller_r2',
            output='screen',
            parameters=[{
                'nmspace': 'r2',
                'ur': robot2,
                'control_topic': '/joint_trajectory_controller/joint_trajectory',
                'geomagic': geomagic_enabled,
                'geomagic_topic': det['phantom2_pose'],
                'geomagic_button_topic': det['phantom2_button'],
            }],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Tipos de robot y modo fake hardware
        DeclareLaunchArgument('robot1', default_value='ur5', description='Robot 1 type'),
        DeclareLaunchArgument('robot2', default_value='ur5e', description='Robot 2 type'),
        DeclareLaunchArgument('r1_simulation', default_value='true', description='Use simulation for robot 1'),
        DeclareLaunchArgument('r2_simulation', default_value='true', description='Use simulation for robot 2'),
        # Pose de colocación opcional
        DeclareLaunchArgument('r1_x_pos', default_value='0.0'),
        DeclareLaunchArgument('r1_y_pos', default_value='0.9'),
        DeclareLaunchArgument('r1_z_pos', default_value='0.0'),
        DeclareLaunchArgument('r1_rot_x', default_value='0.0'),
        DeclareLaunchArgument('r1_rot_y', default_value='0.0'),
        DeclareLaunchArgument('r1_rot_z', default_value='0.0'),
        DeclareLaunchArgument('r2_x_pos', default_value='0.0'),
        DeclareLaunchArgument('r2_y_pos', default_value='-0.9'),
        DeclareLaunchArgument('r2_z_pos', default_value='0.0'),
        DeclareLaunchArgument('r2_rot_x', default_value='0.0'),
        DeclareLaunchArgument('r2_rot_y', default_value='0.0'),
        DeclareLaunchArgument('r2_rot_z', default_value='3.14'),
        # OpaqueFunction ejecuta la detección y devuelve las acciones
        OpaqueFunction(function=_setup_actions),
    ])
