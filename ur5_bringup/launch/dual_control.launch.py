# multi_ur_control.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos que podríamos querer cambiar desde la línea de comandos
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_fake_hardware_r1 = LaunchConfiguration("use_fake_hardware_r1")
    use_fake_hardware_r2 = LaunchConfiguration("use_fake_hardware_r2")
    r1_type = LaunchConfiguration("r1_type")
    r2_type = LaunchConfiguration("r2_type")
    r1_x_pos = LaunchConfiguration("r1_x_pos")
    r1_y_pos = LaunchConfiguration("r1_y_pos")
    r1_z_pos = LaunchConfiguration("r1_z_pos")
    r1_rot_x = LaunchConfiguration("r1_rot_x")
    r1_rot_y = LaunchConfiguration("r1_rot_y")
    r1_rot_z = LaunchConfiguration("r1_rot_z")
    r2_x_pos = LaunchConfiguration("r2_x_pos")
    r2_y_pos = LaunchConfiguration("r2_y_pos")
    r2_z_pos = LaunchConfiguration("r2_z_pos")
    r2_rot_x = LaunchConfiguration("r2_rot_x")
    r2_rot_y = LaunchConfiguration("r2_rot_y")
    r2_rot_z = LaunchConfiguration("r2_rot_z")
    r1_IP = LaunchConfiguration("r1_IP")
    r2_IP = LaunchConfiguration("r2_IP")
    r1_TCP_port = LaunchConfiguration("r1_TCP_port")
    r2_TCP_port = LaunchConfiguration("r2_TCP_port")
    launch_feedback = LaunchConfiguration("launch_feedback")
    r1_force_filter_alpha = LaunchConfiguration("r1_force_filter_alpha")
    r2_force_filter_alpha = LaunchConfiguration("r2_force_filter_alpha")
    # --- Configuración del UR5 ---
    r1_launch = GroupAction(
        actions=[
            # Empuja todos los nodos de este grupo al namespace 'r1'
            PushRosNamespace("r1"),

            # Incluye el launch file de control original del driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur5_bringup"), "launch", "ur_control.launch.py"
                    ])
                ),
                launch_arguments={
                    "ur_type": r1_type,
                    "robot_ip": r1_IP,
                    "description_package": "ur5_description",
                    # Evitar f-strings con LaunchConfiguration: usar listas de sustituciones
                    "tf_prefix": ["r1", "_"],
                    "runtime_config_package": "ur_robot_driver",
                    "controllers_file": PathJoinSubstitution([
                        FindPackageShare("ur5_bringup"), "config", "ur_controllers_r1.yaml"
                    ]),
                    "kinematics_params_file": PathJoinSubstitution([
                        FindPackageShare("ur5_description"),
                        "config",
                        ["my_robot_calibration_", r1_type, ".yaml"],
                    ]),
                    "use_fake_hardware": use_fake_hardware_r1,
                    "launch_dashboard_client": launch_dashboard_client,
                    "launch_rviz": "false", # Lanzaremos un RViz común si es necesario
                    # Asignación explícita de puertos para el hardware real
                    "reverse_port": PythonExpression(["str(int('", r1_TCP_port, "') - 1)"]),
                    "script_sender_port": r1_TCP_port,
                    "trajectory_port": PythonExpression(["str(int('", r1_TCP_port, "') + 1)"]),
                    "script_command_port": PythonExpression(["str(int('", r1_TCP_port, "') + 2)"]),
                    "pos_x": r1_x_pos,
                    "pos_y": r1_y_pos,
                    "pos_z": r1_z_pos,
                    "rot_x": r1_rot_x,
                    "rot_y": r1_rot_y,
                    "rot_z": r1_rot_z,
                }.items(),
            ),
        ]
    )

    # --- Configuración del robot 2: UR5e ---
    r2_launch = GroupAction(
        actions=[
            # Empuja todos los nodos de este grupo al namespace configurado para r2
            PushRosNamespace("r2"),

            # Incluye el launch file de control original del driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur5_bringup"), "launch", "ur_control.launch.py"
                    ])
                ),
                launch_arguments={
                    "ur_type": r2_type,
                    "robot_ip": r2_IP,
                    "description_package": "ur5_description",
                    # Evitar f-strings con LaunchConfiguration: usar listas de sustituciones
                    "tf_prefix": ["r2", "_"],
                    "runtime_config_package": "ur_robot_driver",
                    "controllers_file": PathJoinSubstitution([
                        FindPackageShare("ur5_bringup"), "config", "ur_controllers_r2.yaml"
                    ]),
                    "kinematics_params_file": PathJoinSubstitution([
                        FindPackageShare("ur5_description"),
                        "config",
                        ["my_robot_calibration_", r2_type, ".yaml"],
                    ]),
                    "use_fake_hardware": use_fake_hardware_r2,
                    "launch_dashboard_client": "false", # Solo lanzamos un dashboard client
                    "launch_rviz": "false",
                    # Asignación explícita de puertos DIFERENTES para el hardware real
                    "reverse_port": PythonExpression(["str(int('", r2_TCP_port, "') - 1)"]),
                    "script_sender_port": r2_TCP_port,
                    "trajectory_port": PythonExpression(["str(int('", r2_TCP_port, "') + 1)"]),
                    "script_command_port": PythonExpression(["str(int('", r2_TCP_port, "') + 2)"]),
                    "pos_x": r2_x_pos,
                    "pos_y": r2_y_pos,
                    "pos_z": r2_z_pos,
                    "rot_x": r2_rot_x,
                    "rot_y": r2_rot_y,
                    "rot_z": r2_rot_z,
                }.items(),
            ),
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "rviz", "view_robot.rviz"]
    )
    
    #lanzar  un solo rviz para ambos robots
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    # Nodos de feedback háptico (uno por robot)
    feedback_r1_node = Node(
        package="ur5_torque",
        executable="feedback_node",
        name="feedback_r1",
        output="screen",
        parameters=[{
            "namespace": "r1",
            "ph_namespace": "phantom1",
            "force_filter_alpha": r1_force_filter_alpha,
        }],
        condition=IfCondition(launch_feedback),
    )

    feedback_r2_node = Node(
        package="ur5_torque",
        executable="feedback_node",
        name="feedback_r2",
        output="screen",
        parameters=[{
            "namespace": "r2",
            "ph_namespace": "phantom2",
            "force_filter_alpha": r2_force_filter_alpha,
        }],
        condition=IfCondition(launch_feedback),
    )
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        ),
        DeclareLaunchArgument(
            "r1_type", default_value="ur5e", description="Robot 1 type"
        ),
        DeclareLaunchArgument(
            "r2_type", default_value="ur5e", description="Robot 2 type"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware_r1",
            default_value="true",
            description="Use fake hardware for UR5.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware_r2",
            default_value="true",
            description="Use fake hardware for UR5e.",
        ),
        DeclareLaunchArgument(
            "r1_IP", default_value="192.168.10.104", description="Robot 1 IP Address"
        ),
        DeclareLaunchArgument(
            "r2_IP", default_value="192.168.10.103", description="Robot 2 IP Address"
        ),
        DeclareLaunchArgument(
            "r1_TCP_port", default_value="50002", description="Robot 1 TCP Port"
        ),
        DeclareLaunchArgument(
            "r2_TCP_port", default_value="50012", description="Robot 2 TCP Port"
        ),
        DeclareLaunchArgument(
            "r1_x_pos", default_value="0.0", description="Robot 1 X Position"
        ),
        DeclareLaunchArgument(
            "r1_y_pos", default_value="0.9", description="Robot 1 Y Position"
        ),
        DeclareLaunchArgument(
            "r1_z_pos", default_value="0.0", description="Robot 1 Z Position"
        ),
        DeclareLaunchArgument(
            "r1_rot_x", default_value="0.0", description="Robot 1 X Rotation"
        ),
        DeclareLaunchArgument(
            "r1_rot_y", default_value="0.0", description="Robot 1 Y Rotation"
        ),
        DeclareLaunchArgument(
            "r1_rot_z", default_value="0.0", description="Robot 1 Z Rotation"
        ),
        DeclareLaunchArgument(
            "r2_x_pos", default_value="0.0", description="Robot 2 X Position"
        ),
        DeclareLaunchArgument(
            "r2_y_pos", default_value="-0.9", description="Robot 2 Y Position"
        ),
        DeclareLaunchArgument(
            "r2_z_pos", default_value="0.0", description="Robot 2 Z Position"
        ),
        DeclareLaunchArgument(
            "r2_rot_x", default_value="0.0", description="Robot 2 X Rotation"
        ),
        DeclareLaunchArgument(
            "r2_rot_y", default_value="0.0", description="Robot 2 Y Rotation"
        ),
        DeclareLaunchArgument(
            "r2_rot_z", default_value="3.14", description="Robot 2 Z Rotation"
        ),
        DeclareLaunchArgument(
            "launch_feedback", default_value="true", description="Launch haptic force feedback nodes"
        ),
        DeclareLaunchArgument(
            "r1_force_filter_alpha", default_value="0.2", description="LPF alpha for r1 feedback (0..1)"
        ),
        DeclareLaunchArgument(
            "r2_force_filter_alpha", default_value="0.2", description="LPF alpha for r2 feedback (0..1)"
        ),
        r1_launch,
        r2_launch,
        feedback_r1_node,
        feedback_r2_node,
        #rviz_node,
    ])