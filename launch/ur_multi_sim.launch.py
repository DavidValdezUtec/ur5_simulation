from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Arguments for robot 1 (UR5)
    ur_type_1 = "ur5"
    safety_limits_1 = LaunchConfiguration("safety_limits_1")
    safety_pos_margin_1 = LaunchConfiguration("safety_pos_margin_1")
    safety_k_position_1 = LaunchConfiguration("safety_k_position_1")
    runtime_config_package_1 = LaunchConfiguration("runtime_config_package_1")
    controllers_file_1 = LaunchConfiguration("controllers_file_1")
    description_package_1 = LaunchConfiguration("description_package_1")
    description_file_1 = LaunchConfiguration("description_file_1")
    # Retrieve the actual prefix from LaunchConfiguration for robot 1
    prefix_1 = LaunchConfiguration("prefix_1")
    start_joint_controller_1 = LaunchConfiguration("start_joint_controller_1")
    initial_joint_controller_1 = LaunchConfiguration("initial_joint_controller_1")

    # Arguments for robot 2 (UR5e)
    ur_type_2 = "ur5e"  # Fixed type for the second robot
    safety_limits_2 = LaunchConfiguration("safety_limits_2")
    safety_pos_margin_2 = LaunchConfiguration("safety_pos_margin_2")
    safety_k_position_2 = LaunchConfiguration("safety_k_position_2")
    runtime_config_package_2 = LaunchConfiguration("runtime_config_package_2")
    controllers_file_2 = LaunchConfiguration("controllers_file_2")
    description_package_2 = LaunchConfiguration("description_package_2")
    description_file_2 = LaunchConfiguration("description_file_2")
    # Retrieve the actual prefix from LaunchConfiguration for robot 2
    prefix_2 = LaunchConfiguration("prefix_2")
    start_joint_controller_2 = LaunchConfiguration("start_joint_controller_2")
    initial_joint_controller_2 = LaunchConfiguration("initial_joint_controller_2")

    # General arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    # Paths for Robot 1 (used to load parameters)
    initial_joint_controllers_path_1 = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package_1), "config", controllers_file_1]
    )
    rviz_config_file_1 = PathJoinSubstitution(
        [FindPackageShare(description_package_1), "rviz", "view_robot.rviz"]
    )

    # Robot Description for Robot 1
    # Pass the prefix_1 directly to the xacro command
    robot_description_content_1 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package_1), "urdf", description_file_1]
            ),
            " ",
            "safety_limits:=",
            safety_limits_1,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin_1,
            " ",
            "safety_k_position:=",
            safety_k_position_1,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type_1,
            " ",
            "prefix:=",  # Use prefix here for the URDF
            prefix_1, # Use LaunchConfiguration for prefix here
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers_path_1, # Pass the path to the controller YAML
        ]
    )
    robot_description_1 = {"robot_description": robot_description_content_1}

    # Paths for Robot 2 (used to load parameters)
    initial_joint_controllers_path_2 = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package_2), "config", controllers_file_2]
    )
    rviz_config_file_2 = PathJoinSubstitution(
        [FindPackageShare(description_package_2), "rviz", "view_robot.rviz"]
    )

    # Robot Description for Robot 2
    # Pass the prefix_2 directly to the xacro command
    robot_description_content_2 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package_2), "urdf", description_file_2]
            ),
            " ",
            "safety_limits:=",
            safety_limits_2,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin_2,
            " ",
            "safety_k_position:=",
            safety_k_position_2,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type_2,
            " ",
            "prefix:=",  # Use prefix here for the URDF
            prefix_2, # Use LaunchConfiguration for prefix here
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers_path_2, # Pass the path to the controller YAML
        ]
    )
    robot_description_2 = {"robot_description": robot_description_content_2}

    # Robot State Publisher for Robot 1
    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description_1],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace=prefix_1,  # Apply namespace
    )

    # Robot State Publisher for Robot 2
    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description_2],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace=prefix_2,  # Apply namespace
    )

    # RViz Node (can visualize both if their topics are remapped correctly or by using namespaces)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file_1],
        condition=IfCondition(launch_rviz),
    )

    # Joint State Broadcaster Spawner for Robot 1
    joint_state_broadcaster_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", [prefix_1, "controller_manager"]],
        parameters=[
            initial_joint_controllers_path_1, # Load the YAML
            {"robot_prefix": prefix_1}, # Pass the robot_prefix parameter
        ],
        namespace=prefix_1,
    )

    # Joint State Broadcaster Spawner for Robot 2
    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", [prefix_2, "controller_manager"]],
        parameters=[
            initial_joint_controllers_path_2, # Load the YAML
            {"robot_prefix": prefix_2}, # Pass the robot_prefix parameter
        ],
        namespace=prefix_2,
    )

    # Delay rviz start after `joint_state_broadcaster` (for Robot 1)
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_1,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # Initial Joint Controller Spawner for Robot 1
    initial_joint_controller_spawner_started_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_1, "-c", [prefix_1, "controller_manager"]],
        parameters=[
            initial_joint_controllers_path_1, # Load the YAML
            {"robot_prefix": prefix_1}, # Pass the robot_prefix parameter
        ],
        condition=IfCondition(start_joint_controller_1),
        namespace=prefix_1,
    )
    initial_joint_controller_spawner_stopped_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_1, "-c", [prefix_1, "controller_manager"], "--stopped"],
        parameters=[
            initial_joint_controllers_path_1, # Load the YAML
            {"robot_prefix": prefix_1}, # Pass the robot_prefix parameter
        ],
        condition=UnlessCondition(start_joint_controller_1),
        namespace=prefix_1,
    )

    # Initial Joint Controller Spawner for Robot 2
    initial_joint_controller_spawner_started_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_2, "-c", [prefix_2, "controller_manager"]],
        parameters=[
            initial_joint_controllers_path_2, # Load the YAML
            {"robot_prefix": prefix_2}, # Pass the robot_prefix parameter
        ],
        condition=IfCondition(start_joint_controller_2),
        namespace=prefix_2,
    )
    initial_joint_controller_spawner_stopped_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_2, "-c", [prefix_2, "controller_manager"], "--stopped"],
        parameters=[
            initial_joint_controllers_path_2, # Load the YAML
            {"robot_prefix": prefix_2}, # Pass the robot_prefix parameter
        ],
        condition=UnlessCondition(start_joint_controller_2),
        namespace=prefix_2,
    )

    # GZ nodes for Robot 1
    gz_spawn_entity_1 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content_1,
            "-name",
            "ur1",  # Unique name in Gazebo
            "-allow_renaming",
            "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.0", # Initial position
        ],
    )

    # GZ nodes for Robot 2
    gz_spawn_entity_2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content_2,
            "-name",
            "ur2",  # Unique name in Gazebo
            "-allow_renaming",
            "true",
            "-x", "0.5", "-y", "0.5", "-z", "0.0", # Different initial position
        ],
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": ["-r", "-v", "4", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": ["-s", "-r", "-v", "4", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    nodes_to_start = [
        # Robot 1
        robot_state_publisher_node_1,
        joint_state_broadcaster_spawner_1,
        initial_joint_controller_spawner_stopped_1,
        initial_joint_controller_spawner_started_1,
        gz_spawn_entity_1,
        # Robot 2
        robot_state_publisher_node_2,
        joint_state_broadcaster_spawner_2,
        initial_joint_controller_spawner_stopped_2,
        initial_joint_controller_spawner_started_2,
        gz_spawn_entity_2,
        # Common nodes
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge,
        delay_rviz_after_joint_state_broadcaster_spawner, # RViz only needs to be launched once
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # Arguments for Robot 1 (UR5)
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type_1",
            description="Type/series of UR robot 1.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits_1",
            default_value="true",
            description="Enables the safety limits controller for robot 1 if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin_1",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller for robot 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position_1",
            default_value="20",
            description="k-position factor in the safety controller for robot 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package_1",
            default_value="ur5_simulation",
            description='Package with the controller\'s configuration for robot 1 in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file_1",
            default_value="ur_multi_controllers.yaml",
            description="YAML file with the controllers configuration for robot 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_1",
            default_value="ur5_simulation",
            description="Description package with robot 1 URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file_1",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with robot 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix_1",
            default_value="robot1/",
            description="Prefix of the joint names for robot 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller_1",
            default_value="true",
            description="Enable headless mode for robot 1 control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller_1",
            default_value="joint_trajectory_controller",
            description="Robot 1 controller to start.",
        )
    )

    # Arguments for Robot 2 (UR5e)
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type_2",
            description="Type/series of UR robot 2.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e", # Fixed to ur5e
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits_2",
            default_value="true",
            description="Enables the safety limits controller for robot 2 if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin_2",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller for robot 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position_2",
            default_value="20",
            description="k-position factor in the safety controller for robot 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package_2",
            default_value="ur5_simulation", # You might want a different package for ur5e
            description='Package with the controller\'s configuration for robot 2 in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file_2",
            default_value="ur_multi_controllers.yaml", # You might want a different controller file for ur5e
            description="YAML file with the controllers configuration for robot 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_2",
            default_value="ur5_simulation", # You might want a different package for ur5e
            description="Description package with robot 2 URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file_2",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with robot 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix_2",
            default_value="robot2/",
            description="Prefix of the joint names for robot 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller_2",
            default_value="true",
            description="Enable headless mode for robot 2 control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller_2",
            default_value="joint_trajectory_controller",
            description="Robot 2 controller to start.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])