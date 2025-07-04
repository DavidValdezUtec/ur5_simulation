
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml, xacro

#https://github.com/bponsler/ros2-support

def generate_launch_description():

    package_path = get_package_share_directory("ur5_simulation")
    
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value='true', description="Use simulator time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_type = ["ur5","ur5e"]  # ROBOT_MODEL  #LaunchConfiguration("robot")
    
    world = LaunchConfiguration("world")
    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            package_path,
            "worlds",
            "empty.world"
        ),
        description="Full path to world model file to load",
    )


    

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            world,
        ],
        output="screen",
    )
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")




    ld = LaunchDescription()
    ld.add_action(declare_world_path)
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    robots = [
            {'name': 'arm1', 'x_pose': '-1.5', 'y_pose': '-1.50', 'Y':'0.0'},
            {'name': 'arm2', 'x_pose': '-1.5', 'y_pose': '1.5', 'Y':'0.0'},
            #{'name': 'arm3', 'x_pose': '1.5', 'y_pose': '-1.5', 'Y':'-3.14'},
            #{'name': 'arm4', 'x_pose': '1.5', 'y_pose': '1.5', 'Y':'-3.14'},
            # …
            # …
        ]

    # Multiple ARMs in gazebo must be spawned in a serial fashion due to 
    # a global namespace dependency introduced by ros2_control.
    # robot_final_action is the last action from previous robot and 
    # new robot will only be spawned once previous action is completed
    robot_final_action = None
    i=0
    for robot in robots:    
        
        declare_robot_type = DeclareLaunchArgument(
            name="ur_type", default_value=robot_type[i], description="Robot type"
        )
        ld.add_action(declare_robot_type)
        robot_final_action = spawn_robot(
            ld,
            robot_type[i],
            robot["name"] ,
            use_sim_time,
            robot["x_pose"],
            robot["y_pose"],
            robot["Y"],
            robot_final_action,
        )
        i += 1
        if i >= len(robots):
            i = 0

    return ld


def spawn_robot(
        ld, robot_type, robot_name, use_sim_time, x, y, Y,
        previous_final_action=None):

    package_path = get_package_share_directory("ur5_simulation")
    namespace = "/" + robot_name

    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = RewrittenYaml(
        source_file=package_path
        + "/config/" + "/ur_controllers.yaml",
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    context = LaunchContext()
    controller_paramfile = configured_params.perform(context)
    xacro_path = os.path.join(package_path, "urdf", "ur2.urdf.xacro")

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={
            "name": robot_name,
            "ur_type": robot_type,
            "robot_namespace": namespace,
            "sim_gazebo": "1",
            "simulation_controllers": controller_paramfile,
            "safety_limits": "true",
            "tf_prefix": "",
        },
    )

    robot_urdf = robot_doc.toprettyxml(indent="  ")


    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_params = {"robot_description": robot_urdf,
                    "use_sim_time": use_sim_time}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        output="screen",
        remappings=remappings,
        parameters=[robot_params],
    )

    robot_description = {"robot_description": robot_urdf}

  


    
    # ROS2 Controller Manager in Foxy uses 'start' while Humble version expects 'active'

    controller_run_state = 'active'
    

    robot_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", namespace + "/robot_description",
            "-entity", robot_name,
            "-robot_namespace", namespace,
            "-x", x,
            "-y", y,
            "-z", "0.0",
            "-Y", Y,
            "-unpause",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "joint_state_broadcaster",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    load_arm_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "joint_trajectory_controller",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )
   
    # Set initial joint position for robot.   This step is not needed for Humble 
    # In Humble, initial positions are taken from initial_positions.yaml and set by ros2 control plugin

    
    if previous_final_action is not None:
        spawn_entity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_final_action,
                on_exit=[robot_spawn_entity],
            )
        )
    else:
        spawn_entity = robot_spawn_entity

    state_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_arm_trajectory_controller],
        )
    )



    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(state_controller_event)
    ld.add_action(arm_controller_event)

    return load_arm_trajectory_controller


def load_file(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
