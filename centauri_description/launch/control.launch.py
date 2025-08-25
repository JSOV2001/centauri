# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable

# Get URDF filepath
desc_pkg_filepath = get_package_share_directory("centauri_description")
xacro_filepath = os.path.join(desc_pkg_filepath, "description", "centauri.urdf.xacro")
robot_description_file = xacro.process_file(xacro_filepath)

# Get Gazebo's filepaths
gz_pkg_filepath = get_package_share_directory('ros_gz_sim')
gz_gui_filepath = os.path.join(gz_pkg_filepath, 'launch', 'gz_sim.launch.py')
gz_bridge_filepath = os.path.join(desc_pkg_filepath, "config", "gz_bridge.yaml")

# Define ROS2 arguments for launch configutation
use_sim_time = LaunchConfiguration("use_sim_time")
use_ros2_control = LaunchConfiguration('use_ros2_control')

def generate_launch_description():
    # Run GazeboSim processes
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value= "true", 
        description= "Use sim time if true"
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml(),
                "use_sim_time": use_sim_time
            }
        ]
    )

    gz_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_gui_filepath]),
        launch_arguments= {'gz_args': ['-r -v4']}.items()
    )

    gz_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(desc_pkg_filepath, 'meshes')
    )
    
    gz_create_node = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'centauri',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    gz_params_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        parameters= [gz_bridge_filepath]
    )

    # Run ros2_control processes
    ros2_control_node = Node(
        package= "controller_manager",
        executable= "ros2_control_node",
        remappings=[
            ("~/robot_description", "robot_description")
        ]
    )

    jsb_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["joint_state_broadcaster"]
    )
    
    jgpc_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["joint_group_position_controller"]
    )
    
    gc_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["gripper_controller"]
    )

    nodes_to_run = [
        use_sim_time_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        gz_gui_launch, 
        gz_env_vars,
        gz_create_node,
        gz_params_node,
        ros2_control_node,
        jsb_spawner_node,
        jgpc_spawner_node,
        gc_spawner_node
    ]
    return LaunchDescription(nodes_to_run)