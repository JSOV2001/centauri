# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

description_filepath = get_package_share_directory("centauri_description")
control_filepath = os.path.join(description_filepath,'launch','control.launch.py')
rviz_config_filepath = os.path.join(description_filepath, "config", "display.rviz")

def generate_launch_description():
    kinematics_server_node = Node(
        package="centauri_kinematics",
        executable="kinematics_server.py"
    )

    home_server_node = Node(
        package="centauri_kinematics",
        executable="home_server.py",
        output={
            'stdout': 'screen',
            'stderr': 'screen'
        }
    )
    
    gripper_server_node = Node(
        package="centauri_description",
        executable="gripper_server.py"
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_filepath]), 
        launch_arguments= {
            'use_sim_time': 'true', 
            'use_ros2_control': 'true'
        }.items()
    )
    
    nodes_to_run = [
        control_launch,
        kinematics_server_node,
        gripper_server_node,
        home_server_node
    ]
    return LaunchDescription(nodes_to_run)
