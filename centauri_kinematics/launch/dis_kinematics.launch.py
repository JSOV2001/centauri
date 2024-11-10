# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription

# URDF's filepath
description_filepath = get_package_share_directory("centauri_description")
xacro_filepath = os.path.join(description_filepath, "description", "centauri.urdf.xacro")
robot_description_file = xacro.process_file(xacro_filepath)
rviz_filepath = os.path.join(description_filepath, "config", "display.rviz")

def generate_launch_description():
    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml()
            }
        ]
    )

    kinematics_server_node = Node(
        package="centauri_kinematics",
        executable="kinematics_server.py"
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_filepath]
    )

    nodes_to_run = [
        robot_state_publisher_node,
        rviz_cmd,
        kinematics_server_node
    ]
    return LaunchDescription(nodes_to_run)
