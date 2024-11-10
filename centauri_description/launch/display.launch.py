# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

# URDF's filepath
pkg_filepath = get_package_share_directory("centauri_description")
xacro_filepath = os.path.join(pkg_filepath, "description", "centauri.urdf.xacro")
robot_description_file = xacro.process_file(xacro_filepath)
rviz_config_filepath = os.path.join(pkg_filepath, "config", "display.rviz")

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

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_config_filepath]
    )

    jsp_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
    )

    nodes_to_run = [
        robot_state_publisher_node,
        rviz_cmd, 
        jsp_gui_node
    ]
    return LaunchDescription(nodes_to_run)
       
