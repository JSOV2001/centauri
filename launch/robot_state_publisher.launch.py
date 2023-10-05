#Librerias para Ubicar Archivos
import os
from ament_index_python import get_package_share_directory
#Librerias para Procesar Archivos
import xacro
#Librerias para Lanzar Nodos
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

pkg_filepath = os.path.join(get_package_share_directory("centauri"))

use_sim_time = LaunchConfiguration("use_sim_time")
urdf_filepath = os.path.join(pkg_filepath, "description", "centauri.urdf.xacro")
robot_description_file = xacro.process_file(urdf_filepath)

def generate_launch_description():
    use_sim_time_declaration = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    rsp_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml(),
                "use_sim_time": use_sim_time
            }
        ]
    )

    nodes_to_run = [use_sim_time_declaration, rsp_node]
    return LaunchDescription(nodes_to_run)
