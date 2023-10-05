#!/usr/bin/env python3
#Librerias de ROS2
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
#Librerias de Python
import math

class JointGroupTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("joint_trajectory_publisher")
        
        self.joint_group_trajectory_publisher = self.create_publisher(
            JointTrajectory, 
            "/joint_trajectory_controller/joint_trajectory", 
            10
        )
        
        self.timer = self.create_timer(1.0, self.publish_trajectory_in_joints)
        
        self.joint_group_names = [
            "joint1", 
            "joint2", 
            "joint3", 
            "joint4", 
            "joint5", 
            "joint6"
        ]

        self.goal_positions = [
            math.radians(45), #Posicion Final del Joint 1
            math.radians(45), #Posicion Final del Joint 2
            math.radians(45), #Posicion Final del Joint 3
            math.radians(-45), #Posicion Final del Joint 4
            math.radians(-45), #Posicion Final del Joint 5
            math.radians(-45) #Posicion Final del Joint 6
        ]

        self.joint_group_trajectory_msg = JointTrajectory()
        
    def publish_trajectory_in_joints(self):
        self.joint_group_trajectory_msg.joint_names = self.joint_group_names
        
        goal_point = JointTrajectoryPoint()
        goal_point.positions = self.goal_positions
        goal_point.time_from_start = Duration(sec = 10)

        self.joint_group_trajectory_msg.points.append(goal_point)

        self.joint_group_trajectory_publisher.publish(self.joint_group_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_group_trajectory_publisher_node = JointGroupTrajectoryPublisher()
    try:
        rclpy.spin(joint_group_trajectory_publisher_node)
    except KeyboardInterrupt:
        joint_group_trajectory_publisher_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()