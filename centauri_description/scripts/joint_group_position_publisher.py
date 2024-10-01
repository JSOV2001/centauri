#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# Python libraries
from numpy import deg2rad, rad2deg

class JointGroupPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')
        self.pos_final = deg2rad(15)
        self.pos_curr = 0.0
        
        self.controller_topic  =  "/joint_group_position_controller/commands"
        self.controller_pub  =  self.create_publisher(Float64MultiArray, self.controller_topic, 10)
        self.controller_msg  =  Float64MultiArray()
        
        self.timer = self.create_timer(0.1, self.publish_joint_group_position)
    
    def publish_joint_group_position(self):
		# The order is the same described as yaml file
        self.controller_msg.data = [
             self.pos_curr, # Joint 1
             self.pos_curr, # Joint 2
             self.pos_curr, # Joint 3
             self.pos_curr, # Joint 4
             self.pos_curr, # Joint 5
             self.pos_curr  # Joint 6
        ]
        self.controller_pub.publish(self.controller_msg)
        
        print("\nCurrent Value in Joints:")
        print(f"Joint Group's Position: {round(rad2deg(self.pos_curr), 2)} °")

        self.pos_curr += deg2rad(0.5)
        if(self.pos_curr >= self.pos_final):
            self.pos_curr = self.pos_final
        
def main(args=None):
    rclpy.init(args=args)
    node = JointGroupPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
