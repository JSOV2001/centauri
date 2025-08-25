#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64MultiArray
# Python libraries
import numpy as np

class HomeServer(Node):
    def __init__(self):
        super().__init__("kinematics_server")

        self.q_home = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])

        self.home_srv = self.create_service(SetBool, "/home", self.home_callback)

        self.controller_topic = "/joint_group_position_controller/commands"
        self.controller_pub = self.create_publisher(Float64MultiArray, self.controller_topic, 10)
        self.controller_msg = Float64MultiArray()
    
    # Get client's request for home configuration
    def home_callback(self, request, response):
        if request.data:
            # Publish q_list in Gazebo's controller
            self.controller_msg.data = self.q_home.tolist()
            self.controller_pub.publish(self.controller_msg)
            response.success = True
            response.message = "Change to the home configuration"
        else:
            response.success = False
            response.message = "Stay at the same configuration"
        
        # Send server's response to client
        return response

def main():
    rclpy.init()
    node = HomeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
  
if __name__ == "__main__":
    main()