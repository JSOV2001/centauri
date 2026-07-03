#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64MultiArray
# Python libraries
import numpy as np
import time

class GripperServer(Node):
    def __init__(self):
        print("Ready")
        super().__init__('gripper_server')
        self.close_to = np.deg2rad(20)
        self.open_to = np.deg2rad(5)
        
        self.controller_topic  =  "/gripper_controller/commands"
        self.controller_pub  =  self.create_publisher(Float64MultiArray, self.controller_topic, 10)
        self.controller_msg  =  Float64MultiArray()

        self.gripper_srv = self.create_service(SetBool, "/gripper", self.gripper_callback)
    
    # Get client's request for gripper
    def gripper_callback(self, request, response):
        # Close gripper
        if request.data:
            pos_list = np.linspace(0.0, self.close_to, 50)
            for pos_curr in pos_list:
                self.controller_msg.data = [-pos_curr, pos_curr, -pos_curr, pos_curr, -pos_curr, pos_curr]
                self.controller_pub.publish(self.controller_msg)
                time.sleep(0.1)
            response.success = False
            response.message = "Gripper openned"
        # Open gripper
        else:
            pos_list = np.linspace(-self.close_to, -self.open_to, 50)
            print(pos_list)
            for pos_curr in pos_list:
                self.controller_msg.data = [pos_curr, -pos_curr, pos_curr, -pos_curr, pos_curr, -pos_curr]
                self.controller_pub.publish(self.controller_msg)
                time.sleep(0.1)
            response.success = True
            response.message = "Gripper closed"
        
        # Send server's response to client
        return response
    
    def open_gripper(self):
        while self.pos_curr >= self.pos_final:
            if (self.pos_final <= 0):
                # The order is the same described as yaml file
                self.controller_msg.data = [
                    self.pos_curr, # Gripper's right gear joint
                    -self.pos_curr, # Gripper's right finger joint
                    self.pos_curr, # Gripper's right pivot joint
                    -self.pos_curr, # Gripper's Left gear joint
                    self.pos_curr, # Gripper's Left finger joint
                    -self.pos_curr  # Gripper's Left pivot joint
                ]
                
                self.controller_pub.publish(self.controller_msg)
                self.controller_rate.sleep()

                self.pos_curr -= np.deg2rad(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
