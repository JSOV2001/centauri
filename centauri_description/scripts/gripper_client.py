#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys

class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.client = self.create_client(SetBool, "/gripper")
        while not self.client.wait_for_service(timeout_sec= 5.0):
            print("Waiting for an available service")
        self.srv_request = SetBool.Request()
    
    def send_request(self, enable_gripper):
        self.srv_request.data = enable_gripper
        response = self.client.call_async(self.srv_request) 
        return response

def main():
    rclpy.init()
    node = GripperClient()
    enable_gripper = False
    gripper_state = str(sys.argv[1])
    
    if (gripper_state == "on"):
        enable_gripper = True
    else:
        enable_gripper = False
    
    try:
        future = node.send_request(enable_gripper)

        # Get answer from server
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        response_msg = response.message
        
        print(response_msg)
    
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()