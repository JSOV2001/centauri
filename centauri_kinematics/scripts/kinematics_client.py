#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from centauri_interfaces.srv import KinematicGoal
# Python libraries
import numpy as np
import sys

class KinematicsClient(Node):
    def __init__(self):
        super().__init__("kinematics_client")

        self.client = self.create_client(KinematicGoal, "kinematics")
        while not self.client.wait_for_service(timeout_sec= 5.0):
            print("Waiting for an available service")
        self.srv_request = KinematicGoal.Request()
    
    def send_request(self, goal):
        self.srv_request.x = goal[0]
        self.srv_request.y = goal[1]
        self.srv_request.z = goal[2]
        response = self.client.call_async(self.srv_request) 
        return response

# Available goals
# Goal 1: 0.351 -0.288 0.351
# Goal 2: 0.161 -0.288 0.547
# Goal 3: 0.351 0.001 0.405
def main():
    rclpy.init()
    node = KinematicsClient()

    x_des = float(sys.argv[1])
    y_des = float(sys.argv[2])
    z_des = float(sys.argv[3])
    
    try:
        # Send desired goal to server
        goal = [x_des, y_des, z_des]
        future = node.send_request(goal)
        print("\nRequest:")
        print(f"\tX: {goal[0]} m")
        print(f"\tY: {goal[1]} m")
        print(f"\tZ: {goal[2]} m")
        
        # Get answer from server
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        q_list = response.q_list
        total_iters = response.iters
        e_norm = response.error
        success = response.success
        
        if success:
            print(f"\nResponse:")
            print(f"\tθ: {np.around(np.rad2deg(q_list), 2)} °")
            print(f"\tError: {np.around(e_norm * 100, 2)} %")
            print(f"\tTotal Iterations: {total_iters}")
        else:
            print(f"\nResponse:")
            print(f"\tNo viable solution at {total_iters} iterations")
            print(f"\tGot a {np.around(e_norm*100, 2)} % error")
    
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()