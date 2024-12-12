#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from centauri_interfaces.srv import KinematicGoal
# Python libraries
import numpy as np

class KinematicsClient(Node):
    def __init__(self):
        super().__init__("kinematics_client")
        self.srv_request = KinematicGoal.Request()

        self.client = self.create_client(KinematicGoal, "kinematics")
        while not self.client.wait_for_service(timeout_sec= 5.0):
            print("Waiting for an available service")
        
        self.s_total = 50
    
    def send_request(self, input):
        self.srv_request.x = input[0]
        self.srv_request.y = input[1]
        self.srv_request.z = input[2]
        response = self.client.call_async(self.srv_request) 
        return response
    
    def send_linear_path(self, p_start, p_end):
        p_start = np.array(p_start)
        p_end = np.array(p_end)
        s_list = np.linspace(0, 1, self.s_total)
        for s in s_list:
            goal = np.around(p_start + s * (p_end- p_start), 3)
            i = np.where(s_list == s)[0][0]

            # Send goal to server
            future = self.send_request(list(goal))

            # Get answer from server
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            q_rad = np.array(response.q_list)
            q_deg = np.rad2deg(q_rad)
            e_norm = response.error
            total_iters = response.iters

            print(f"\nRequest #{i + 1}:")
            print(f"\tXYZ: {goal} m")
            print(f"\tθ: {np.around(q_deg, 2)} ")
            print(f"\tError: {np.around(e_norm * 100, 2)} %")
            print(f"\tTotal Iterations: {total_iters}")

def main():
    rclpy.init()
    node = KinematicsClient()
    try:
        node.send_linear_path([0.350, -0.125, 0.176], [0.350, 0.000, 0.352])
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()