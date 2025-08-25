#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from centauri_interfaces.action import TrajectoryGoal
from std_srvs.srv import SetBool
# Python libraries
import numpy as np
import sys

class TrajectoryClient(Node):
    def __init__(self, des_goal, time_range):
        self.des_goal = des_goal
        self.time_range = time_range

        super().__init__("trajectory_client")

        self.trajectory_client = ActionClient(self, TrajectoryGoal, "/trajectory")

        self.client = self.create_client(SetBool, "/gripper")
        while not self.client.wait_for_service(timeout_sec= 5.0):
            print("Waiting for an available service")
        self.srv_request = SetBool.Request()

        print("Initializing trajectory client")
    
    def send_goal(self):
        # Wait for server
        self.trajectory_client.wait_for_server(3)

        # Create goal
        goal_msg = TrajectoryGoal.Goal()
        goal_msg.x = self.des_goal[0]
        goal_msg.y = self.des_goal[1]
        goal_msg.z = self.des_goal[2]
        goal_msg.time_range = self.time_range
        
        # Send goal asincronally
        status_future = self.trajectory_client.send_goal_async(goal_msg, self.get_feedback)
        status_future.add_done_callback(self.get_goal_status)
    
    def get_goal_status(self, future_msg):
        self.goal_handle: ClientGoalHandle = future_msg.result()
        if self.goal_handle.accepted:
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.get_goal_result)
        else:
            print("Goal wasn't acomplished")
    
    def get_goal_result(self, future_msg):
        result_msg = future_msg.result().result
        q_final = result_msg.q_final
        print(f"Got to θ: {np.around(np.rad2deg(q_final), 2)} °")

        enable_gripper = False
        gripper_state = "on"
        
        if (gripper_state == "on"):
            enable_gripper = True
        else:
            enable_gripper = False
        
        # Get answer from server
        gripper_request_future = self.send_gripper_request(enable_gripper)
        rclpy.spin_until_future_complete(self, gripper_request_future)
        gripper_response = gripper_request_future.result()
        gripper_msg = gripper_response.message
        print(gripper_msg)
        
    def get_feedback(self, feedback_msg):
        feedback_msg = feedback_msg.feedback
        q_curr = feedback_msg.q_curr
        print(f"Current θ: {np.around(np.rad2deg(q_curr), 2)} °")
    
    def send_gripper_request(self, enable_gripper):
        self.srv_request.data = enable_gripper
        response = self.client.call_async(self.srv_request) 
        return response

def main():
    rclpy.init()
    des_x = float(sys.argv[1])
    des_y = float(sys.argv[2])
    des_z = float(sys.argv[3])
    des_goal = np.array([des_x, des_y, des_z], dtype= np.float64)
    time_range = float(sys.argv[4])
    # Goal Example: 0.351 -0.288 0.351 5
    node = TrajectoryClient(des_goal, time_range)
    try:
        # Send desired goal to serve
        print(f"Get to {des_goal} m in {time_range} seconds")
        node.send_goal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()