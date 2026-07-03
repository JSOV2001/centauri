#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from centauri_interfaces.srv import KinematicGoal
from centauri_interfaces.action import TrajectoryGoal
from std_msgs.msg import Float64MultiArray
# Python libraries
import numpy as np
import time
import matplotlib.pyplot as plt

class TrajectoryServer(Node):
    def __init__(self):
        super().__init__("trajectory_server")

        # Set up a kinematics service to get the joints' desired position
        self.kinematics_client = self.create_client(KinematicGoal, "/centauri/kinematics")
        while not self.kinematics_client.wait_for_service(timeout_sec= 5.0):
            print("\nWaiting for an available kinematics server")

        self.trajectory_server = ActionServer(
            self,
            TrajectoryGoal,
            "/centauri/trajectory",
            self.process_goal
        )

        self.is_controller_enabled = True
        self.controller_topic = "/joint_group_position_controller/commands"
        self.controller_msg = Float64MultiArray()
        self.controller_pub = self.create_publisher(Float64MultiArray, self.controller_topic, 10)

        print("\nInitializing trajectory server")
    
    def process_goal(self, goal_handle: ServerGoalHandle):
        # Compute joints' desired position based on desired goal
        des_x = goal_handle.request.x
        des_y = goal_handle.request.y
        des_z = goal_handle.request.z
        t_final = goal_handle.request.time_range
        desired_goal = np.array([des_x, des_y, des_z], dtype= np.float64)
        
        request_future = self.request_kinematics_solution(desired_goal)
        rclpy.spin_until_future_complete(self, request_future)
        response = request_future.result()
        q_des = response.q_list

        # print(f"\nDesired θ, {np.around(q_des, 2)} rad, in {t_final} seconds")

        # Creating feedback
        feedback_msg = TrajectoryGoal.Feedback()

        # Processing goal
        # Generating each joint's trajectory
        q_length = len(q_des)
        q_cons = np.zeros((q_length, 6))
        for i in range(q_length):
            q_param = np.array([0, q_des[i], 0, 0, 0, 0]).T
            q_cons[i, :] = self.generate_traj_cons(q_param, 0, t_final)
        # print(f"\nTrajectory Constants Matrix:\n{q_cons}")
        
        # Generating time
        t_step = 0.01
        t_vec = np.arange(0, t_final + t_step, t_step, dtype= np.float64)
        t_legth = len(t_vec)
        # print(f"\nTime Vector:\n{t_vec}\n")
        
        q_final = np.zeros(q_length, dtype= np.float64)
        q_list_pos = np.zeros((q_length, t_legth), dtype= np.float64)
        q_list_vel = np.zeros((q_length, t_legth), dtype= np.float64)
        q_list_acc = np.zeros((q_length, t_legth), dtype= np.float64)
        
        for t_ind, t in enumerate(t_vec):
            q_pos = np.zeros(q_length, dtype= np.float64)

            for i in range(q_length):
                c0 = q_cons[i, 5]
                c1 = q_cons[i, 4]
                c2 = q_cons[i, 3]
                c3 = q_cons[i, 2]
                c4 = q_cons[i, 1]
                c5 = q_cons[i, 0]

                q_pos[i] = c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
                q_vel = c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
                q_acc = 2*c2 + 2*3*c3*t + 3*4*c4*t**2 + 4*5*c5*t**3
                
                q_list_pos[i, t_ind] = q_pos[i]
                q_list_vel[i, t_ind] = q_vel
                q_list_acc[i, t_ind] = q_acc
            
            # Sending feedback
            feedback_msg.q_curr = q_pos.tolist() # Must be a standard list instead of numpy-type
            goal_handle.publish_feedback(feedback_msg)

            # Sending current joint positions to gazebo
            if self.is_controller_enabled:
                self.controller_msg.data = q_pos.tolist()
                self.controller_pub.publish(self.controller_msg)    

            # print(f"\nFeedback θ: {np.around(q_pos, 2)} rad")
            if(t == t_final):
                q_final = q_pos
            
            time.sleep(t_step)
        
        # Creating and sending result
        result_msg = TrajectoryGoal.Result()
        result_msg.q_final = q_final.tolist() # Must be a standard list instead of numpy-type
        
        # Setting goal status
        goal_handle.succeed()

        print(f"\nResult θ: {np.around(q_pos, 2)} rad")
        return result_msg
    
        """
        # Plotting each joint's position, velocity and acceleration
        for ind in range(q_length):            
            plt.subplot(2, 3, ind + 1)
            plt.title(f'Q{ind + 1}')
            plt.ylim(-2, 2)
            plt.xlim(0, t_final)
            plt.plot(t_vec, q_list_pos[ind, :], 'r-.', label= f"Q{ind + 1}'s Position")
            plt.plot(t_vec, q_list_vel[ind, :], 'g--', label= f"Q{ind + 1}'s Velocity")
            plt.plot(t_vec, q_list_acc[ind, :], 'b:', label= f"Q{ind + 1}'s Acceleration")
            plt.grid(True)
            plt.legend()
        plt.show()
        """
    
    def request_kinematics_solution(self, desired_goal):
        self.kinematics_request_msg = KinematicGoal.Request()
        self.kinematics_request_msg.x = desired_goal[0]
        self.kinematics_request_msg.y = desired_goal[1]
        self.kinematics_request_msg.z = desired_goal[2]
        response = self.kinematics_client.call_async(self.kinematics_request_msg) 
        return response
    
    def generate_traj_cons(self, q_param, ts, tf):
        t_mat = np.array([
            [ts**5, ts**4, ts**3, ts**2, ts, 1],
            [tf**5, tf**4, tf**3, tf**2, tf, 1],
            [5*ts**4, 4*ts**3, 3*ts**2, 2*ts, 1, 0],
            [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
            [20*ts**3, 12*ts**2, 6*ts, 2, 0, 0],
            [20*tf**3, 12*tf**2, 6*tf, 2, 0, 0],
        ])

        t_inv = np.linalg.inv(t_mat)
        c_vec = np.dot(t_inv, q_param)
        return c_vec

def main():
    rclpy.init()
    trajectory_server_node = TrajectoryServer()
    try:
        rclpy.spin(trajectory_server_node)
    except KeyboardInterrupt:
        trajectory_server_node.destroy_node()
        rclpy.shutdown()
  
if __name__ == "__main__":
    main()