#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from centauri_interfaces.srv import KinematicGoal
from std_msgs.msg import Float64MultiArray
# Python libraries
import numpy as np
from numpy import pi, rad2deg, around
from numpy.linalg import pinv

class KinematicsServer(Node):
    def __init__(self):
        super().__init__("kinematics_publisher")
        print("\nWaiting for a client's request")
        
        # Define Centauri's links
        self.L1 = 17/100 # In cm
        self.L2 = 22/100 # In cm
        self.L3 = 22/100 # In cm
        self.L4 = 10/100 # In cm

        # Define Centauri's DH parameters
        self.d_list = np.array([self.L1, 0.0, 0.0, self.L3, 0.0, self.L4])
        self.a_list = np.array([0.0, self.L2, 0.0, 0.0, 0.0, 0.0])
        self.α_list = np.array([pi/2, 0.0, -pi/2, pi/2, -pi/2, 0.0])

        self.controller_topic = "/joint_group_position_controller/commands"
        self.controller_pub = self.create_publisher(Float64MultiArray, self.controller_topic, 10)
        self.controller_msg = Float64MultiArray()

        self.kinematics_srv = self.create_service(
            KinematicGoal, 
            "/kinematics", 
            self.kinematics_callback
        )

    def kinematics_callback(self, request, response):
        # Define desired goal
        goal = [request.x, request.y, request.z]
        goal = np.array(goal)
        print(f"\nGoal: {goal}")

        # Define Centauri's DH parameters
        q1, q2, q3, q4, q5, q6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        q_init = np.array([q1 + pi/2, q2 - pi/2, q3 + pi/2, q4, q5, q6])

        dh_params = [
            q_init, 
            self.d_list, 
            self.a_list, 
            self.α_list
        ] 

        # Compute inverse kinematics
        q_solv = self.compute_ikine(goal, dh_params)

        self.controller_msg.data = [
            q_solv[0],
            q_solv[1],
            q_solv[2],
            q_solv[3],
            q_solv[4],
            q_solv[5]
        ]
        self.controller_pub.publish(self.controller_msg)

        response.q = self.controller_msg.data
        print(f"\nSolution: {around(q_solv, 2)}")
        
        return response
    
    # Compute homogeneus transformation matrix
    def compute_dh_row(self, q, d, a, α): 
        m11 = np.cos(q)
        m12 = -np.cos(α) * np.sin(q)
        m13 = np.sin(α)*np.sin(q)
        m14 = a*np.cos(q)

        m21 = np.sin(q)
        m22 = np.cos(α) * np.cos(q)
        m23 = -np.sin(α)*np.cos(q)
        m24 = a*np.sin(q)

        m31 = 0.0
        m32 = np.sin(α)
        m33 = np.cos(α)
        m34 = d
        
        htm_mat = [
            [m11, m12,  m13, m14],
            [m21, m22,  m23, m24],
            [m31, m32,  m33, m34],
            [0.0, 0.0,  0.0, 1.0]
        ]
        htm_mat = np.array(htm_mat)
        return htm_mat

    # Compute forward kinematics numerically
    def compute_fkine(self, q_list, d_list, a_list, α_list): 
        htm_list = []

        for i in range(0, len(a_list)):
            htm_curr = self.compute_dh_row(q_list[i], d_list[i], a_list[i], α_list[i])
            htm_list.append(htm_curr)
        
        htm_mat = htm_list[0]
        for i in range(1, len(htm_list)):
            htm_mat = np.dot(htm_mat, htm_list[i])
        
        return htm_mat
    
    # Adjust joint position to real limitations
    def adjust_q(self, q_solv):
        q_adjusted = np.zeros(q_solv.shape)
        
        for i in range(0, len(q_solv)):
            while q_solv[i] > 2 * pi:
                q_solv[i] -= 2*pi
            while q_solv[i] < -2 * pi:
                q_solv[i] += 2*pi
            q_adjusted[i] = q_solv[i]
        
        return q_adjusted
    
    # Get position from homogeneus transformation matrix
    def get_xyz_from_htm(self, fkine_mat):
        x = fkine_mat[0, 3]
        y = fkine_mat[1, 3]
        z = fkine_mat[2, 3]
        
        xyz = np.array([x, y, z])
        return xyz

    # Compute jacobian numerically
    def compute_jacobian(self, q_solv, d_list, a_list, α_list, goal):
        delta = 0.00000001
        goal_length = len(goal)
        q_length = len(q_solv)
        J_shape = (goal_length, q_length)
        J_curr = np.zeros(J_shape)

        for k in range(len(q_solv)):
            q_plus_delta = np.copy(q_solv)
            q_plus_delta[k] += delta

            fkine_mat = self.compute_fkine(q_solv, d_list, a_list, α_list)
            fkine_coor = self.get_xyz_from_htm(fkine_mat)
            
            fkine_plus_mat = self.compute_fkine(q_plus_delta, d_list, a_list, α_list)
            fkine_plus_coor = self.get_xyz_from_htm(fkine_plus_mat)
            
            J_curr[:, k] = (1/delta) * (fkine_plus_coor - fkine_coor)
        
        return J_curr

    # Compute inverse kinematics numerically
    def compute_ikine(self, goal, dh_params):
        q_solv = dh_params[0]
        d_list = dh_params[1]
        a_list = dh_params[2]
        α_list = dh_params[3]

        e_max = 0.1/100

        # Compute gradient method numerically
        max_iters = 500
        i = 0
        e_curr = 0
        e_norm = 1.0

        while e_norm > e_max and i < max_iters:
            J_curr = self.compute_jacobian(q_solv, d_list, a_list, α_list, goal)
            J_inv = pinv(J_curr)
                
            fkine_curr = self.compute_fkine(q_solv, d_list, a_list, α_list)
            xyz_curr = self.get_xyz_from_htm(fkine_curr)
            
            # Compute new q-values
            e_curr = goal - xyz_curr
            q_solv += np.dot(J_inv, e_curr)
            e_norm = np.linalg.norm(e_curr)
            i += 1
        
        q_solv = self.adjust_q(q_solv)
        return q_solv
    
def main():
    rclpy.init()
    node = KinematicsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()