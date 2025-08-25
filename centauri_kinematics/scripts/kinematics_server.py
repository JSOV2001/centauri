#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from centauri_interfaces.srv import KinematicGoal
from std_msgs.msg import Float64MultiArray
# Python libraries
import numpy as np
from numpy import pi
from numpy.linalg import pinv

class KinematicsServer(Node):
    def __init__(self):
        super().__init__("kinematics_server")

        # Each joint's screw-axis point
        p1 = np.array([0.000,  0.000, 0.000])
        p2 = np.array([0.000,  0.000, 0.211])
        p3 = np.array([0.003, -0.001, 0.433])
        p4 = np.array([0.003, -0.001, 0.560])
        p5 = np.array([0.002, -0.001, 0.655])
        p6 = np.array([0.002, -0.002, 0.696])

        # Each joint's screw-axis unit vector
        ω1 = np.array([0, 0, 1])
        ω2 = np.array([1, 0, 0])
        ω3 = np.array([1, 0, 0])
        ω4 = np.array([0, 0, 1])
        ω5 = np.array([1, 0, 0])
        ω6 = np.array([0, 0, 1])
        
        # Each joint's screw-axis
        S1 = self.get_screw_axis(p1, ω1, 0)
        S2 = self.get_screw_axis(p2, ω2, 0)
        S3 = self.get_screw_axis(p3, ω3, 0)
        S4 = self.get_screw_axis(p4, ω4, 0)
        S5 = self.get_screw_axis(p5, ω5, 0)
        S6 = self.get_screw_axis(p6, ω6, 0)

        # Joint group's screw-axis list
        self.S_list = np.array([S1, S2, S3, S4, S5, S6]).T

        self.home = np.array([
            [1.000, -0.002, -0.000,  0.002],
            [0.002,  1.000, -0.018, -0.004],
            [0.000,  0.018,  1.000,  0.776],
            [0.000,  0.000,  0.000,  1.000]
        ])

        self.q_list = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
        self.q_prev = self.q_list.copy().astype(float)

        # Inverse kinematics' parameters
        self.max_iters = 100
        self.partial_iters = int(0.6 * self.max_iters)
        self.e_max = 0.1/100
        self.kinematics_srv = self.create_service(KinematicGoal, "/kinematics", self.kinematics_callback)
        print("Initializing kinematics server")
    
    # Get client's desired goal
    def kinematics_callback(self, request, response):
        goal = np.array([request.x, request.y, request.z])
        print(f"\nPrevious: {np.around(self.q_list, 2)}")

        # Compute inverse kinematics for goal
        q_solv, e_norm, total_iters, success = self.compute_ikine(goal, self.q_list)
        self.q_list = q_solv
        
        # Define server's response to client
        response.q_list = q_solv.tolist() # Must be a standard list instead of numpy-type
        response.error = e_norm
        response.iters = total_iters
        response.success = success

        # Send server's response to client
        return response
    
    # Compute forward kinematics numerically 
    def compute_fkine(self, theta_list):
        fkine = self.home # End-effector's home configuration
        s_list = self.S_list # Joint group's screw axes
        theta_length = len(theta_list)

        # Apply product of exponentials
        for i in range(theta_length - 1, -1, -1):
            se3 = self.convert_vec_to_se3(s_list[:, i] * theta_list[i])
            matrix_exp = self.compute_se3_matrix_exp(se3)
            fkine = np.dot(matrix_exp, fkine)
        
        return fkine
    
    # Compute jacobian numerically
    def compute_jacobian(self, q_list):
        delta = 1/1000
        q_length = len(q_list)
        J_shape = (3, q_length)
        J_curr = np.zeros(J_shape)

        for k in range(q_length):
            q_plus = np.copy(q_list)
            q_plus[k] += delta

            fkine_mat = self.compute_fkine(q_list)
            fkine_xyz = fkine_mat[0:3,3]
            
            fkine_plus_mat = self.compute_fkine(q_plus)
            fkine_plus_xyz = fkine_plus_mat[0:3,3]
            
            J_curr[:, k] = (1/delta) * (fkine_plus_xyz - fkine_xyz)
        
        return J_curr
    
    # Compute inverse kinematics numerically
    def compute_ikine(self, goal, q_guess):
        e_curr = 0
        e_norm = 1.0
        iters = 0
        q_solv = q_guess
        success = False

        while e_norm > self.e_max and iters < self.max_iters:
            J_curr = self.compute_jacobian(q_solv)
            fkine_curr = self.compute_fkine(q_solv)
            xyz_curr = fkine_curr[0:3,3]
            
            # Compute new q-values
            e_curr = goal - xyz_curr

            if iters < self.partial_iters:
                # Apply Newton-Raphson method
                J_inv = pinv(J_curr)
                q_solv += np.dot(J_inv, e_curr)
            else:
                # Apply Gradient method
                J_tran = J_curr.T
                damp = 0.5
                q_solv += damp * np.dot(J_tran, e_curr)
            
            q_solv = np.clip(q_solv, -pi/2, pi/2)
            e_norm = np.linalg.norm(e_curr)
            iters += 1
        
        if iters == self.max_iters:
            success = False
        else:
            success = True
        
        return q_solv, e_norm, iters, success

    # Get a normalized screw axis from a parametric description
    def get_screw_axis(self, q, s, h):
        screw_axis = np.r_[
            s, 
            np.cross(q, s) + np.dot(h, s)
        ] 
        return screw_axis
    
    # Normalizes a vector
    def normalize_vector(self, v):
        v_norm = np.linalg.norm(v)
        v_unit = v / v_norm
        return v_unit
    
    # Converts a vector to SO3 matrix
    def convert_vec_to_so3(self, omg):
        so3 = np.array([
            [0,       -omg[2],  omg[1]],
            [ omg[2],       0, -omg[0]],
            [-omg[1],  omg[0],       0]
        ])
        return so3

    # Converts SO3 matrix to a vector 
    def convert_so3_to_vec(self, so3_mat):
        omg = np.array([so3_mat[2][1], so3_mat[0][2], so3_mat[1][0]])
        return omg

    # Converts a vector of exponential coordinates for rotation into axis-angle form
    def convert_expc3_to_axis_ang(self, expc3):
        omg_hat = self.normalize_vector(expc3)
        theta = np.linalg.norm(expc3)
        axis_ang = (omg_hat, theta)
        return axis_ang

    # Computes the matrix exponential of a SO3 matrix 
    def compute_so3_matrix_exp(self, so3_mat):
        omgtheta = self.convert_so3_to_vec(so3_mat)
        omgtheta_norm = np.linalg.norm(omgtheta)
        if (abs(omgtheta_norm) < 1e-6):
            return np.eye(3)
        else:
            theta = self.convert_expc3_to_axis_ang(omgtheta)[1]
            omg_mat = so3_mat / theta
            matrix_exp = np.eye(3) + np.sin(theta) * omg_mat + (1 - np.cos(theta)) * np.dot(omg_mat, omg_mat)
        return matrix_exp

    # Converts a spatial velocity vector into a SE3 matrix
    def convert_vec_to_se3(self, v):
        se3 = np.r_[
            np.c_[
                self.convert_vec_to_so3([v[0], v[1], v[2]]), 
                [v[3], v[4], v[5]]
            ], 
            np.zeros((1, 4))
        ]
        return se3

    # Computes the matrix exponential of an SE3 matrix
    def compute_se3_matrix_exp(self, se3_mat):
        se3_mat = np.array(se3_mat)
        omgtheta = self.convert_so3_to_vec(se3_mat[0: 3, 0: 3])
        omgtheta_norm = np.linalg.norm(omgtheta)
        if (abs(omgtheta_norm) < 1e-6):
            matrix_exp = np.r_[np.c_[np.eye(3), se3_mat[0: 3, 3]], [[0, 0, 0, 1]]]
            return matrix_exp
        else:
            theta = self.convert_expc3_to_axis_ang(omgtheta)[1]
            omgmat = se3_mat[0: 3, 0: 3] / theta
            matrix_exp = np.r_[
                np.c_[
                    self.compute_so3_matrix_exp(se3_mat[0: 3, 0: 3]),
                    np.dot(np.eye(3) * theta + (1 - np.cos(theta)) * omgmat + (theta - np.sin(theta)) * np.dot(omgmat,omgmat), 
                    se3_mat[0: 3, 3]) / theta
                ],
                [[0, 0, 0, 1]]
            ]
            return matrix_exp

    # Computes the adjoint representation of an SE3 matrix
    def compute_adj_rep(self, se3_mat):
        # Rotation matrix
        rot_mat  = se3_mat[0:3, 0:3]
        rot_shape = rot_mat.shape

        # Translation vector
        t_vec = se3_mat[0:3, 3]
        t_so3 = self.convert_vec_to_so3(t_vec)

        adj_rep = np.r_[
            np.c_[rot_mat, np.zeros(rot_shape)], 
            np.c_[np.dot(t_so3, rot_mat), rot_mat]
        ]

        return adj_rep

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