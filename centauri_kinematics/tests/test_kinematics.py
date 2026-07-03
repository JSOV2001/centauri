#!/usr/bin/env python3
import pytest
import rclpy
from scripts.kinematics_server import KinematicsServer
from rclpy.executors import SingleThreadedExecutor
import threading
from scripts.kinematics_client import KinematicsClient

# ========== Testing. Phase 1: Arrange ==========
@pytest.fixture
def rclpy_init():
    # Open ROS2 comunication
    rclpy.init()

def spin_executor(executor):
    executor.spin()

@pytest.fixture
def server():
    # Call server for Kinematics service
    server_node = KinematicsServer()
    server_executor = SingleThreadedExecutor()
    server_executor.add_node(server_node)
    server_thread = threading.Thread(target= spin_executor, args=(server_executor,))
    return server_node, server_thread, server_executor

def test_service_response(rclpy_init, server):
    # Execute Kinematics service server in a parallel thread
    server_node, server_thread, server_executor = server
    server_thread.start()

    # Call Kinematics service client
    client_node = KinematicsClient()
    
    try:
        # ========== Testing. Phase 2: Act ==========
        
        # Request Centauri to reach certain goal
        # Send desired goal to server
        goal = [0.351, -0.288, 0.351]
        response_future = client_node.send_request(goal)
        rclpy.spin_until_future_complete(client_node, response_future, timeout_sec= 2)
        response_msg = response_future.result()
        
        # Get end-effector position from FKine algorithm
        fkine = server_node.compute_fkine(response_msg.q_list)
        fkine_x = fkine[0][3]
        fkine_y = fkine[1][3]
        fkine_z = fkine[2][3]
        print(f"\nFKine: {fkine_x}, {fkine_y}, {fkine_z}")

        # Get end-effector position from Gazebo's TFs
        tf_curr = server_node.get_tf()
        tf_curr_x = tf_curr.transform.translation.x
        tf_curr_y = tf_curr.transform.translation.y
        tf_curr_z = tf_curr.transform.translation.z
        print(f"\nTF: {tf_curr_x}, {tf_curr_y}, {tf_curr_z}")
        
        # ========== Testing. Phase 3: Assert ==========
        # Assert each node has the expected name
        assert client_node.get_name() == 'kinematics_client'
        assert server_node.get_name() == 'kinematics_server'

        # Assert each node has the proper attributes
        assert hasattr(client_node, 'kinematics_client')
        assert hasattr(server_node, 'kinematics_server')

        # Assert that both nodes share the same service and messsage
        assert server_node.kinematics_server.srv_name == client_node.kinematics_client.srv_name
        assert server_node.kinematics_server.srv_type == client_node.kinematics_client.srv_type

        # Assert the end-effector's x-position
        position_error_threshold = 5/1000 # In meters
        assert float(abs(tf_curr_x - fkine_x)) <= position_error_threshold
        
        # Assert the end-effector's y-position
        assert float(abs(tf_curr_y - fkine_y)) <= position_error_threshold
        
        # Assert the end-effector's z-position
        assert float(abs(tf_curr_z - fkine_z)) <= position_error_threshold

        # Assert the ikine algorithms iterations
        max_iters = 100
        assert response_msg.iters < max_iters
    finally:
        # Close ROS2 comunication
        server_node.destroy_node()
        client_node.destroy_node()
        server_executor.shutdown()
        rclpy.shutdown()