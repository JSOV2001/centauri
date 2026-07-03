#!/usr/bin/env python3
import pytest
import rclpy
from scripts.trajectory_server import TrajectoryServer
from rclpy.executors import SingleThreadedExecutor
import threading
from scripts.trajectory_client import TrajectoryClient
from centauri_interfaces.action import TrajectoryGoal
from rclpy.action.client import ClientGoalHandle
import numpy as np

def get_goal_feedback(feedback_future_msg):
    feedback_msg = feedback_future_msg.feedback
    q_curr = feedback_msg.q_curr
    # print(f"\nFeedback: {q_curr}")

def compute_goal_result(result_future_msg):
    # Get result from the goal
    result_msg = result_future_msg.result().result
    q_final = result_msg.q_final
    assert q_final == np.array([1.07, 0.68, 0.99, -1.57, 0.77, 1.56])
    print(f"\nResult: {q_final}")

    # Close ROS2 communication
    server_node.destroy_node()
    client_node.destroy_node()
    server_executor.shutdown()
    client_executor.shutdown()
    rclpy.shutdown()

def get_goal_status(status_future_msg):
        # ========== Testing. Phase 3: Assert ==========
        # Assert whether goal was accepted or not
        goal_handle: ClientGoalHandle = status_future_msg.result()
        if goal_handle.accepted:
            assert goal_handle.accepted
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(compute_goal_result)
            print("\nGoal was accepted")
        else:
            assert not goal_handle.accepted
            print("\nGoal is invalid")

# ========== Testing. Phase 1: Arrange ==========
@pytest.fixture
def rclpy_init():
    # Open ROS2 communication
    rclpy.init()

def spin_executor(executor):
    executor.spin()

@pytest.fixture
def action_service():
    # Call client from trajectory service
    server_node = TrajectoryServer()

    # Initialize a parallel thread to execute server
    server_executor = SingleThreadedExecutor()
    server_executor.add_node(server_node)
    server_thread = threading.Thread(target= spin_executor, args=(server_executor,))
    
    # Execute server in a parallel thread
    server_thread.start()
    
    # Call client from trajectory service
    client_node = TrajectoryClient()

    # Initialize a parallel thread to execute client
    client_executor = SingleThreadedExecutor()
    client_executor.add_node(client_node)
    client_thread = threading.Thread(target= spin_executor, args=(client_executor,))
    
    # Execute client in a parallel thread
    client_thread.start()

    return server_node, server_executor, client_node, client_executor

def test_service_response(rclpy_init, action_service):
    global server_node, server_executor, client_node, client_executor
    server_node, server_executor, client_node, client_executor = action_service

    # Wait for an available server, and
    # assert if action service is available
    is_trajectory_server_ready = client_node.trajectory_client.wait_for_server(timeout_sec= 2)
    assert is_trajectory_server_ready
    print(f"\nTrajectory action server-client is ready")
    
    # ========== Testing. Phase 2: Act ==========
    # Initiaize goal
    des_goal = np.array([0.351, -0.288, 0.351], dtype= np.float64)
    global goal_msg
    goal_msg = TrajectoryGoal.Goal()
    goal_msg.x = des_goal[0]
    goal_msg.y = des_goal[1]
    goal_msg.z = des_goal[2]
    goal_msg.time_range = 1.0
    
    # Request response from server
    print(f"\nGet to {des_goal} m in {goal_msg.time_range} seconds")
    status_future = client_node.trajectory_client.send_goal_async(
        goal_msg,
        feedback_callback= get_goal_feedback
    )
    status_future.add_done_callback(get_goal_status)