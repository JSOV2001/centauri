# Centauri Trajectories

## Test for trajectories
Commands to compile and source packages:
    colcon build --packages-select centauri_description centauri_interfaces centauri_kinematics centauri_trajectories
    source ./install/setup.bash

Command to launch the nodes:

    ros2 launch centauri_trajectories test_trajectory.launch.py

Command to run the test code:

    cd ros2_ws/src/centauri/centauri_trajectories/tests/
    python -m pytest -s test_trajectory.py

Command to locate the tool center point with respect to the base:

    ros2 run tf2_ros tf2_echo base_link tool_center
