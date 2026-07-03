# Centauri Kinematics

## Test for kinematics
Commands to compile and source packages:
    colcon build --packages-select centauri_description centauri_interfaces centauri_kinematics centauri_trajectories
    source ./install/setup.bash

Command to launch the nodes:

    ros2 launch centauri_trajectories test_trajectory.launch.py

Commands to run the test code:

    cd ros2_ws/src/centauri/centauri_kinematics/tests/
    python -m pytest -s test_kinematics.py

Command to locate the tool center point with respect to the base:

    ros2 run tf2_ros tf2_echo base_link tool_center