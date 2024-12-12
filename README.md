
# Centauri
Based-on the work of: https://github.com/andresaraque/centauri6dof.git

Special thanks to @andresaraque, @franciscopedroza030595 and @vromerocano for their contribution to open-source movement in Colombia.

## Home Configuration
In order to play with the robot, it needs to be launched in Gazebo firstly:

    ros2 launch centauri_kinematics sim_kinematics.launch.py

Said launch file contains several nodes and commands to launch the robot in Gazebo, as well to activate a kinematics, a gripper and get-home configuration servers, respectively.

No matter in what configuration is robot itself, it's possible for getting it into the home configuration using the following command:

    ros2 launch centauri_kinematics sim_kinematics.launch.py
  
## Robot Kinematics
For the forward kinematics, the Product of Exponential method was selected due to its capability to deal with *link offsets* easily. On the other hand, for the inverse kinematics, a hybrid numerical method between Newton-Raphson and Descendent Gradiante was implemented, due to the former's *fast convergence* and the latter's *stable solutions*. 

Due to robot kinematics' nature, a *ROS2 service* pipeline was implemented for getting to a desired one-goal position. Also, as this package contains a simulation-only manipulator, the *ros2_control* package was implemented to control each joint's position.

First, the robot needs to be launched in Gazebo:

    ros2 launch centauri_kinematics sim_kinematics.launch.py

Second, the robot needs a request in order to get to a desired goal:

    ros2 run centauri_kinematics kinematics_client.py 0.351 -0.288 0.351

In this case: x = 0.351 ,  y = -0.288 and  z = 0.351. Here are other suggested goals to try:

 - **Goal 1:** 0.161 -0.288 0.547
 - **Goal 2:** 0.351 0.001 0.405

In case the kinematics solution needs to be checked through TF2, use the following command:

    ros2 run tf2_ros tf2_echo base_footprint tool_center    

## Gripper
Just like before, the robot needs to be launched in the Gazebo enviroment:

    ros2 launch centauri_kinematics sim_kinematics.launch.py

To close the gripper for grasping tasks, use the following command:

    ros2 run centauri_description gripper_client.py on
 
However, the same node can used to open the gripper for the same purposes:

    ros2 run centauri_description gripper_client.py off

## Demostration:

Finally, here's a demo of Centauri's current state:

![Centauri-   Gripper](https://github.com/user-attachments/assets/5c7beddb-1ec3-4e95-b682-5813f1459270)

