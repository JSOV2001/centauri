# Centauri

![image](https://github.com/user-attachments/assets/29b65d70-324c-4231-9c77-fcaac0e6c7d1)

First, the robot need to be launched in RViz:

    ros2 launch centauri_kinematics dis_kinematics.launch.py

Second, the robot needs a request in order to get to a goal:

    ros2 run centauri_kinematics kinematics_client.py 0.351 -0.288 0.351

Where 0.351 is the desired x, -0.288 the desired y and 0.351 the desired z.

Here are other suggested goals to try:

 - **Goal 1:** 0.161 -0.288 0.547
 - **Goal 2:** 0.351 0.001 0.405

If it's necessary to check the kinematics solution through TF2, it can be done through the following command:

    ros2 run tf2_ros tf2_echo link0 link6

![Centauri](https://github.com/user-attachments/assets/6e597bb3-bcee-4de0-9a1c-046138014e7c)
