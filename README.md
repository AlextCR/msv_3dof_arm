# msv_3dof_arm

3 DOF robotic arm using actuators based in brushless motors and the Moteus controller.

![Alt text](Images/C5_final_Assem1.png?raw=true "msv_3dof_arm")


- Plug a generic ps2 style joystick (joy package is needed) and run teleop_final.launch for a complete simulation of the robot
- Run the moteus_com_node.py node (Python 3.7) to communicate with the moteus controller using the same data as Gazebo

The actual robot works and has only been tested in Ubuntu 18.04 and Xubuntu 18.04 (Raspberry Pi 3b) with ROS Melodic.

Moteus controller general info: https://github.com/mjbots/moteus

To make ROS Melodic compatible with Python 3.7: https://www.dhanoopbhaskar.com/blog/2020-05-07-working-with-python-3-in-ros-kinetic-or-melodic/ (Do it at your own risk!)

Trajectory tracking test : https://youtu.be/6b1fZ_Ul1Zo

Teleop test: https://youtu.be/edxL0ZtZc2E
