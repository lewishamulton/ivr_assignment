# ivr_assignment
This is the final assignment for my Introduction to Vision and Robotics Course. 

This assignment involves using both OpenCV and ROS Noetic in order to estimates the joint angles of a 4 link robot moving in a sinusoidal trajectory as well as determining the location of a target cube orbiting the robot. There is also some additional Python Scripts for determining the theoretical aspects of the Jacobian and Forward Kinematics of the robot. These use input angles to determine the final end-effector position (the last link) of the robot in terms of x,y coordinates relative to the base of said robot. 

This assignment requires Ubuntu OS in order to download and run ROS Noetic. 

How to Run Scripts: 
 For getting the estimated joint angles in Part 2 we first ran the spawn.launch then image2.py in a seperate terminal then image1.py in another seperate terminal 
 
 For getting the chamfer matching in second part of Part 2 we ran spawn.launch then target_move.py in a seperate terminal, the chamfer matching template,  template1.jpg was in the catkin_ws folder and can either be downloaded as part of the Github repo or generated via a function and some commented out code in the target_move.py folder 
 
  For getting the calculated Forward Kinematics and estimated end position of end effector in part 3 we ran spawn.launch in one terminal then kinematics.py in a seperate terminal 
