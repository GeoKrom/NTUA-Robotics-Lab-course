# Robotics Lab

Lab Assignments and Semester Project for course DPMS_AS/2205 - Robotics Lab, School of Mechanical Engineering, National Technical University of Athens

## Lab 1

The first robotics lab concists of the kinematic model and manipulator structure of IR2C Robot Manipulator. It also contains and an algorithm for checking materials. 
The algorithm uses point-to-point and line trajectory movements for the robot to execute, in order to validate the type of the material and if is electrically charged.

## Lab 2

The second robotics lab consists of the controller design for Pendubot. In this lab were used three control methods. The first one was a classic PD controller which in the end could not revolve the first joint to the desired angle. 
In order to do that, a second type of controller was used which was a PD controller with graviry copensastion. The controller succsesfully created the necessery control signals so the first joint could stop at the desired angle.

The third and final controller which was used was a nonlinear algorithm which consists of two parts. The first part is the swinging part. The controller sends a strong signal on the first joint in order to oscillate. With the oscillation, the second joint of the mechanism will turn 180 degrees. When the second joint turns from -90 deg to +90 deg, then the second part of the controller activaties. The second part of the controller tries to stabilize the system in an equilibrium point where the system is unstable. In order to achieve that there was used Taylor series on the dynamics of system to create a stabilized linear model and then was used a LQR controller to find the optimal gains for the linear control system.


## Lab 3

## Lab 4 - Semester Project

