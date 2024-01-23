# Carleton Mail Delivery Robot System

Build Status:  ![Build Status](https://github.com/bardia-p/carleton-mail-delivery-robot/actions/workflows/ros_colcon_test.yml/badge.svg)

### Group Members: 
- [Max Curkovic](https://github.com/maxcurkovic)
- [Bardia Parmoun](https://github.com/bardia-p)
- [Matt Reid](https://github.com/MattReid6767)
- [Cassidy Pacada](https://github.com/cassidypacada)

## Description:

This project is being completed to fulfill the 2023 - 2024 Capstone project requirements for SYSC 4907 at Carleton University.

The Carleton Mail Delivery Robot system aims to automate Carleton's existing mail deliver system. With the placement of several bluetoopth beacons throughout the tunnels, the robot will be able to navigate across Carleton using a LiDAR sensor.
For the purpose of this project, the robot's navigation system should allow it to reach its intended destination while detecting and avoiding any obstacles. The robot must be resilient and should be able to find its destination in the event
that it strays off its original course.
The system will be controlled through a web application component which will allow users to give the robot a desired destination.

<div align='center'>
  <img src='https://github.com/bardia-p/carleton-mail-delivery-robot/assets/140274454/b95c08f0-48f5-4438-bfb1-273eea98c4e5'>
  <p>iRobot Create 2 version of the Carleton Mail Delivery Robot System</p>
</div>

## Project Components:

This project involves both software and hardware components. 

The hardware build consists of:
* an iRobot Create 2 (soon to be upgraded to the iRobot Create 3)
* a 3D printed mailbox chassis to hold external components and mail
* a power bank
* a Raspberry Pi 4B to controls the robot's actions
* a LiDAR to provide environmental data for navigation

The software consists of:
* ROS 2 (Foxy) which is a set of libraries used to communicate with the robot
* the AutonomyLab create_robot repository which allows us to program the robot using Python
* Slamtec Lidar Ros2 package to allow the robot to communicate with the LiDAR

## Running the System:
1. Ensure that you have an Ubuntu environment setup with [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) and the [create_robot](https://github.com/AutonomyLab/create_robot/tree/foxy) repository installed.
2. Clone this repository into the create_ws folder created during the create_robot installation. 
3. Clone the [Slamtec Lidar Ros2](https://github.com/Slamtec/sllidar_ros2) repository into the same directory.
4. In your terminal, run the commands ```source /opt/ros/foxy/setup.bash``` and ```source ~/create_ws/install/setup.bash```
5. Run the command ```source ~/create_ws/install/setup.bash``` and the system should start, indicated by the appearance of logs.





