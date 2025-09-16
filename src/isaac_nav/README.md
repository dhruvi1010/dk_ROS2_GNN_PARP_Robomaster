# Robomaster Navigation RoNa

In this repository, you will find the necessary packages to use navigation on the Robomaster using waypoints via MQTT and the robot follower. 
Follow the instructions to install all the required libraries and packages.

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)


## Introduction

This project is based on ROS2 Humble. It includes a simple navigation stack that receives waypoints through MQTT messages and the position using the vicon system . The package for controlling the robot can be found in the "robomaster_setup" package.


## Installation

First we need to install all the dependencies:
```
pip install paho-mqtt
pip install transforms3d
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## Usage
the user computer Need to use  cycloneDDs and same Domain// FastDDs RMW_IMPLEMENTATIONcould be better 
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

and same domain  
```
export ROS_DOMAIN_ID=130
```
SSH on the robot
the current IP of the robots conected to the FLW_CPS:\
robot_2:192.168.2.127

robot_3:192.168.2.80  

robot_5:192.168.2.161 \
or you can use the following command   
```
ssh robot_3@ep03.local (e.g. for robomaster 3) 
password: robomaster
```
First enable the vicon bridge using the NUC sensorfloor with IP 192.168.2.186

 (after ssh on the robot)

The robots have a service active that include the waypoint follower that enable the navigation and the vicon pose once the robot boot is done. To check the status or stop the service use the following command:

```
sudo systemctl status robomaster_waypoint.service
sudo systemctl kill robomaster_waypoint.service
sudo systemctl start ...
```
If you want to use the navigation with out the vicon and the mqtt bridge then use the robomaster_start.service 

The following command launch the bringup for the waypoint follower using the mqtt and the position of the robot via Vicon 

```
ros2 launch rona_navigation waypoint_bringup_launch.py
```
If you wan to use the follower code use the launcher on the follower robot (working on ep05 and ep03)
```
ros2 launch rona_navigation follower_bringup_launch.py
```
and In the followed robot use this command 
```
ros2 run rona_navigation base_footprint_pose_publisher
```
 To use the navigation code inside of the robot without the mqtt receiver 

```
ros2 launch rona_navigation robomaster_nav_launch.py
```

to see the rviz from the user computer use the next command:
```
ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=ep02 use_namespace:=true rviz_config:="/wk_name/src/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz"
```
## Issues

1. **Tunning of the parameters of the controller***
The parameters using the mqtt is almost 1.3m/s
Using the DWB controller the max velocity is 2.8 m/s
2. **The Navigation throught poses is not working**
Install the navigation2 packages directly from the git  to modify the remove past goals
3. **The commuincation of the ROS Nodes is not working withall the robots**
When connect the nodes between different nodes from a device to another, the comunication get overloaded 

