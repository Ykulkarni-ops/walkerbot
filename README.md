# walker_turtlebot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Author
Yash Kulkarni

## Overview
A simple semi-automated control of turtle bot to avoid obstacles using ROS packages. The created walker node utilizes the laser scan callaback to detect the obstacles and based on this data turtlebot is either moves or rotates to avoid the obstacles.

## Dependencies
The package has the following dependencies:

- Ubuntu .04
- ROS Noetic
- catkin_ws
- roscpp
- turtlebot3
- gazebo


## Build method
For the package, you must create and build the catkin workspace as follows in the command line:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
cd src/
```
Now clone the repo into the src folder
```
git clone --recursive https://github.com/Ykulkarni-ops/walkerbot.git
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
```
## Running the walker node
After building the package in a catkin workspace, run the following commands from root workspace(source the workspace if any command fails):
```
In Terminal-1: roscore
In Terminal-2: TURTLEBOT3_MODEL=burger roslaunch walker_turtlebot walker.launch
```

## Rosbag instructions
1) Recording: Use flag rosbagRecorder:=true flag at the end of roslaunch command, to record the data(Make sure you source the workspace):
```
In Terminal-1: roscore
In Terminal-2: TURTLEBOT3_MODEL=burger roslaunch walker_turtlebot walker.launch Recorder:=true

```
2) Playing: To replay the rosbag data collected, run the following command while the listener node is running in another terminal.(Make sure you source the workspace):
```
cd ~/catkin_ws/src/walker_turtlebot/results/
rosbag play recorder.bag 
```

