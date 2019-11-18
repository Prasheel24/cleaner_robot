[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/Prasheel24/cleaner_robot/blob/master/License)

## Authors

**Prasheel Renkuntla** - [GitHub](https://github.com/Prasheel24)

## Overview
Roomba like vacuum cleaner that performs the basic functionality of autnomous navigation with obstacle avoidance.

## Description
An example of obstacle avoidance in ROS for a turtlebot simulated in Gazebo. The world here is turtlebot_world in which if any of the objects are moved and placed in front of the turtlebot, it will move around and start autonomously again. A launch file is made to start the nodes at once. Also, ROSBag is used to examine the output from the topics using an argument in launch file.

## Dependencies	
1. ROS Kinetic - [Installation](http://wiki.ros.org/kinetic/Installation)
2. Catkin(installed by default with ROS) - a low level build system macros and infrastructure for ROS.
3. Gazebo - [Gazebo Installation](http://gazebosim.org/tutorials?tut=ros_installing)
4. Turtlebot - [Turtlebot Package Installation](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation)
5. ROS libraries - roscpp, geometry_msgs, sensor_msgs

## Build
Build using the following commands-

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
cd src/

git clone --recursive https://github.com/prasheel24/cleaner_robot
cd ..
catkin_make
```
This will make the workspace and package ready for execution

## To run the cleaner robot
These are the following steps to run the cleaner_robot.
1. Using launch file from a new terminal-
```
roslaunch cleaner_robot runNodes.launch
```

2. To run the node separately-
```
cd ~/catkin_ws
source ./devel/setup.bash
roscore
```
&nbsp;&nbsp;&nbsp;Ensure if roscore is running in the terminal. For any issues check [ROS Troubleshoot](http://wiki.ros.org/ROS/Troubleshooting)


&nbsp;&nbsp;&nbsp; Then, in another terminal, open the turtlebot_world 
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch 
```

&nbsp;&nbsp;&nbsp; Open a new terminal to run the node-
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun cleaner_robot cleanerBot 
```

## To record bag files
ROSBag recording can be done using the following command-
Open a terminal to run the launch file: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch cleaner_robot runNodes.launch rosbagRecord:=true
```
&nbsp;&nbsp;&nbsp;When the flag(default = false) is true, it will record until SIGINT(ctrl+c) is pressed.

&nbsp;&nbsp;&nbsp;Following are the steps to examine the recorded bag file
1. Open a terminal to setup the Master Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roscore
```

2. Open a new terminal to play the bag file:
&nbsp;&nbsp;&nbsp;Open the turtlebot_world in a terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch 
```
&nbsp;&nbsp;&nbsp;Then in another terminal, run this command
```
cd ~/catkin_ws
source ./devel/setup.bash
cd src/beginner_tutorials/results
rosbag play recording.bag
```
&nbsp;&nbsp;&nbsp;The info on rosbag file can be seen using the following command
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag info recording.bag
```

## References
1. For issues in turtlebot installation, read this page- https://github.com/ROBOTIS-GIT/turtlebot3/issues/304
2. https://en.cppreference.com/w/cpp/language/range-for
3. https://stackoverflow.com/questionas/37621342/cmake-will-not-compile-to-c-11-standard
4. http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
5. http://wiki.ros.org/cmd_vel_mux
