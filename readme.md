# C++ Boilerplate
[![Build Status](https://travis-ci.org/yhsueh/Midterm.svg?branch=master)](https://travis-ci.org/yhsueh/Midterm)

## Project Descriptioin
The project focus on a portion of the application of the ball collector robot. In a closed environment like cafe, the robot would detect a few balls and move toward them based on the camera images.  

## Project Personnel
The author of this ROS package is a second year master student enrolling at the University of Maryland at College Park. This pcakge is the final deliverable for the final project of the class software developement of robotics. 

## Document Links:
SIP Spreadsheet (Product Backlog and Iterations):
https://docs.google.com/spreadsheets/d/12HwD9ipKP0mMze7esa8kOW532ijbNlVK7lirzBBcEvI/edit?usp=sharing

Sprint Planning Notes and Review:
https://docs.google.com/document/d/1imp_Num2covjx0czFeErUQy3KCeT4MHiEl-_trNzzP8/edit?usp=sharing

## Build procedures:
1. Clone turtlebot_walker branch from github by inputting
```
git clone -b turtlebot_walker https://github.com/yhsueh/turtlebot_walker.git
```

2. Move beginner_tutorials into src folder in your catkin_ws.

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Input 
```
catkin_make
```
to build the ROS package.

## Procedures for running the turtlebot simulation:
1. Run roslaunch file to start gazebo, and two necessary nodes.
```
roslaunch turtlebot_walker turtlebot_world.launch
```
The simulation can be recorded if record_flag is set to 1;
```
roslaunch turtlebot_walker turtlebot_world.launch record_flag:=1
```

2. Switch back to gazebo and view the simulation.

3. See what topics are recorded with rosbag info and rosbag play. Change directory to results folder. Before running rosbag play, make sure roscore is up and running.
```
roscore
cd {"turtlebot_walker"}/results
rosbag info bagfile.bag
rosbag play bagfile.bag
```