# C++ Boilerplate
[![Build Status](https://travis-ci.org/yhsueh/FinalProject.svg?branch=master)](https://travis-ci.org/yhsueh/FinalProject)

## Project Descriptioin
The project focus on a portion of the application of the ball collector robot. In a closed environment like cafe, the robot would detect the balls and move toward them based on the camera images. Once the robot gets to the object, the object would be removed. The final task is to remove all the objects present in the cafe.

## Project Personnel
The author of this ROS package is a second year master student enrolling at the University of Maryland at College Park. This pcakge is the final deliverable for the final project of the class software developement of robotics. 

## Document Links:
SIP Spreadsheet (Product Backlog and Iterations):
https://docs.google.com/spreadsheets/d/12HwD9ipKP0mMze7esa8kOW532ijbNlVK7lirzBBcEvI/edit?usp=sharing

Sprint Planning Notes and Review:
https://docs.google.com/document/d/1imp_Num2covjx0czFeErUQy3KCeT4MHiEl-_trNzzP8/edit?usp=sharing

## Build procedures:
1. Clone from github by inputting
```
git clone https://github.com/yhsueh/final_project.git
```

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Input 
```
catkin_make
```
to build the ROS package.

## Procedures for running the turtlebot simulation:
1. Run roslaunch file to start gazebo, and two necessary nodes.
```
roslaunch final_package turtlebot_world.launch
```

## Simple test
1. This test is just for verifying that the unit testing is configured correctly.
```
cd <catkin_ws>
catkin_make run_tests_final_package_gtest_base_test
```