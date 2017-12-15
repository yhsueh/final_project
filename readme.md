# Final Project
[![Build Status](https://travis-ci.org/yhsueh/final_project.svg?branch=master)](https://travis-ci.org/yhsueh/final_project)
[![Coverage Status](https://coveralls.io/repos/github/yhsueh/final_project/badge.svg?branch=master)](https://coveralls.io/github/yhsueh/final_project?branch=master)

## Project Descriptioin
The project focus on a portion of the application of the ball collector robot. In a closed environment like cafe, the robot would detect and move toward to the specified ball based on its color. Once the robot is within certain distance to the object, the object would be removed. The final task is to remove all the objects present in the cafe.

## Project Personnel
The author of this ROS package is a second year master student enrolling at the University of Maryland at College Park. This pcakge is the final deliverable for the final project of the class software developement of robotics. 

## Disclaimer
Copyright 2017 YuyuHsueh

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Document Links:
SIP Spreadsheet (Product Backlog/Iteration Backlog/Bug Report):
https://docs.google.com/spreadsheets/d/12HwD9ipKP0mMze7esa8kOW532ijbNlVK7lirzBBcEvI/edit?usp=sharing

Sprint Planning Notes and Review:
https://docs.google.com/document/d/1imp_Num2covjx0czFeErUQy3KCeT4MHiEl-_trNzzP8/edit?usp=sharing

## Youtube Link:
https://www.youtube.com/watch?v=GGgvapKYKoI&t=

## Build procedures:
1. Clone from github by inputting
```
git clone https://github.com/yhsueh/final_project.git
```

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. 
```
cd ~/catkin_ws
catkin_make
```
to build the ROS package. A catkin_ws must be created previously. 

## Procedures for running the turtlebot simulation:
1. Create three terminals:
* First window
```
roslaunch final_package turtle_world.launch
```
or
```
roslaunch final_package turtle_world.launch record_flag:=1
```
To trigger rosbag.
* Second window
```
rosrun final_package turtleCtrller
```
The terminal should show "Initialization".
* Third window
```
roslaunch final_package base.launch view:=t
```
The argument at the end would display the image turtlebot perceived.

NOTE: The nodes must be executed in this order. If the order is reversed, the program will result in unexpected condition.

## Simple test
1. There are multiple tests created to verify this package. 
```
cd ~/catkin_ws
catkin_make run_tests_final_package
```

## Dependencies
1. All the necessary dependencies for this project are listed in the package_xml.

 * catkin
 * genmsg
 * rosunit
 * message_generation
 * roscpp
 * rospy
 * std_msgs
 * sensor_msgs
 * rostest
 * cv_bridge
 * image_transport
 * gazebo_msgs
 * message_runtime

2. It's worth noting that OpenCV is not included here. That is because package cv_bridge would transitively include "libopencv-dev".
 
## Documentation
* The documentation including dependency graphs, files, classes, tests, etc., are generated automatically in HTML file. 
* To access to the documentation, enter the following command:
```
cd ~/catkin_ws/src/final_package/html
xdg-open ./index.html
```