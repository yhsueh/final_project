/*
 * Copyright (C) 2017, Yuyu Hsueh.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file turtleCtrl.cpp
 *  @brief The velocity commanding node. It takes the decision made from the 
 *  laserReading node and pass the velocities message to the turtlebot.
 */

#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

geometry_msgs::Twist msg; /**<The geometry msgs that would be sent to the mobile node */

/**
 * Takes the msg from the laserReading node and decides the velocities of the turtlebot.
 */
void motion_callback(const std_msgs::Bool& vel_msg) {

  if (vel_msg.data) {
    msg.linear.x = 0.5;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
  } else {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.5;
  }
}


  ros::NodeHandle n;

  ros::Publisher ctrl_pub = n.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 10);

  ros::Subscriber sub = n.subscribe("base/disp", 10,
                                    motion_callback);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtleCtrl");
  TurtleCtrl turtleCtrlObj;

  ros::Rate loop_rate(5); //5 Htz

  while(ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
}
