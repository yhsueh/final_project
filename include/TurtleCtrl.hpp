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

/** @file TurtleCtrl.hpp
 *	@brief The turtlectrller node has been implemented as a class. It has 
 *  multiple services and publisher/subscribers in order to drive the 
 *  turtlebot toward the ball objects. 
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#pragma once
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "final_package/ColorChange.h"

class TurtleCtrl {
 public:
  TurtleCtrl(); /**< Constrcutor that initialize pubs/subs/servers/clients */

  /**
   * This is a subscriber callback of the displacment message used to find the 
   * turtlebot's position in relation to the ball.
   */
  void dispCallback(const std_msgs::Int64& dispMsg);

  /**
   * This is a subscriber callback of the range measurement. This information is
   * critical for determining whether the ball is within the turtlebot's reach.
   */
  void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * This is a server callback of the color information. The specified color is 
   * necessary for the program to delete the correct object.
   */
  bool colorCallback(final_package::ColorChange::Request &req,
                     final_package::ColorChange::Response &resp);
  ros::ServiceClient statusCheckCli_;
  bool terminate;

private:
  ros::NodeHandle nh;
  ros::Subscriber rngSub;
  ros::Publisher velPub;
  ros::Subscriber dispSub;
  ros::ServiceClient deleteClient;
  ros::ServiceServer colorChangeSrv_;
  float velMax;
  float kp;
  float kd;
  float lMinimal;
  int color;
  int disp;
  int lDisp;  // for derivative control term
};

