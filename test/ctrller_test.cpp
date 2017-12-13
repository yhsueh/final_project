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

/** @file ctrller_test.cpp
 *	@brief This node takes and analyze the range data. Subsequently, pass the
 *	the decision made based on the data to the turtleCtrl node which 
 *	manipulates the turtlebot.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "final_package/ColorChange.h"
#include "gazebo_msgs/DeleteModel.h"
#include "CtrllerTest.hpp"

std::shared_ptr<ros::NodeHandle> nh;
float angularZ, linearX;

void velCallback(const geometry_msgs::Twist &msg) {
  linearX = msg.linear.x;
  angularZ = msg.angular.z;
}

TEST(integrationTest, TskT1_advance) {
  sensor_msgs::LaserScan laserMsg;
  std_msgs::Int64 dispMsg;
  float expX = 0.2;

  ros::Publisher rngPub = nh->advertise<sensor_msgs::LaserScan>("scan",10);
  ros::Publisher dispPub = nh->advertise<std_msgs::Int64 >("base/disp",10);
  ros::Subscriber velSub = nh->subscribe("/mobile_base/commands/velocity",1,velCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (count < 20) {
    laserMsg.ranges.resize(1);
    laserMsg.ranges[0] = 1.0;
    rngPub.publish(laserMsg);

    dispMsg.data = 10;
    dispPub.publish(dispMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  EXPECT_EQ(0, angularZ);
  EXPECT_EQ(expX, linearX);
}

TEST(integrationTest, TskT1_turn) {
  sensor_msgs::LaserScan laserMsg;
  std_msgs::Int64 dispMsg;

  ros::Publisher rngPub = nh->advertise<sensor_msgs::LaserScan>("scan",10);
  ros::Publisher dispPub = nh->advertise<std_msgs::Int64>("base/disp",10);
  ros::Subscriber velSub = nh->subscribe("/mobile_base/commands/velocity",1,velCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (count < 20) {
    laserMsg.ranges.resize(1);
    laserMsg.ranges[0] = 0.0;
    rngPub.publish(laserMsg);

    dispMsg.data = 10000;
    dispPub.publish(dispMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  EXPECT_EQ(0.5, angularZ);
  EXPECT_EQ(0, linearX);
}

TEST(integrationTest, TskT3_color_removal) {
  CtrllerTest ctrllerObj;
  sensor_msgs::LaserScan laserMsg;
  std_msgs::Int64 dispMsg;
  final_package::ColorChange srv;
  int count = 0;
  ros::Publisher rngPub = nh->advertise<sensor_msgs::LaserScan>("scan",10);
  ros::Publisher dispPub = nh->advertise<std_msgs::Int64>("base/disp",10);
  ros::Subscriber velSub = nh->subscribe("/mobile_base/commands/velocity",1,velCallback);
  ros::ServiceClient colorChangeClient = nh->serviceClient<final_package::ColorChange>("base_color_change");
  ros::ServiceServer deleteServer = nh->advertiseService("/gazebo/delete_model",
  															&CtrllerTest::deleteCallback,
  															&ctrllerObj);

  srv.request.input = 2;
  colorChangeClient.call(srv);
  ros::Rate loop_rate(10);
  while (count < 20) {
    if (ctrllerObj.deleteFlag) {
      break;
    }

    laserMsg.ranges.resize(1);
    laserMsg.ranges[0] = 0.1;
    rngPub.publish(laserMsg);

    dispMsg.data = 10;
    dispPub.publish(dispMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  EXPECT_EQ("GreenBall", ctrllerObj.deleteColor);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ctrller_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

