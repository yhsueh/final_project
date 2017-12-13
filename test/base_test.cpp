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

/** @file base_test.cpp
 *	@brief This node takes and analyze the range data. Subsequently, pass the
 *	the decision made based on the data to the turtleCtrl node which 
 *	manipulates the turtlebot.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "final_package/StatusCheck.h"
#include "final_package/ColorChange.h"
#include "std_msgs/Int64.h"
#include "ImageProcess.hpp"

float min_dist;
int displacement, color;

void dispCallback(const std_msgs::Int64 &msg) {
  displacement = msg.data;
}

bool colorCallback(final_package::ColorChange::Request &req,
                   final_package::ColorChange::Response &resp) {
  color = req.input;
  return true;
}

TEST(IntegrationTest, TskT7_velocity_command_red) {
  ros::NodeHandle nh("~");
  ros::NodeHandle nh2;
  std::string test_dir,imgDir;
  nh.getParam("test_dir", test_dir);

  ros::Subscriber dispSub = nh2.subscribe("base/disp",1, dispCallback);
  image_transport::ImageTransport it(nh2);
  image_transport::Publisher imgPub = it.advertise("camera/rgb/image_raw", 10);

  imgDir = test_dir + "/Redball.jpg";
  cv::Mat lImage = cv::imread(imgDir);
  int count = 0;
  ros::Rate loop_rate(5);
  while (count < 20) {
    sensor_msgs::ImagePtr imMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lImage).toImageMsg();
    imgPub.publish(imMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  EXPECT_NE(displacement,0);
}

TEST(IntegrationTest, TskT7_velocity_command_void) {
  ros::NodeHandle nh("~");
  ros::NodeHandle nh2;
  std::string test_dir,imgDir;
  nh.getParam("test_dir", test_dir);
  ros::Subscriber dispSub = nh2.subscribe("base/disp",1, dispCallback);
  image_transport::ImageTransport it(nh2);
  image_transport::Publisher imgPub = it.advertise("camera/rgb/image_raw", 10);

  imgDir = test_dir + "/void.png";
  cv::Mat lImage = cv::imread(imgDir);
  int count = 0;
  ros::Rate loop_rate(5);
  while (count < 20) {
    sensor_msgs::ImagePtr imMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lImage).toImageMsg();
    imgPub.publish(imMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  EXPECT_EQ(displacement,10000);
}

TEST(IntegrationTest, TskT6_change_color) {
  ros::NodeHandle nh2;
  ros::Subscriber dispSub = nh2.subscribe("base/disp",1, dispCallback);
  ros::ServiceClient status = nh2.serviceClient<final_package::StatusCheck>("status_check");
  ros::ServiceServer colorServ = nh2.advertiseService("base_color_change",colorCallback);

  final_package::StatusCheck srv;
  srv.request.input = true;
  status.call(srv);

  int count = 0;
  ros::Rate loop_rate(5);
  while (count < 20) {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  EXPECT_EQ(color,2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

