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

/** @file talker_test.cpp
 *	@brief Integration test on service/client nodes.
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "BaseTestCallback.hpp"
#include <iostream>
std::shared_ptr<ros::NodeHandle> nh;

float min_dist;

/** 
 * Testing case confirming that the messages passed between service and 
 * client are same.
 */



void distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
	min_dist = msg->data;
}


/*
TEST(TESTSuite, test_setup) {
	int i = 1;
  	EXPECT_EQ(i,1);
}
*/

TEST(integrationTest, range_call_back) {
	BaseTestCallback BaseObj;
	sensor_msgs::LaserScan msg;
	
	msg.ranges.resize(3);
	msg.ranges[0] = 1.0;
	msg.ranges[1] = 2.0;
	msg.ranges[2] = 3.0;
	
	ros::Publisher pub = nh->advertise<sensor_msgs::LaserScan>("scan",100);
	//ros::Subscriber sub = nh->subscribe("base/min_distance", 100, &BaseTestCallback::distanceCallback, &BaseObj);
	ros::Subscriber sub = nh->subscribe("base/min_distance", 100, distanceCallback);
	pub.publish(msg);

	ros::spinOnce();
	//EXPECT_EQ(2,BaseObj.minimum);
	EXPECT_EQ(2,min_dist);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

