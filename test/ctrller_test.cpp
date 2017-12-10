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

/** @file base.cpp
 *	@brief This node takes and analyze the range data. Subsequently, pass the
 *	the decision made based on the data to the turtleCtrl node which 
 *	manipulates the turtlebot.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "final_package/ColorChange.h"
/*
#include "ImageProcess.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
*/

std::shared_ptr<ros::NodeHandle> nh;
float angularZ, linearX;

/** 
 * Testing case confirming that the messages passed between service and 
 * client are same.
 */



void velCallback(const geometry_msgs::Twist &msg ) {
	linearX = msg.linear.x;
	angularZ = msg.angular.z;
}


/**
  * Sending fake laserscan measurment and subscribe to the base node and 
  * see if the result is expected.
  */


TEST(integrationTest, control_test_advance) {
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


TEST(integrationTest, control_test_turn) {
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

/*
TEST(integrationTest, control_test_delete_model) {
	sensor_msgs::LaserScan laserMsg;
	std_msgs::Float32 dispMsg;
	
	ros::Publisher rngPub = nh->advertise<sensor_msgs::LaserScan>("scan",10);
	ros::Publisher dispPub = nh->advertise<std_msgs::Int64>("base/disp",10);
	ros::Subscriber velSub = nh->subscribe("/mobile_base/commands/velocity",1,velCallback);
	
	ros::Rate loop_rate(10);

	int count = 0;
	while (count < 20) {
	    laserMsg.ranges.resize(1);
	    laserMsg.ranges[0] = 0.0;
	    rngpub.publish(laserMsg);

	    dispMsg.data = 10000;
	    dispPub.publish(dispMsg); 

		ros::spinOnce();

	    loop_rate.sleep();
	    ++count;
  	}

  	EXPECT_EQ(0.5, angularZ);
  	EXPECT_EQ(0, linearX);
}
*/
/*
TEST(integrationTest, image_call_back){
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/rgb/image_raw",1);
	cv:: Mat image = cv::imread("/home/yuyuhsueh/catkin_ws/src/final_package",
		CV_LOAD_IMAGE_COLOR);
	cv::waitKey(0.1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
   
     ros::Rate loop_rate(5);
     int count = 0;
     while (count < 20) {
       pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
       ++count;
    }

}
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "ctrller_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

