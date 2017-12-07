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

#include <stdlib.h>
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "TurtleCtrl.hpp"
#include "gazebo_msgs/DeleteModel.h"
#include "final_package/ColorChange.h"
#include "final_package/StatusCheck.h"

TurtleCtrl::TurtleCtrl() {
	dispSub = nh.subscribe("base/disp",1, &TurtleCtrl::dispCallback, this);
	velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	rngSub = nh.subscribe("scan", 1, &TurtleCtrl::rangeCallback, this);
	deleteClient = nh.serviceClient <gazebo_msgs::DeleteModel>("gazebo/delete_model");
	statusCheckCli_ = nh.serviceClient<final_package::StatusCheck>("status_check");
	//nh2.setCallbackQueue(&color_queue);
	colorChangeSrv_ = nh.advertiseService("base_color_change",&TurtleCtrl::colorCallback,this);
	kp = 0.001;
	kd = 0.0001;
	velMax = 0.15;
	lMinimal = 10;
	color = 0;
	terminate = false;
	ROS_INFO("INITIZLIATION");
}

bool TurtleCtrl::colorCallback(final_package::ColorChange::Request &req,
				final_package::ColorChange::Response &resp) {
	color = req.input;
	if (color == -1){
		terminate = true;
	}
	ROS_INFO("Color input from Base:%d",color);
	return true;	
}

void TurtleCtrl::dispCallback( const std_msgs::Int64& dispMsg) {	
	disp = dispMsg.data;
	bool deleteFlag = false;
	geometry_msgs::Twist msg;

	msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

	if (disp == 10000) {
		msg.angular.z = 0.5;
		// Add something to track the robot 
	} 
	else {
		if (lMinimal > 0.6) {
			if (abs(disp) < 35) {
				msg.linear.x = 0.2;
			}
			else{
				msg.angular.z = -(kp*disp+kd*(disp-lDisp));
				if (msg.angular.z > velMax) {
					msg.angular.z = velMax;
				}
				else if (msg.angular.z < -velMax) {
					msg.angular.z = -velMax;
				}
			}
		}
		else{
			 //Calling rosmodel delete service 
			gazebo_msgs::DeleteModel srv;
			final_package::StatusCheck statusSrv;
			switch(color) {
			case 0:
				ROS_ERROR("No color specified");
			case 1:
				srv.request.model_name = "RedBall";
				statusSrv.request.input = true;
				deleteClient.call(srv);
				statusCheckCli_.call(statusSrv);
				deleteFlag = true;
				break;
			case 2:
				srv.request.model_name = "GreenBall";
				statusSrv.request.input = true;
				deleteClient.call(srv);
				statusCheckCli_.call(statusSrv);
				deleteFlag = true;
				break;
			case 3:
				srv.request.model_name = "BlueBall";
				statusSrv.request.input = true;
				deleteClient.call(srv);
				statusCheckCli_.call(statusSrv);
				deleteFlag = true;
				break;
			}
		}
	}
	velPub.publish(msg);
	lDisp = disp;
}

void TurtleCtrl::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float minimal = 10;
  for (auto &i : msg->ranges) {
    if (i < minimal) {
      minimal = i;
    }
  }
  lMinimal = minimal;
}