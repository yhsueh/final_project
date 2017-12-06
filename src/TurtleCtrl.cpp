#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <stdlib.h>
#include "std_msgs/Int64.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
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