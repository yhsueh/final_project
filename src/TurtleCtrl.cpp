#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int64.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "TurtleCtrl.hpp"
#include "gazebo_msgs/DeleteModel.h"
#include "final_package/ColorChange.h"

TurtleCtrl::TurtleCtrl() {
	dispSub = nh.subscribe("base/disp",10, &TurtleCtrl::dispCallback, this);
	velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	rngSub = nh.subscribe("scan", 10, &TurtleCtrl::rangeCallback, this);
	colorClient = nh.serviceClient <gazebo_msgs::DeleteModel>("gazebo/delete_model");
	kp = 0.001;
	velMax = 0.3;
	lMinimal = 10;
	color = 1;
}

bool TurtleCtrl::cmdVel() {
	geometry_msgs::Twist msg;
	ROS_INFO("DISP:%d",disp);

	msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

	if (disp == 10000) {
		msg.angular.z = 0.3;
		// Add something to track the robot 
	} 
	else {
		if (lMinimal > 0.6) {
			if (abs(disp) < 25) {
				msg.linear.x = 0.2;
			}
			else if (disp > 0) {
				msg.angular.z = -kp*disp;
				if (abs(msg.angular.z) > velMax) {
					msg.angular.z = -velMax;
				}
			}
			else if (disp < 0) {
				msg.angular.z = kp*disp;
				if (abs(msg.angular.z) > velMax) {
					msg.angular.z = velMax;
				}
			}
		}
		else{
			 //Calling rosmodel delete service 
			gazebo_msgs::DeleteModel srv;
			switch(color) {
			case 1:
				srv.request.model_name = "RedBall";
				colorClient.call(srv);
				break;
			case 2:
				srv.request.model_name = "GreenBall";
				colorClient.call(srv);
				break;
			case 3:
				srv.request.model_name = "BlueBall";
				colorClient.call(srv);
				break;
			}
		}
	}
	velPub.publish(msg);
}

void TurtleCtrl::dispCallback( const std_msgs::Int64& dispMsg) {	
	disp = dispMsg.data;
	this->cmdVel();
}

void TurtleCtrl::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float minimal = 10;
  for (auto &i : msg->ranges) {
    if (i < minimal) {
      minimal = i;
    }
  }
  lMinimal = minimal;
  ROS_INFO("Minimal distance is: %f", minimal);
}