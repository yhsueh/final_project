#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int64.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "TurtleCtrl.hpp"

TurtleCtrl::TurtleCtrl() {
	dispSub = nh.subscribe("base/disp",10, &TurtleCtrl::dispCallback, this);
	velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	rngSub = nh.subscribe("scan", 10, &TurtleCtrl::rangeCallback, this);
    rngPub = nh.advertise<std_msgs::Float32>("base/min_distance",10);
	kp = 0.001;
	velMax = 0.3;
}

void TurtleCtrl::dispCallback( const std_msgs::Int64& dispMsg) {
	geometry_msgs::Twist msg;
	int disp = dispMsg.data;

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
	else if (abs(disp) < 25) {
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
	velPub.publish(msg);
}

void TurtleCtrl::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float minimal = 10;
  for (auto &i : msg->ranges) {
    if (i < minimal) {
      minimal = i;
    }
  }
  std_msgs::Float32 distMsg;
  distMsg.data = minimal;
  rngPub.publish(distMsg);
  ROS_INFO("Minimal distance is: %f", minimal);
}