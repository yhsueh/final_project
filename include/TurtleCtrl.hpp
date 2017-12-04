#pragma once
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"

class TurtleCtrl {
public:
	TurtleCtrl();
	void dispCallback(const std_msgs::Int64& dispMsg);
private:
	ros::NodeHandle nh;
	ros::Publisher velPub;
	ros::Subscriber dispSub;
	float velMax;
	float kp;
	float odometry;
};



