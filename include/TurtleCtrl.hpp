#pragma once
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class TurtleCtrl {
public:
	TurtleCtrl();
	void dispCallback(const std_msgs::Int64& dispMsg);
	void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
private:
	ros::NodeHandle nh;
	ros::Subscriber rngSub;
	ros::Publisher rngPub;
	ros::Publisher velPub;
	ros::Subscriber dispSub;
	float velMax;
	float kp;
	float odometry;
};



