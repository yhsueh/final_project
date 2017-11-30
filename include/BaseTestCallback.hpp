#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

class BaseTestCallback {
public:
	void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
	float minimum;
};