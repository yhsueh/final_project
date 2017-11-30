#include <ros/ros.h>
#include "BaseTestCallback.hpp"
#include "std_msgs/Float32.h"
#include <iostream>
void BaseTestCallback::distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
	minimum = msg->data;
	//std::cout << "Min: " << minimum << std::endl;
}