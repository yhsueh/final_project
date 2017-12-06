#pragma once
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "final_package/ColorChange.h"

class TurtleCtrl {
public:
	TurtleCtrl();
	void dispCallback(const std_msgs::Int64& dispMsg);
	void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	bool cmdVel();
	bool colorCallback(final_package::ColorChange::Request &req,
				final_package::ColorChange::Response &resp);
	//ros::CallbackQueue color_queue;
	ros::ServiceClient statusCheckCli_;
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh2;
	ros::Subscriber rngSub;
	ros::Publisher velPub;
	ros::Subscriber dispSub;
	ros::ServiceClient deleteClient;
	ros::ServiceServer colorChangeSrv_;
	float velMax;
	float kp;
	float odometry;
	float lMinimal;
	int color;
	int disp;
	bool completeFlag;
};



