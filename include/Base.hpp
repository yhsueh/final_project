#pragma once
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"
#include "final_package/StatusCheck.h"

class Base {
	public:
		Base();
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		bool statusCallback(final_package::StatusCheck::Request& req,
			final_package::StatusCheck::Response &resp);
		bool completeFlag;
		ros::ServiceClient colorChangeCli_;
		int color; //1:Red, 2:Green, 3:Blue
	private:
		ros::NodeHandle nh;
		ros::Publisher cmdPub;
		ros::ServiceServer statusSrv_;
		image_transport::Subscriber imageSub;
		ImageProcess imgProcess;
		float centerline;
		int lDisp;
		
};

