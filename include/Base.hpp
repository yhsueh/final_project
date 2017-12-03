#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/LaserScan.h"
#include "ImageProcess.hpp"

class Base {
	public:
		Base();
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		//void computeCentroids()
	private:
		ros::NodeHandle nh;
		ros::Subscriber rngSub;
		ros::Publisher rngPub;
		ros::Publisher cmdPub;
		image_transport::Subscriber imageSub;
		ImageProcess imgProcess;
	    int centerline;
		int buffer;
		int lDisp;
		//cv::Mat lastImage;
		//float minDistance;
};

