#include "Base.hpp"
#include "ImageProcess.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

Base::Base() {
  image_transport::ImageTransport it(nh);
	imageSub = it.subscribe("camera/rgb/image_raw", 1000, &Base::imageCallback, this);  
  sub = nh.subscribe("scan", 1000, &Base::rangeCallback, this);
  pub = nh.advertise<std_msgs::Float32>("base/min_distance",1000);
}

void Base::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat newImage;
	try
	{  
    	cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    	//newImage = cv_bridge::toCvShare(msg, "bgr8")->image;
      /*
      imgProcess.loadImage(cv_bridge::toCvShare(msg, "bgr8")->image);
      imgProcess.detection();
      newImage = imgProcess.getImage();
      cv::imshow("view", newImage);
*/
    	cv::waitKey(1);
  	}
  	catch (cv_bridge::Exception& e)
  	{
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

void Base::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float minimal = 10;
  for (auto &i : msg->ranges) {
    if (i < minimal) {
      minimal = i;
    }
  }
  std_msgs::Float32 distMsg;
  distMsg.data = minimal;
  pub.publish(distMsg);
  ROS_INFO("Minimal distance is: %f", minimal);
}