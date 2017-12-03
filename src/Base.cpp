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
#include "std_msgs/Int64.h"
#include <stdlib.h>  

Base::Base() {
  centerline = 640/2;
  buffer = 5;
  image_transport::ImageTransport it(nh);
	imageSub = it.subscribe("camera/rgb/image_raw", 10, &Base::imageCallback, this);  
  rngSub = nh.subscribe("scan", 10, &Base::rangeCallback, this);
  rngPub = nh.advertise<std_msgs::Float32>("base/min_distance",10);
  cmdPub = nh.advertise<std_msgs::Int64>("base/disp",10);

}

void Base::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  //cv::Mat newImage;
	int displacement;
  try
	{  
    	//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    	//newImage = cv_bridge::toCvShare(msg, "bgr8")->image;
      
      imgProcess.loadImage(cv_bridge::toCvShare(msg, "bgr8")->image);
      imgProcess.detection();
      //newImage = ;
      //ROS_INFO("CircleSize:%zu", imgProcess.circles.size());
      
      if (imgProcess.detectFlag) {
        //PID PIDObj;
        for (auto &i : imgProcess.circles) {
          //cv::Point center(cvRound(i[0]),cvRound(i[1]));
          displacement = cvRound(i[0]) - centerline;
          break;
        }

        std_msgs::Int64 dispMsg;
        dispMsg.data = displacement;
        cmdPub.publish(dispMsg);
        ROS_INFO("Displacement is: %d", displacement);
      }

//      cv::imshow("view", imgProcess.getImage());
//    	cv::waitKey(1);
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
  rngPub.publish(distMsg);
  ROS_INFO("Minimal distance is: %f", minimal);
}