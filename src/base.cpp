#include "Base.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/LaserScan.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base");
  cv::namedWindow("view");
  cv::startWindowThread();
  Base baseObj;

  ros::Rate loop_rate(5); //5 Htz

  while(ros::ok()) {

  	ros::spinOnce();
  	loop_rate.sleep();
  }
  
  cv::destroyWindow("view");
}
