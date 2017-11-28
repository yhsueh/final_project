#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/LaserScan.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float minimal = 10;
  for (auto &i : msg->ranges) {
    if (i < minimal) {
      minimal = i;
    }
  }
  ROS_INFO("Minimal distance is: %f", minimal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  
  ros::Subscriber sub2 = nh2.subscribe("scan", 10, rangeCallback);

  ros::spin();
  cv::destroyWindow("view");
}
