#include "Base.hpp"
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/LaserScan.h"
#include "final_package/ColorChange.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base");

  Base baseObj;
  final_package::ColorChange srv;
  //Starting with red
  baseObj.color = 1;
  srv.request.input = baseObj.color;
  bool colorChangeFlag = baseObj.colorChangeCli_.call(srv);
  int pickCount = 0; 

  cv::namedWindow("view");
  cv::startWindowThread();

  ros::Rate loop_rate(5); //5 Htz

  while(ros::ok()) {
    if (baseObj.completeFlag) {
      baseObj.color += 1;
      srv.request.input = baseObj.color;
      if (baseObj.colorChangeCli_.call(srv)) {
        ROS_INFO("New color is specified");
      }
      pickCount += 1;
      baseObj.completeFlag = false;
      if (pickCount == 3) {
        ROS_INFO("All ball are being collected");
        break;
      }
    }
  	ros::spinOnce();
  	loop_rate.sleep();
  }
  srv.request.input = -1;
  baseObj.colorChangeCli_.call(srv);
  cv::destroyWindow("view");
  ros::shutdown();
}
