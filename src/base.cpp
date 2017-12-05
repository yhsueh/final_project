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
  cv::namedWindow("view");
  cv::startWindowThread();
  Base baseObj;
  final_package::ColorChange srv;
  srv.request.input = baseObj.color;
  bool colorChangeFlag = baseObj.colorChangeCli_.call(srv);
  ROS_INFO("service request bool: %c", srv.request.input);
  //ROS_INFO("Service reseponse: %c", srv.response.output);

  ros::Rate loop_rate(5); //5 Htz
  ros::AsyncSpinner spinner(2,&baseObj.queue);
  spinner.start();

  while(ros::ok()) {
    colorChangeFlag = false;
    srv.request.input = baseObj.color;

    ROS_INFO("RESPONSEOUTPUT = %d", srv.response.output);
    if (srv.response.output) {
      baseObj.color += 1;
      if (baseObj.color > 3) {
        ROS_INFO("Nothing is left");
        ros::shutdown();
      }
    }    
  
  	ros::spinOnce();
  	loop_rate.sleep();
  }
  
  cv::destroyWindow("view");
}
