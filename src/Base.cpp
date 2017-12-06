#include "Base.hpp"
#include "ImageProcess.hpp"
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int64.h"
#include <stdlib.h>
#include "final_package/ColorChange.h"
#include "final_package/StatusCheck.h"

Base::Base() {
  image_transport::ImageTransport it(nh);
	imageSub = it.subscribe("camera/rgb/image_raw", 1, &Base::imageCallback, this);  
  cmdPub = nh.advertise<std_msgs::Int64>("base/disp",1);  
  colorChangeCli_ = nh.serviceClient<final_package::ColorChange>("base_color_change");
  statusSrv_ = nh.advertiseService("status_check",&Base::statusCallback,this);
  centerline = 640/2;
  lDisp = 10000;
  color = 0;
  completeFlag = false;
}

bool Base::statusCallback(final_package::StatusCheck::Request &req,
				final_package::StatusCheck::Response &resp) {
  completeFlag = req.input;
  return true;
}

void Base::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	int disp;
  std_msgs::Int64 dispMsg;
  try
	{
      imgProcess.color = color;
      imgProcess.detectFlag = false;
      imgProcess.loadImage(cv_bridge::toCvShare(msg, "bgr8")->image);
      imgProcess.detection();

      if (imgProcess.detectFlag) {
        for (auto &i : imgProcess.circles) {
          disp = cvRound(i[0]) - centerline;
          
          /*Tracking*/
          if (std::abs(disp-lDisp) < 50 || lDisp == 10000) { 
            break;
          }
        }

        //ROS_INFO("Displacment is:%d LDisp is :%d", disp, lDisp);

        if (std::abs(disp-lDisp) > 50)
          ROS_INFO("Tracked object is missing, tracking new object");
        
        lDisp = disp;
      }
      else {
        disp = 10000;
      }

      dispMsg.data = disp;
      cmdPub.publish(dispMsg);

      cv::imshow("view", imgProcess.getImage());
    	cv::waitKey(1);
  	}
  	catch (cv_bridge::Exception& e)
  	{
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

