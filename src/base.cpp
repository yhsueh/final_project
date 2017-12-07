/*
 * Copyright (C) 2017, Yuyu Hsueh.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file base.cpp
 *	@brief This is a source file for the base node of this project. The base
 *  node is responsible of getting and converting the images from the 
 *  asus-xtion_pro 3D sensor. Then, it passes these images into an ImageProcess
 *  object, which processes these images and search for the balls' centroids.
 *  Finally, the displacment between the center of the ball and the center of the
 *  image is published to the turtleCtrller node in charge of the turtlebot actuator.
 *  In every iteration, the processed image would be displayed in an view window
 *  for users to see what the turtlebot sees. There are two services called to
 *  make sure all colors of balls are being collected. All the callbacks in 
 *  this node are running at the rate of 5Hz.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

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
  ros::init(argc, argv, "base"); /**< Node declaration */  
  Base baseObj;  /**< Node is being implemented as a class object.*/
  final_package::ColorChange srv;
 
  baseObj.color = 1;  /**< Begin the program with the red ball. */
  srv.request.input = baseObj.color;
  bool colorChangeFlag = baseObj.colorChangeCli_.call(srv);
  int pickCount = 0; 

  cv::namedWindow("view");
  cv::startWindowThread();

  ros::Rate loop_rate(5);

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
