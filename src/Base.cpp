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

/** 
 *  @file Base.cpp
 *	@brief Base class implementation source file.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int64.h"
#include "final_package/ColorChange.h"
#include "final_package/StatusCheck.h"
#include "Base.hpp"
#include "ImageProcess.hpp"

/**
 * @brief Initialize publishers/subscribers/servers/clients and some constants.
 * @param none
 * @return none
 */
Base::Base() {
  image_transport::ImageTransport it(nh);
  imageSub = it.subscribe("camera/rgb/image_raw", 1, &Base::imageCallback,
                          this);
  cmdPub = nh.advertise < std_msgs::Int64 > ("base/disp", 1);
  colorChangeCli_ = nh.serviceClient < final_package::ColorChange
      > ("base_color_change");
  statusSrv_ = nh.advertiseService("status_check", &Base::statusCallback, this);
  centerline = 640 / 2;
  lDisp = 10000;
  color = 0;
  completeFlag = false;
}

/**
 * @brief This is the service callback from the turtleCtrller node.
 * @param StatusCheck service request and respond.
 * @return true
 */
bool Base::statusCallback(final_package::StatusCheck::Request &req,
                          final_package::StatusCheck::Response &resp) {
  completeFlag = req.input;
  return true;
}

/**
 * @brief In this callback, the displacement of the centeroids of the balls and 
 * the image center is computed and passed to the turtlectrller node. A simple
 * tracking check is added. If the displacment of an obect at one time is significantly
 * smaller or larger than the displacment at last timestep, then the object detected
 * is likely to be a different one.
 * @param sensor_msg::ImageConstPtr& msg
 * @return none
 */

void Base::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std_msgs::Int64 dispMsg;
  try {
  	int disp;
    imgProcess.color = color;
    imgProcess.detectFlag = false;
    imgProcess.loadImage(cv_bridge::toCvShare(msg, "bgr8")->image);
    imgProcess.detection();

    if (imgProcess.detectFlag) {
      for (auto &i : imgProcess.circles) {
        disp = cvRound(i[0]) - centerline;

        if (std::abs(disp - lDisp) < 50 || lDisp == 10000) {
          break;
        }
      }

      if (std::abs(disp - lDisp) > 50)
        ROS_INFO("Tracked object is missing, tracking new object");

      lDisp = disp;
    } else {
      disp = 10000;
    }

    dispMsg.data = disp;
    cmdPub.publish(dispMsg);

    cv::imshow("view", imgProcess.getImage());
    cv::waitKey(1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

