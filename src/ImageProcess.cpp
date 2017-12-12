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
 *  @file ImageProcess.cpp
 *	@brief This node takes and analyze the range data. Subsequently, pass the
 *	the decision made based on the data to the turtleCtrl node which 
 *	manipulates the turtlebot.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"

void ImageProcess::loadImage(const cv::Mat& img) {
  cv::GaussianBlur(img,lastImage,cv::Size(9,9),0,0);
  //lastImage = img;
}

/**
 * @brief Based on the color chosen in the range from 1 to 3, the corresponding filter
 * will be applied in HSV color space. If no color is specified, it would do nothing and 
 * await next callback.
 * @param none;
 * @return none;
 */

cv::Mat& ImageProcess::getImage() {
  return lastImage;
}

void ImageProcess::detection() {
  cv::Mat structure = cv::getStructuringElement(1, cv::Size(7, 7));
  cv::Mat structure2 = cv::getStructuringElement(1, cv::Size(9, 9));
  cv::cvtColor(lastImage, lastImage, CV_BGR2HSV);
  bool noColor = false;
  switch (color) {
    case 0:
      ROS_INFO("No color specified");
      noColor = true;
      break;
    case 1:
      cv::inRange(lastImage, cv::Scalar(0, 200, 70), cv::Scalar(5, 255, 255),
                  lastImage);  // Red
      break;
    case 2:
      cv::inRange(lastImage, cv::Scalar(25, 90, 0), cv::Scalar(70, 255, 255),
                  lastImage);  // Green
      break;
    case 3:
      cv::inRange(lastImage, cv::Scalar(100, 50, 0), cv::Scalar(140, 255, 255),
                  lastImage);  // Blue
      break;
  }

  if (!noColor) {
    cv::erode(lastImage, lastImage, structure);
    cv::dilate(lastImage, lastImage, structure2);
    cv::HoughCircles(lastImage, circles, CV_HOUGH_GRADIENT, 2,
                     lastImage.rows / 16, 100, 30, 0, 500);

    if (circles.size() > 0) {
      detectFlag = true;
    }
  }
}
