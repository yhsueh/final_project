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
 *  @file imageProcess_test.cpp
 *	@brief This is the testing file for imageProcess class. 
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"


/** @brief A method for testing imageProcess's capability to detect the
  * correct color given two images balls obtained from internet.
  * @test Detect the selected image and see if the circle vector returned
  * by cv::houghTransformCircle is greater than one. When the size of the
  * circle is greater than one, then the color of the ball is equivalent 
  * to the color of the filter.
  */

TEST(UnitTesting, color_test_blue_green) {
  ImageProcess imgObj, imgObj2;

  while (imgObj.color < 4) {
    imgObj.loadImage(cv::imread("./Greenball.jpg",CV_LOAD_IMAGE_COLOR));
    imgObj.detection();

    if (imgObj.circles.size() > 0) {
      break;
    }

    imgObj.color+=1;
  }

  EXPECT_EQ(imgObj.color,2);

  while (imgObj2.color < 4) {
    imgObj2.loadImage(cv::imread("./Blueball.jpg",CV_LOAD_IMAGE_COLOR));
    imgObj2.detection();

    if (imgObj2.circles.size() > 0) {
      break;
    }

    imgObj2.color+=1;
  }

  EXPECT_EQ(imgObj2.color,3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
