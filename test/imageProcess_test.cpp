#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"

TEST(UnitTesting, color_test_blue_green) {
	ImageProcess imgObj, imgObj2;

	while(imgObj.color < 4) {		
		imgObj.loadImage(cv::imread("./Greenball.jpg",CV_LOAD_IMAGE_COLOR));
		imgObj.detection();

		if (imgObj.circles.size() > 0) {
			break;
		}

		imgObj.color+=1;
	}

	EXPECT_EQ(imgObj.color,2);

	while(imgObj2.color < 4) {		
		imgObj2.loadImage(cv::imread("./Blueball.jpg",CV_LOAD_IMAGE_COLOR));
		imgObj2.detection();

		if (imgObj2.circles.size() > 0) {
			break;
		}

		imgObj2.color+=1;
	}

	EXPECT_EQ(imgObj2.color,3);
}
	




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
