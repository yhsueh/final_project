#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"

TEST(UnitTesting, color_test_blue_green) {
	//ImageProcess imgObj;
	//ImageProcess imgObj2;

	cv::Mat image = cv::imread("../Blueball.jpg",CV_LOAD_IMAGE_COLOR);
	//image2 = cv::imread("~/catkin_ws/src/final_package/Greenball.jpg", CV_LOAD_IMAGE_UNCHANGED);
	//imgObj.loadImage(image);
	int row = image.rows;

	EXPECT_EQ(row,1);


	//cv::cvtColor(image,image2,CV_BGR2HSV);
	//imgObj.detection();
	//imgObj2.loadImage(image2);

	/*int color = 1;
	int color2 = 1;
	while(1) {
		imgObj.color = color;
		imgObj.detection();

		if (imgObj.circles.size() > 0) {
			break;
		}

		color+=1;
		break;
	}
*/
	//EXPECT_EQ(color,3);

/*
	while(1) {
		imgObj2.color = color2;
		imgObj2.detection();

		if (imgObj2.circles.size() > 0) {
			break;
		}

		color2 += 1;
	}

	EXPECT_EQ(color,2);
	*/
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
