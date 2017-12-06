#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ImageProcess.hpp"

void ImageProcess::loadImage (const cv::Mat& img) {
	lastImage = img;
}

void ImageProcess::detection() {
	cv::Mat structure = cv::getStructuringElement(1,cv::Size(7,7));
	cv::cvtColor(lastImage, lastImage, CV_BGR2HSV);
	bool noColor = false;
	switch(color) {
		case 0:
			ROS_INFO("No color specified");
			noColor = true;
			break;
		case 1:
			cv::inRange(lastImage, cv::Scalar(0,70,0), cv::Scalar(0,255,255), lastImage); //R
			break;
		case 2:
			cv::inRange(lastImage, cv::Scalar(50,90,0), cv::Scalar(70,255,255), lastImage); //G
			break;
		case 3:
			cv::inRange(lastImage, cv::Scalar(120,50,0), cv::Scalar(120,255,255), lastImage);//B
			break;
	}

	if (!noColor) {
		cv::erode(lastImage, lastImage, structure);
		cv::HoughCircles(lastImage, circles, CV_HOUGH_GRADIENT, 2, lastImage.rows/16, 100, 30, 1, 500);

		if (circles.size() > 0) {
			detectFlag = true;
		}
	}
}