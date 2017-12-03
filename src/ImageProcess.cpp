#include "ImageProcess.hpp"
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>


ImageProcess::ImageProcess() {};

void ImageProcess::loadImage (const cv::Mat& img) {
	lastImage = img;
}


void ImageProcess::detection() {
	cv::Mat structure = cv::getStructuringElement(1,cv::Size(7,7));
	cv::cvtColor(lastImage, lastImage, CV_BGR2HSV);
	cv::inRange(lastImage, cv::Scalar(0,70,0), cv::Scalar(0,255,255), lastImage);
	cv::erode(lastImage, lastImage, structure);
	cv::HoughCircles(lastImage, circles, CV_HOUGH_GRADIENT, 2, lastImage.rows/16, 100, 30, 1, 300);

	int count = 0;
	/*
	for (auto&i : circles) {
		cv::Point center(cvRound(i[0]),cvRound(i[1]));
		ROS_INFO("CircleCenter X:%d,Y:%d",center.x,center.y);
		count+=1;
	}
	*/
	if (count > 0) {
		detectFlag = true;
	}


}