#include "ImageProcess.hpp"
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>


void ImageProcess::loadImage (const cv::Mat& img) {
	lastImage = img;
}


void ImageProcess::detection() {
	cv::Mat structure = cv::getStructuringElement(1,cv::Size(7,7));
	cv::cvtColor(lastImage, lastImage, CV_BGR2HSV);
	cv::inRange(lastImage, cv::Scalar(0,70,0), cv::Scalar(0,255,255), lastImage);
	cv::erode(lastImage, lastImage, structure);
	cv::HoughCircles(lastImage, circles, CV_HOUGH_GRADIENT, 2, lastImage.rows/16, 100, 30, 1, 300);

	if (circles.size() > 0) {
		detectFlag = true;
	}
}