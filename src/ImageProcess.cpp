#include "ImageProcess.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

ImageProcess::ImageProcess() {};

void ImageProcess::loadImage (const cv::Mat& img) {
	lastImage = img;
	//cv::fastNlMeansDenoisingColored(lastImage, lastImage);
	//cv::cvtColor(lastImage, lastImage, )
}


void ImageProcess::detection() {
	cv::Mat lastImageHSV;
	cv::Mat HSVchannels[3];
	cv::Mat mask;
	cv::cvtColor(lastImage, lastImageHSV, CV_BGR2HSV);
	//cv::split(lastImageHSV,HSVchannels);
	cv::inRange(lastImageHSV, cv::Scalar(0,0,86.3), cv::Scalar(0,0,41.2), mask);
    lastImage = mask;
}