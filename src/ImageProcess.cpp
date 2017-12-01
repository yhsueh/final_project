#include "ImageProcess.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

ImageProcess::ImageProcess() {};

void ImageProcess::loadImage (const cv::Mat& img) {
	lastImage = img;
	cv::fastNlMeansDenoisingColored(lastImage, lastImage);
	//cv::cvtColor(lastImage, lastImage, )
}


void ImageProcess::detection() {
	lastImage 

}