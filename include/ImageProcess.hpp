#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

class ImageProcess {
public:
	ImageProcess();
	void loadImage(const cv::Mat&);
	void detection();
	cv::Mat getImage() {return lastImage;}


private:
	cv::Mat lastImage;









};