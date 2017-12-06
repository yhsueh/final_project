#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

class ImageProcess {
public:
	ImageProcess() : detectFlag(false), color(0) {}
	void loadImage(const cv::Mat&);
	void detection();
	const cv::Mat& getImage() {return lastImage;}
	std::vector<cv::Vec3f>circles;
	int getColor() {return color;}
	bool detectFlag;
	int color;

private:
	cv::Mat lastImage;
};