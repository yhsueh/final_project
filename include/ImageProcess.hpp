#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ImageProcess {
public:
	ImageProcess() : detectFlag(false){}
	void loadImage(const cv::Mat&);
	void detection();
	const cv::Mat& getImage() {return lastImage;}
	std::vector<cv::Vec3f>circles;

private:
	cv::Mat lastImage;
	bool detectFlag;
	









};