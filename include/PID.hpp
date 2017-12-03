#pragma once
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

class PID {
public:
	PID() : kp(10), kd(5), deltaT(0.2) {}




private:
	double kp;
	double kd;
	double deltaT;
};