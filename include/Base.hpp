/*
 * Copyright (C) 2017, Yuyu Hsueh.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** 
 *  @file Base.hpp
 *	@brief In this project, the nodes are being implemented as class objects.
 *  This is the header file for the base class.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */

#pragma once
#include <ros/ros.h>
#include <ros/advertise_service_options.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageProcess.hpp"
#include "final_package/StatusCheck.h"


/** 
* The base class initializes all the necessary publishers, subscribers,
* servers and clients. It also takes cares of the callbacks for subscribers
* and servers. It's core function is to locate the ball objects and command
* the turtlebot to reach it.
*/
class Base {
	public:
		Base();

		/**
		  * The camera images are obtained from the 3D sensor. These images are passed
		  * to an imageprocess object for further image analysis. At the end, the centroid
		  * of the detected circles are computed.
		  */
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);


		/** 
		  * This function is the service callback from the turtleCtrller node. It
		  * recieves a request when a model in Gazebo is removed.
		  */
		bool statusCallback(final_package::StatusCheck::Request& req,
			final_package::StatusCheck::Response &resp);
		bool completeFlag;
		ros::ServiceClient colorChangeCli_; /**< Tells the turtlebot which color ball to collect*/
		int color; /**< 1:Red, 2:Green, 3:Blue */
	private:
		ros::NodeHandle nh;
		ros::Publisher cmdPub;
		ros::ServiceServer statusSrv_;
		image_transport::Subscriber imageSub;
		ImageProcess imgProcess;
		float centerline;
		int lDisp;
		
};

