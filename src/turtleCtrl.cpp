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

/** @file base.cpp
 *	@brief This node takes and analyze the range data. Subsequently, pass the
 *	the decision made based on the data to the turtleCtrl node which 
 *	manipulates the turtlebot.
 *	@author Yuyu Hsueh
 *  @Copyright 2017, Yuyu Hsueh
 */
#include "ros/ros.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include "TurtleCtrl.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtleCtrller");
  TurtleCtrl turtleCtrlObj;
  //ros::AsyncSpinner async_spinner(2, &turtleCtrlObj.color_queue);
  //async_spinner.start();
  ros::Rate loop_rate(5); //5 Htz

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    if (turtleCtrlObj.terminate)
    	break;
  }
  ROS_INFO("Shutting down");
  ros::shutdown();
}
