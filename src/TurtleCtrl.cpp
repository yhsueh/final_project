#include "geometry_msgs/Twist.h"
geometry_msgs::Twist msg; /**<The geometry msgs that would be sent to the mobile node */


TurtleCtrl::TurtleCtrl() {
	dispSub = nh.subscribe("base/disp",10, &TurtleCtrl::dispCallback, this);
	velPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
}

void TurtleCtrl::dispCallback( const std_msgs::Int64& dispMsg) {

/*
if (vel_msg.data) {
    msg.linear.x = 0.5;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
  } else {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.5;
  }
*/

}