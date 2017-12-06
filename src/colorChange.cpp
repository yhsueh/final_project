#include "ros/ros.h"
#include "final_package/ColorChange.h"
#include <cstdlib>

bool add(final_package::ColorChange::Request  &req,
         final_package::ColorChange::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "colorChange");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<final_package::ColorChange>("base_color_change");
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();




  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}