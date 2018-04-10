#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roch_asr_nav/asrnav.h"

void chatterCallback(const roch_asr_nav::asrnav::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->num);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listen_int");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("number", 1000, chatterCallback);
  ros::spin();
  return 0;
}
