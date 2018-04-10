#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roch_asr_nav/asrnav.h"
#include <sstream>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "publish_int");
  ros::NodeHandle n;
  ros::Publisher number_pub = n.advertise<roch_asr_nav::asrnav>("number", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
 // std_msgs::String msg;
    roch_asr_nav::asrnav msg;
    msg.num = count;
    ROS_INFO("%d", msg.num);
    number_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
