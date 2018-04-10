#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roch_asr_nav/asrnav.h"
#include <stdio.h>
#include <iostream>
using namespace std;

class Lisandpub{
public:
    Lisandpub();
private:
    int a,b;
    roch_asr_nav::asrnav c;
    ros::NodeHandle n;
    ros::Publisher lis_pub;
    ros::Subscriber lis_sub;    
    void chatterCallback(const roch_asr_nav::asrnav::ConstPtr& msg); 
};

void Lisandpub::chatterCallback(const roch_asr_nav::asrnav::ConstPtr& msg)
{
    a=msg->num;
    b=a+10;
    c.num = a + 10;
    ROS_INFO("I heard: [%d]", c.num);
    lis_pub.publish(c);
}

Lisandpub::Lisandpub()
{
    lis_pub = n.advertise<roch_asr_nav::asrnav>("number2", 1000);
    lis_sub = n.subscribe("number", 1000, &Lisandpub::chatterCallback,this);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listen_and_publish");
  Lisandpub Lisandpub;
  ros::spin();
  return 0;
}
