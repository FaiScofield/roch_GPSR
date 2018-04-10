#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "roch_asr_nav/asrnav.h"
#include "std_msgs/String.h"
using namespace std ; 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Gocofetable
{
public:
    Gocofetable();

private:
    void asrCallback(const roch_asr_nav::asrnav::ConstPtr& msg);
    ros::NodeHandle node;
    float x_goal,y_goal,w_goal;
    ros::Publisher cmdVelPub;
    ros::Subscriber sub;
    void set_goal_site(float x_goal,float y_goal,float w_goal);
    move_base_msgs::MoveBaseGoal goal;

};

    Gocofetable::Gocofetable(){
 sub = node.subscribe("/number", 1, &Gocofetable::asrCallback, this);
 }

    void Gocofetable::set_goal_site(float x_goal,float y_goal,float w_goal){
    MoveBaseClient ac("move_base", true);
    
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x_goal;
	goal.target_pose.pose.position.y = y_goal;
	goal.target_pose.pose.orientation.w = w_goal;
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
}
    void Gocofetable::asrCallback(const roch_asr_nav::asrnav::ConstPtr& msg){
    int a=msg->num;
    
   switch(a) 
	{
		case 0: set_goal_site(-0.458, 1.734, 1.575);
		break;
		case 1: set_goal_site( 0.640, 1.723, 1.557);
		break;
		case 2: set_goal_site( 1.798, 1.938, 1.567);
		break;
		case 3: set_goal_site( 2.835, 1.724, 1.585);
		break;
		case 4: set_goal_site( 1.700, 0.770,-1.540);
		break;
		case 5: set_goal_site( 2.665,-0.028,-1.577);
		break;
		case 6: set_goal_site( 5.619,-0.699,-0.914);
		break;
		case 7: set_goal_site(7.103, 1.273, 1.572 );
		break;
		case 8: set_goal_site(7.438,-0.150, -3.111 );
		break;
		case 9: set_goal_site( 8.588, -0.383, -1.592);
		break;
		case 10: set_goal_site( 8.453, 1.807, 1.581);
		break;
		case 11: set_goal_site(9.151, 0.955, -0.006 );
		break;
		case 12: set_goal_site(4.868, 6.220,1.519 );
		break;
		case 13: set_goal_site( 6.711, 6.008, 1.524);
		break;
		case 14: set_goal_site( 7.349, 5.670, -1.574);
		break;
		case 15: set_goal_site( 7.300, 3.621, -1.565);
		break;
		case 16: set_goal_site(10.601, 3.307,-0.772 );
		break;
		case 17: set_goal_site( -0.422, 5.654, 1.573);
		break;
		case 18: set_goal_site( 1.189, 5.704, 1.573);
		break;
		case 19: set_goal_site(2.588, 5.819, 1.551 );
		break;
		case 20: set_goal_site( 3.604,5.820, 1.532);
		break;
		case 21: set_goal_site( 1.603, 4.028,-1.570);
		break;
		case 22: set_goal_site( -0.461, 4.072, -1.563);
		break;
		case 23: set_goal_site( 2.031, 1.159, 0.001);
		break;
		case 24: set_goal_site( 8.089, 0.959, 0.001);
		break;
		case 25: set_goal_site( 5.340, 4.679, 0.001);
		break;
		case 26: set_goal_site( 1.693,4.893, -3.141);
		break;		
    }

   }
int main(int argc, char** argv){
	ros::init(argc, argv, "site_nav");
    Gocofetable a;
    ros::spin();
    ros::shutdown();
	return 0;
}
