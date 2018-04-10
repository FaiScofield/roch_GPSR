#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "roch_asr_nav/asrnav.h"
#include "std_msgs/String.h"

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
};

    Gocofetable::Gocofetable(){
 sub = node.subscribe("/number", 1, &Gocofetable::asrCallback, this);
 }

    void Gocofetable::asrCallback(const roch_asr_nav::asrnav::ConstPtr& msg){
    /*
    std::string a="a";
    std::string b="b";
    std::string c="c";
    std::string d="d";   
    std::string e="e";
    std::string f="f";
    std::string g="g";
    std::string h="h";
    std::string i="i";
    std::string j="j";   
    std::string k="k";
    std::string l="l";
    std::string m="m";
    std::string n="n";
    std::string o="o";
    std::string p="p";   
    std::string q="q";
    std::string r="r";
    std::string s="s";
    std::string t="t";
    std::string u="u";
    std::string v="v";   
    std::string w="w";
    std::string x="x"; 
    std::string y="y";
    std::string z="z";   
           */
    int a=msg->num;
    
   switch(a) 
	{
		case 0: x_goal= -0.458,y_goal= 1.734,w_goal= 1.575;
		break;
		case 1: x_goal=  0.640,y_goal= 1.723 ,w_goal= 1.557;
		break;
		case 2: x_goal= 1.798,y_goal= 1.938,w_goal= 1.567;
		break;
		case 3: x_goal= 2.835,y_goal= 1.724,w_goal= 1.585;
		break;
		case 4: x_goal= 1.700,y_goal= 0.770,w_goal= -1.540;
		break;
		case 5: x_goal= 2.665,y_goal= -0.028,w_goal= -1.577;
		break;
		case 6: x_goal= 5.619,y_goal= -0.699,w_goal= -0.914;
		break;
		case 7: x_goal= 7.103,y_goal= 1.273,w_goal= 1.572;
		break;
		case 8: x_goal= 7.438,y_goal= -0.150,w_goal= -3.111;
		break;
		case 9: x_goal= 8.588,y_goal= -0.383,w_goal= -1.592;
		break;
		case 10: x_goal= 8.453,y_goal= 1.807,w_goal= 1.581;
		break;
		case 11: x_goal= 9.151,y_goal= 0.955,w_goal= -0.006;
		break;
		case 12: x_goal= 4.868,y_goal= 6.220,w_goal= 1.519;
		break;
		case 13: x_goal= 6.711,y_goal= 6.008,w_goal= 1.524;
		break;
		case 14: x_goal= 7.349,y_goal= 5.670,w_goal= -1.574;
		break;
		case 15: x_goal= 7.300,y_goal= 3.621,w_goal= -1.565;
		break;
		case 16: x_goal= 10.601,y_goal= 3.307,w_goal= -0.772;
		break;
		case 17: x_goal= -0.422,y_goal= 5.654,w_goal= 1.573;
		break;
		case 18: x_goal= 1.189,y_goal= 5.704,w_goal= 1.573;
		break;
		case 19: x_goal= 2.588,y_goal= 5.819,w_goal= 1.551;
		break;
		case 20: x_goal= 3.604,y_goal= 5.820,w_goal= 1.532;
		break;
		case 21: x_goal= 1.603,y_goal= 4.028,w_goal= -1.570;
		break;
		case 22: x_goal= -0.461,y_goal= 4.072,w_goal= -1.563;
		break;
		case 23: x_goal= 2.031,y_goal= 1.159,w_goal= 0.001;
		break;
		case 24: x_goal= 8.089,y_goal= 0.959,w_goal= 0.001;
		break;
		case 25: x_goal= 5.340,y_goal= 4.679,w_goal= 0.001;
		break;
		case 26: x_goal= 1.693,y_goal= 4.893,w_goal= -3.141;
		break;		
}
//    if(a==msg->data.c_str()){ 
//  if(strcmp(msg->data.c_str(),a)){} 
    ROS_INFO("GetGoal!");
    MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = x_goal;
	goal.target_pose.pose.position.y = y_goal;
	goal.target_pose.pose.orientation.w = w_goal;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the coffee_table goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	}
	
int main(int argc, char** argv){
	ros::init(argc, argv, "nav_home");
    Gocofetable a;
    ros::spin();
    ros::shutdown();
	return 0;
}
