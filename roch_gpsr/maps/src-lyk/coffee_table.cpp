#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
//#include "roch_asr_nav/asrnav.h"
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Gocofetable
{
public:
    Gocofetable();

private:
    void asrCallback(const std_msgs::String::ConstPtr& msg);
    ros::NodeHandle node;
    float x_goal,y_goal,w_goal;
    ros::Publisher cmdVelPub;
    ros::Subscriber sub;
};

    Gocofetable::Gocofetable(){
 sub = node.subscribe("/chatter", 1, &Gocofetable::asrCallback, this);
 }

    void Gocofetable::asrCallback(const std_msgs::String::ConstPtr& msg){
    std::string a="a";
    std::string b="b";
    std::string c="c";
    std::string d="d";
    
  //  if(a==msg->data.c_str()){ msg->data.c_str()和a==msg->data两种方法都可以
  /*a="a"*/
    if(a==msg->data){
    ROS_INFO("GetGoal!");
    MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 5.8;
	goal.target_pose.pose.position.y = 4.5;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the coffee_table goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	}
	/*b="b"*/
	    if(b==msg->data){
    ROS_INFO("GetGoal!");
    MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 1.585;
	goal.target_pose.pose.position.y = 4.133;
	goal.target_pose.pose.orientation.w = -1.580;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the coffee_table goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	}
	/*c="c"*/
	    if(c==msg->data){
    ROS_INFO("GetGoal!");
    MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 9.06;
	goal.target_pose.pose.position.y = 1.074;
	goal.target_pose.pose.orientation.w = -0.009;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the coffee_table goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	}
	/*d="d"*/
	    if(d==msg->data){
    ROS_INFO("GetGoal!");
    MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 2.822;
	goal.target_pose.pose.position.y = 1.602;
	goal.target_pose.pose.orientation.w = 1.572;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the coffee_table goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	}
	
	}
int main(int argc, char** argv){
	ros::init(argc, argv, "coffee_table");
    Gocofetable a;
    ros::spin();
    ros::shutdown();
	return 0;
}
