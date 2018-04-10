//--------------------------------------------------
//C++常用函数
//
//author: Yutaro ISHIDA
//date: 16/03/14
//--------------------------------------------------


#include <roch_gpsr/common_include.h>
#include <roch_gpsr/common_function.h>


//--------------------------------------------------
//--------------------------------------------------
CommonFunction::CommonFunction(ros::NodeHandle nh_){
    // pub_scan_mode = nh_.advertise<std_msgs::String>("/scan/mode", 1);
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}


//--------------------------------------------------
//--------------------------------------------------
CommonFunction::~CommonFunction(){
}


//--------------------------------------------------
//--------------------------------------------------
void CommonFunction::commonf_actionf_speech_single(ros::NodeHandle nh_, string speech_str){
    actionlib::SimpleActionClient<roch_tts::RochTTSAction> speech_syn_action("roch_tts_action", true);
    speech_syn_action.waitForServer();

    roch_tts::RochTTSGoal goal;
    goal.roch_tts_goal = speech_str;
    speech_syn_action.sendGoal(goal);
    speech_syn_action.waitForResult();
}


//--------------------------------------------------
//--------------------------------------------------
void CommonFunction::commonf_actionf_speech_multi(ros::NodeHandle nh_, string speech_str){
    actionlib::SimpleActionClient<roch_tts::RochTTSAction> speech_syn_action("roch_tts_action", true);
    speech_syn_action.waitForServer();

    roch_tts::RochTTSGoal goal;
    goal.roch_tts_goal = speech_str;
    speech_syn_action.sendGoal(goal);
    // speech_syn_action.waitForResult();
}

//--------------------------------------------------
//--------------------------------------------------
void CommonFunction::commonf_actionf_sound_effect_single(ros::NodeHandle nh_, string sound_effect_goal){
    actionlib::SimpleActionClient<roch_gpsr::SoundEffectAction> sound_effect_action("sound_effect_action", true);
    sound_effect_action.waitForServer();

    roch_gpsr::SoundEffectGoal goal;
    goal.sound_effect_goal = sound_effect_goal;
    sound_effect_action.sendGoal(goal);
    sound_effect_action.waitForResult();
}


//--------------------------------------------------
//--------------------------------------------------
void CommonFunction::commonf_actionf_sound_effect_multi(ros::NodeHandle nh_, string sound_effect_goal){
    actionlib::SimpleActionClient<roch_gpsr::SoundEffectAction> sound_effect_action("sound_effect_action", true);
    sound_effect_action.waitForServer();

    roch_gpsr::SoundEffectGoal goal;
    goal.sound_effect_goal = sound_effect_goal;
    sound_effect_action.sendGoal(goal);
    //sound_effect_action.waitForResult();
}


//--------------------------------------------------
//--------------------------------------------------
void CommonFunction::commonf_pubf_cmd_vel(ros::NodeHandle nh, double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.linear.z = linear_z;
    cmd_vel.angular.x = angular_x;   
    cmd_vel.angular.y = angular_y;
    cmd_vel.angular.z = angular_z;

    pub_cmd_vel.publish(cmd_vel);
}
