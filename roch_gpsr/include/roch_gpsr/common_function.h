#include <roch_gpsr/common_include.h>


class CommonFunction{
private:
    // ros::Publisher pub_scan_mode;

    // ros::Publisher pub_cam_pan_angle;
    // ros::Publisher pub_cam_tilt_angle;

    // ros::Publisher pub_mic_pan_angle;
    // ros::Publisher pub_mic_tilt_angle;

    ros::Publisher pub_cmd_vel;


public:
    // 构造函数
    CommonFunction(ros::NodeHandle);

    // 析构函数
    ~CommonFunction();

    void commonf_actionf_speech_single(ros::NodeHandle, string);

    void commonf_actionf_speech_multi(ros::NodeHandle, string);

    void commonf_actionf_sound_effect_single(ros::NodeHandle, string);

    void commonf_actionf_sound_effect_multi(ros::NodeHandle, string);

    // void commonf_actionf_cam_lift(ros::NodeHandle, double);

    // void commonf_pubf_scan_mode(ros::NodeHandle, string);

    // void commonf_pubf_cam_pan(ros::NodeHandle, double);

    // void commonf_pubf_cam_tilt(ros::NodeHandle, double);

    // void commonf_pubf_mic_pan(ros::NodeHandle, double);

    // void commonf_pubf_mic_tilt(ros::NodeHandle, double);

    void commonf_pubf_cmd_vel(ros::NodeHandle, double, double, double, double, double, double);
};
