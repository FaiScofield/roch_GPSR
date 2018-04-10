/*--------------------------------------------------
* 从xml文件中提取任务信息
* ROS服务端节点

* author: Vance Wu
* date: 17/08/01
--------------------------------------------------*/

#include <string.h>
#include <tinyxml2.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <roch_gpsr/TaskRecAction.h>

/*全局变量*/
const char * XML_FILE_TASK = "/home/ubuntu/gpsr_ws/src/roch_GPSR/roch_asr/src/asr_cmd.xml";
const char * XML_FILE_CON = "/home/ubuntu/gpsr_ws/src/roch_GPSR/roch_asr/src/task_con.xml";

/*类的声明*/
class TaskRecServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<roch_gpsr::TaskRecAction> as_;
    roch_gpsr::TaskRecResult result_;

    std::string action_name_;
    std::string goal_;
    std::string grammer_;
    int         num_;       // 过滤后的有效id数量
    int         task_[10];  // 存放有效任务id号(可能带有更详细地址信息)

public:
    TaskRecServer(std::string name);
    virtual ~TaskRecServer(void);

    void executeCB(const roch_gpsr::TaskRecGoalConstPtr &goal);
    bool get_cmd_from_xml();
    bool get_confirm_from_xml();
    bool send_cmd();

private:
    
};

TaskRecServer::TaskRecServer(std::string name):
    as_(nh_, name, boost::bind(&TaskRecServer::executeCB, this, _1), false),
	action_name_(name)
{
    as_.start();
}

TaskRecServer::~TaskRecServer(void)
{
}

void TaskRecServer::executeCB(const roch_gpsr::TaskRecGoalConstPtr &goal)//传入grammer
{
    ros::Rate r(1);
    
    grammer_ = goal->task_rec_goal;
    printf("\n%s: 接收到一个目标(goal),语法为:%s\n", action_name_.c_str(), grammer_.c_str());

    // ros::param::get("/param/gpsr/grammer", grammer_);
    bool success;

    if (grammer_ == "confirm") {
        printf("%s: 检测到语法文件为:confirm\n", action_name_.c_str());
        success = this->get_confirm_from_xml();
    }
    else { //若传入的不是'confirm',则全当做'mission'
        printf("%s: 检测到语法文件为:mission\n", action_name_.c_str());
        success = this->get_cmd_from_xml();
    }
    if (success) {
        printf("%s: 任务已经解析成功...\n", action_name_.c_str());
        result_.task_rec_result = true;
        as_.setSucceeded(result_);
    }
    else {
        printf("%s: 任务解析指令失败...\n", action_name_.c_str());
        result_.task_rec_result = false;
        as_.setAborted(result_);
    }

    if (grammer_ != "confirm" && this->send_cmd()) // 识别情况才执行send_cmd(),确认情况已经设置好参数了
        printf("%s: 解析后的任务已存入参数服务器中...\n", action_name_.c_str());
    
    printf("%s: 本次任务结束,等待下一个目标传入...\n", action_name_.c_str());
}

bool TaskRecServer::get_cmd_from_xml()
{
    printf("%s: 进入到get_cmd_from_xml程序,正在解析任务...\n", action_name_.c_str());
    tinyxml2::XMLDocument xml_doc;

    int res = xml_doc.LoadFile(XML_FILE_TASK); //成功返回0
    if (res != 0){
        printf("%s: 加载 xml 文件失败...\n", action_name_.c_str());
        return false;
    }

    tinyxml2::XMLElement* root_ele = xml_doc.RootElement(); 
    tinyxml2::XMLElement* rawtext_ele = root_ele->FirstChildElement("rawtext");
    printf("%s: 识别到的句子是:%s\n", action_name_.c_str(), rawtext_ele->GetText());
    ros::param::set("/param/gpsr/task/text", rawtext_ele->GetText());
    tinyxml2::XMLElement* result_ele = root_ele->FirstChildElement("result");
    tinyxml2::XMLElement* object_ele = result_ele->FirstChildElement("object");

    num_ = 0;
    for (tinyxml2::XMLElement* object_item = object_ele->FirstChildElement(); object_item != NULL; object_item = object_item->NextSiblingElement()){
        task_[num_] = object_item->FirstAttribute()->IntValue(); // 读取参数中的int值,即xml中的id
        if (task_[num_] != 65535){
            printf("%s: task_[%i]的ID是:%i.\n", action_name_.c_str(), num_, task_[num_]);
            num_++;//num_目前值为有效id数量
        }
    }

    return true;
}

bool TaskRecServer::get_confirm_from_xml()
{
    printf("%s: 进入到get_confirm_from_xml程序中,正在对获取到的任务进行确认...\n", action_name_.c_str());
    tinyxml2::XMLDocument xml_doc;

    int res = xml_doc.LoadFile(XML_FILE_CON); //成功返回0
    if (res != 0){
        printf("%s: 加载 xml 文件失败...\n", action_name_.c_str());
        return false;
    }

    tinyxml2::XMLElement* root_ele = xml_doc.RootElement(); 
    tinyxml2::XMLElement* rawtext_ele = root_ele->FirstChildElement("rawtext");
    printf("%s: 识别到的句子是:%s\n", action_name_.c_str(), rawtext_ele->GetText());
    // ros::param::set("/param/gpsr/task/confirm", rawtext_ele->GetText());
    tinyxml2::XMLElement* result_ele = root_ele->FirstChildElement("result");
    tinyxml2::XMLElement* object_ele = result_ele->FirstChildElement("object");
    tinyxml2::XMLElement* object_item = object_ele->FirstChildElement();
    
    int conf = object_item->FirstAttribute()->IntValue();
    if (conf == 11){ //yes=11 
        printf("%s: 所有任务得到确定,设置参数/param/gpsr/task/confirm为yes\n", action_name_.c_str());
        ros::param::set("/param/gpsr/task/confirm", "yes");
    }
    else if (conf == 10){ //no=10
        printf("%s: 不能确定所有任务,设置参数为no.\n", action_name_.c_str());
        ros::param::set("/param/gpsr/task/confirm", "no");
    }
    else { // 没有识别到yes or no
        printf("%s: 没有识别到yes or no,设置参数为error.\n", action_name_.c_str());
        ros::param::set("/param/gpsr/task/confirm", "error");
    }

    return true;
}



bool TaskRecServer::send_cmd()
{
    int num, cmd[num_];
    for (int i = 0; i < 9; i++) {
        cmd[i] = task_[i];
    }
    num = num_;

    // task1
    // 初始化参数
    ros::param::set("/param/gpsr/task1/pass", false);
    ros::param::set("/param/gpsr/task1/move", false);
    ros::param::set("/param/gpsr/task1/location/match", false);
    ros::param::set("/param/gpsr/task1/location/detail", 0);
    // 判断第一个id是否是移动(id:101)
    if (cmd[0] == 101) 
        ros::param::set("/param/gpsr/task1/move", true);
    else {//第1个id不是移动,跳过task1
        ros::param::set("/param/gpsr/task1/move", false);
        ros::param::set("/param/gpsr/task1/pass", true);
    }
    // 判断第2个和第3个id是否是地点(id:1100-1600),并保留一个更详细的地点
    //第2个id是地点的情况
    if (cmd[1] >= 1100 && cmd[1] <= 1600){
        ros::param::set("/param/gpsr/task1/location/detail", cmd[1]);
        ros::param::set("/param/gpsr/task1/location/match", true);
        // 第3个id也是地址的情况
        if ((cmd[2] >= 1100 && cmd[2] <= 1600)){
            //具体地点和房间地址一致,设目标位置为具体地点位置,并删除房间地点
            if (cmd[2]-cmd[1] < 10 && cmd[2]-cmd[1] > 0){ 
                ros::param::set("/param/gpsr/task1/location/detail", cmd[2]);
                for (int i = 1; i < num-1; i++) 
                    cmd[i] = cmd[i+1]; 
                num -= 1;
            }
            else //具体地点和房间地址不一致
                // ros::param::set("/param/gpsr/task1/location/match", false);
                ros::param::set("/param/gpsr/task1/location/match", true); //不一致就先到这个房间
        }//只有第2个id是地点,不处理第3个id
    }
    else {//第2个id不是地点,无法移动,跳过task1
        ros::param::set("/param/gpsr/task1/location/match", false);
        ros::param::set("/param/gpsr/task1/pass", true);
    }


    // task2
    // 初始化参数
    ros::param::set("/param/gpsr/task2/pass", false);
    ros::param::set("/param/gpsr/task2/move", false);
    ros::param::set("/param/gpsr/task2/location/match", false);
    ros::param::set("/param/gpsr/task2/location/detail", 0);
    ros::param::set("/param/gpsr/task2/grasp", false);
    ros::param::set("/param/gpsr/task2/find", false);
    ros::param::set("/param/gpsr/task2/count", false);
    ros::param::set("/param/gpsr/task2/item/detail", 0);
    ros::param::set("/param/gpsr/task2/item/exist", false);
    // task2,第3个id是移动的情况
    if (cmd[2]==101 | cmd[2]==102) {
        ros::param::set("/param/gpsr/task2/move", true);
        //第4个id是地点的情况
        if (cmd[3] >= 1100 && cmd[3] <= 1600){
            ros::param::set("/param/gpsr/task2/location/match", true);
            ros::param::set("/param/gpsr/task2/location/detail", cmd[3]);
            // 第5个id也是地点的情况
            if ((cmd[4] >= 1100 && cmd[4] <= 1600)){
                //具体地点和房间地址一致,设目标位置为具体地点位置,并删除房间地点
                if (cmd[4]-cmd[3] < 10 && cmd[4]-cmd[3] > 0){ 
                    ros::param::set("/param/gpsr/task2/location/detail", cmd[4]);
                    for (int i = 3; i < num-1; i++) 
                        cmd[i] = cmd[i+1]; 
                    num -= 1;
                }
                else //具体地点和房间地址不一致
                    // ros::param::set("/param/gpsr/task2/location/match", false);
                    ros::param::set("/param/gpsr/task2/location/match", true); //不一致就先到这个房间
            } //只有第4个id是地点,不处理第5个id

        }
        else {//第4个id不是地点,无法移动,跳过task2
            ros::param::set("/param/gpsr/task2/location/match", false);
            ros::param::set("/param/gpsr/task2/pass", true);
        }
    }
    // task2,第3个id是抓,找,数东西(人)的情况
    else if (cmd[2]==201 | cmd[2]==202 | cmd[2]==203){
        // 第4个id是物品(人)
        if (cmd[3] >= 2000) {
            ros::param::set("/param/gpsr/task2/item/detail", cmd[3]);
            ros::param::set("/param/gpsr/task2/item/exist", true);
        }
        else {// 第4个id不是物品(人),跳过task2
            ros::param::set("/param/gpsr/task2/item/exist", false);
            ros::param::set("/param/gpsr/task2/pass", true);
        }
        // 对第3个id进行具体分析
        if (cmd[2] == 201) //抓取东西
            ros::param::set("/param/gpsr/task2/grasp", true);
        else if (cmd[2] == 202) //找东西(人)
            ros::param::set("/param/gpsr/task2/find", true);
        else if (cmd[2] == 203) //数东西(人)
            ros::param::set("/param/gpsr/task2/count", true);
    }
    // task2,第3个id不符合任务的情况
    else ros::param::set("/param/gpsr/task2/pass", true);


    // task3
    // 初始化参数
    ros::param::set("/param/gpsr/task3/enable", true);
    ros::param::set("/param/gpsr/task3/move", false);
    ros::param::set("/param/gpsr/task3/location/match", false);
    ros::param::set("/param/gpsr/task3/location/detail", 0);
    ros::param::set("/param/gpsr/task3/lead", false);
    ros::param::set("/param/gpsr/task3/deliver", false); 
    ros::param::set("/param/gpsr/task3/put", false); 
    ros::param::set("/param/gpsr/task3/tell", false);
    ros::param::set("/param/gpsr/task3/give", false);
    // ros::param::set("/param/gpsr/task3/pass", false); // 不设置跳过task3

    // task3,第5个id是移动的情况(移动到,离开场地,运送物品,放置物品)
    if (cmd[4]==101 | cmd[4]==102 | cmd[4]==103 | cmd[4]==104 | cmd[4]==105) {
        // 初始化参数
        ros::param::set("/param/gpsr/task3/move", true);

        //第6个id是地点的情况
        if (cmd[5] >= 1100 && cmd[5] <= 1600){
            ros::param::set("/param/gpsr/task3/location/match", true);
            ros::param::set("/param/gpsr/task3/location/detail", cmd[5]);
            // 第7个id也是地点的情况(注意后项前移的情况)
            if ((cmd[6] >= 1100 && cmd[6] <= 1600)){
                //具体地点和房间地址一致,设目标位置为具体地点位置,并删除房间地点
                if (cmd[6]-cmd[5] < 10 && cmd[6]-cmd[5] >= 0){ // 用 >= 是因为删除位置时只把后项前移,最后的项目并没有被删除
                    ros::param::set("/param/gpsr/task3/location/detail", cmd[6]);
                    for (int i = 5; i < num-1; i++) 
                        cmd[i] = cmd[i+1]; 
                    // num -= 1;    //这里最后不用减?
                }
                else //具体地点和房间地址不一致
                    // ros::param::set("/param/gpsr/task3/location/match", false);
                    ros::param::set("/param/gpsr/task3/location/match", true); //不一致就先到这个房间
            } //只有第6个id是地点,不处理第7个id

        }
        else {//第6个id不是地点,注意可能是带人离开,不能跳过任务
            ros::param::set("/param/gpsr/task3/location/match", false);
            // ros::param::set("/param/gpsr/task3/pass", true);
        }

        //移动情况的更详细判断
        if (cmd[4] == 102)      // 返回原地
            ros::param::set("/param/gpsr/task3/location/detail", 1500);
        if (cmd[4] == 103)      // 带人走
            ros::param::set("/param/gpsr/task3/lead", true);
        if (cmd[4] == 104)      // 运送
            ros::param::set("/param/gpsr/task3/deliver", true); 
        if (cmd[4] == 105)      // 放下物品
            ros::param::set("/param/gpsr/task3/put", true); 
    }
    // task3,第5个id是其他情况
    else if (cmd[4] == 301)     //找自我介绍
        ros::param::set("/param/gpsr/task3/tell", true);
    else if (cmd[4] == 302) {   //拿给裁判
        ros::param::set("/param/gpsr/task3/give", true);
        ros::param::set("/param/gpsr/task3/move", true);
        ros::param::set("/param/gpsr/task3/location/match", true);
        ros::param::set("/param/gpsr/task3/location/detail", 1500);
    }
    else if (cmd[4] == 303) {   //离开
        ros::param::set("/param/gpsr/task3/move", true);
        ros::param::set("/param/gpsr/task3/location/match", true);
        ros::param::set("/param/gpsr/task3/location/detail", 1600);
    }
    else ros::param::set("/param/gpsr/task3/enable", false);
    
    
    for (int j=0; j<num; j++)
        printf("%s: cmd[%i]的ID是:%i.\n", action_name_.c_str(), j, cmd[j]);

    return true;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "task_rec_server");
    
    TaskRecServer task_rec_server("task_rec_action");

    ros::spin();

    return 0;
}
