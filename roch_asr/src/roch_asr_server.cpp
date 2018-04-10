#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>


#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <roch_asr/RochASRAction.h>

#include <tinyxml2.h>


#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096
#define SAMPLE_RATE_16K     (16000) //离线识别只支持16K
//#define SAMPLE_RATE_8K      (8000)
#define MAX_GRAMMARID_LEN   (32)
#define MAX_PARAMS_LEN      (1024)

//注意,ASR_RES_PATH路径前的'fo|'必须要有
const char * ASR_RES_PATH   = "fo|/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/cfg/common.jet";  //离线语法识别资源路径
const char * GRM_BUILD_PATH = "/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/build/GrmBuilld"; //构建离线语法识别网络生成数据保存路径
const char * GRM_FILE_TASK  = "/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/cfg/mission.bnf"; //构建离线识别语法网络所用的语法文件
const char * GRM_FILE_CON   = "/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/cfg/confirm.bnf";
const char * LEX_NAME       = "item";               //更新离线识别语法的item槽
const char * XML_FILE_TASK  = "/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/src/asr_cmd.xml"; //识别并解析后的关键资料存放
const char * XML_FILE_CON   = "/home/vance/sawyer_ws/src/roch_GPSR/roch_asr/src/task_con.xml";
const char * XML_FILE;
const char * GRM_FILE;

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

bool flag_success = false; //判断命令词是否被识别出的标志

typedef struct _UserData {
    int     build_fini;     //标识语法构建是否完成
    int     update_fini;    //标识更新词典是否完成
    int     errcode;        //记录语法构建或更新词典回调错误码
    char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;

class RochASRAction
{
protected:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<roch_asr::RochASRAction> as_;
    std::string action_name_;
    roch_asr::RochASRFeedback feedback_;
    roch_asr::RochASRResult result_;
    ros::Subscriber sub_;

public:
    RochASRAction(std::string name);
    virtual ~RochASRAction();
    
    void executeCB(const roch_asr::RochASRGoalConstPtr &goal);
    bool asr_record();

    int build_grammar(UserData *udata); //构建离线识别语法网络
    // int build_grm_cb(int ecode, const char *info, void *udata);

    int update_lexicon(UserData *udata);//更新离线识别语法词典
    // int update_lex_cb(int ecode, const char *info, void *udata);

    int run_asr(UserData *udata);       //进行离线语法识别
    void open_mic(const char* session_begin_params);
    // void on_speech_begin();
    // void on_speech_end(int reason);
    // void on_result(const char *result, char is_last);
    // void show_result(char *string, char is_over);

private:
    int         recording_time;
    std::string grammer_;   // 存储传入的goal的值,传入指定的语法文件
};

/*******************
* 非类成员函数定义
********************/
void show_result(char *string, char is_over)
{
    printf("\nResult: [ %s ]", string);
    if(is_over)
        putchar('\n');

    //解析语义中的XML信息并保存成文件
    tinyxml2::XMLDocument xd;
    xd.Parse(string);
    xd.SaveFile(XML_FILE);

    printf("\nroch_asr_action: 识别结果已保存至/src文件夹中...\n");
    
    flag_success = true;    // 仅在识别出结果时才将标志位置true
}

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("roch_asr_action: mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

void on_speech_begin()
{
    if (g_result) {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("roch_asr_action: Start Listening...\n");
}

void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nroch_asr_action: Speaking done... \n");
    else
        printf("\nroch_asr_action: Recognizer error... %d\n", reason);
}

int build_grm_cb(int ecode, const char *info, void *udata)
{
    UserData *grm_data = (UserData *)udata;

    if (NULL != grm_data) {
        grm_data->build_fini = 1;
        grm_data->errcode = ecode;
    }
     
    if (MSP_SUCCESS == ecode && NULL != info) {
        printf("roch_asr_action: 构建语法成功！ 语法ID:%s\n", info);
        if (NULL != grm_data)
            snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
    }
    else
        printf("roch_asr_action: 构建语法失败！错误代码:%d\n", ecode);

    return 0;
}


/*******************
* 类成员函数定义
********************/
RochASRAction::RochASRAction(std::string name):
    as_(nh_, name, boost::bind(&RochASRAction::executeCB, this, _1), false),
    action_name_(name),
    recording_time(15)
    {
        as_.start();
    }

RochASRAction::~RochASRAction(void)
{
}

int RochASRAction::build_grammar(UserData *udata)
{
    FILE *grm_file                          = NULL;
    char *grm_content                       = NULL;
    unsigned int grm_cnt_len                = 0;
    char grm_build_params[MAX_PARAMS_LEN]   = {NULL};
    int ret                                 = 0;

    // 根据参数情况选择要使用的语法
    // ros::param::get("/param/gpsr/grammer", grammer_);//返回"confirm"或"mission"(或"NULL")
    if (grammer_ == "confirm") {
        GRM_FILE = GRM_FILE_CON;
        XML_FILE = XML_FILE_CON;
    }
    else {
        GRM_FILE = GRM_FILE_TASK;
        XML_FILE = XML_FILE_TASK;
    }
    
    grm_file = fopen(GRM_FILE, "rb");
    if(NULL == grm_file) {
        printf("%s: 打开\"%s\"文件失败！[%s]\n", action_name_.c_str(), GRM_FILE, strerror(errno));
        return -1; 
    }
    printf("%s: bnf语法文件存在,打开成功...\n", action_name_.c_str(), action_name_.c_str());
    
    fseek(grm_file, 0, SEEK_END);
    grm_cnt_len = ftell(grm_file);
    fseek(grm_file, 0, SEEK_SET);

    grm_content = (char *)malloc(grm_cnt_len + 1);
    if (NULL == grm_content)
    {
        printf("%s: 内存分配失败!\n", action_name_.c_str());
        fclose(grm_file);
        grm_file = NULL;
        return -1;
    }
    fread((void*)grm_content, 1, grm_cnt_len, grm_file);
    grm_content[grm_cnt_len] = '\0';
    fclose(grm_file);
    grm_file = NULL;

    snprintf(grm_build_params, MAX_PARAMS_LEN - 1, 
        "engine_type = local, \
        asr_res_path = %s, sample_rate = %d, \
        grm_build_path = %s, ",
        ASR_RES_PATH,
        SAMPLE_RATE_16K,
        GRM_BUILD_PATH
        );
    ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

    free(grm_content);
    grm_content = NULL;

    return ret;
}

int RochASRAction::run_asr(UserData *udata)
{
    char asr_params[MAX_PARAMS_LEN]    = {NULL};
    const char *rec_rslt               = NULL;
    const char *session_id             = NULL;
    const char *asr_audiof             = NULL;
    FILE *f_pcm                        = NULL;
    char *pcm_data                     = NULL;
    long pcm_count                     = 0;
    long pcm_size                      = 0;
    int last_audio                     = 0;

    int aud_stat                       = MSP_AUDIO_SAMPLE_CONTINUE;
    int ep_status                      = MSP_EP_LOOKING_FOR_SPEECH;
    int rec_status                     = MSP_REC_STATUS_INCOMPLETE;
    int rss_status                     = MSP_REC_STATUS_INCOMPLETE;
    int errcode                        = -1;
    int aud_src                        = 0;
    //离线语法识别参数设置
    snprintf(asr_params, MAX_PARAMS_LEN - 1, 
        // type:local, mixed..
        "engine_type = local, \
        asr_res_path = %s, sample_rate = %d, \
        grm_build_path = %s, local_grammar = %s, \
        result_type = xml, result_encoding = UTF-8, ",  //plain, json, xml
        ASR_RES_PATH,
        SAMPLE_RATE_16K,
        GRM_BUILD_PATH,
        udata->grammar_id
        );
    
    open_mic(asr_params);

    return 0;
}

/* demo recognize the audio from microphone */
void RochASRAction::open_mic(const char* session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("%s: speech recognizer init failed\n", action_name_.c_str());
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("%s: start listen failed %d\n", action_name_.c_str(), errcode);
    }
    /* demo 15 seconds recording */
    while(i++ < recording_time)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("%s: stop listening failed %d\n", action_name_.c_str(), errcode);
    }
    
    if (flag_success) {
        result_.roch_asr_result  = true;
        printf("%s: 命令词识别成功...\n", action_name_.c_str());
    }
    else {
        result_.roch_asr_result = false;
        printf("%s: 命令词识别失败...\n", action_name_.c_str());
    }
    
    as_.setSucceeded(result_);

    sr_uninit(&iat);

}

void RochASRAction::executeCB(const roch_asr::RochASRGoalConstPtr &goal) //传入goal(string类型),指定grammer文件
{
    ros::Rate r(1);
    bool success = false;
    flag_success = false; //每次开启识别时先初始化识别成功的标志位,防止第一次识别成功第二次识别失败时,被判断成识别成功
    grammer_ = goal->roch_asr_goal;

    printf("\n%s: 语音识别正在执行中...\n", action_name_.c_str());
    printf("%s: 获取到一个客户目标,语法文件为: %s\n", action_name_.c_str(), grammer_.c_str());

    if (as_.isPreemptRequested() || !ros::ok())	{
        printf("%s: Preempted...\n", action_name_.c_str());
        as_.setPreempted();
        success = false;
    }
    
    //执行录音+命令词识别
    this->asr_record();

}

bool RochASRAction::asr_record()
{
    const char *login_config    = "appid = 5972ada2";
    UserData asr_data; 
    int ret                     = 0 ;
    char c;

    ret = MSPLogin(NULL, NULL, login_config);
    if (MSP_SUCCESS != ret) {
        printf("%s: 登录失败：%d\n", action_name_.c_str(), ret);
        goto exit;
    }
    printf("%s: 登录成功...\n", action_name_.c_str());
    
    memset(&asr_data, 0, sizeof(UserData));
    printf("%s: 构建离线识别语法网络...\n", action_name_.c_str());
    ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
    if (MSP_SUCCESS != ret) {
        printf("%s: 构建语法调用失败！\n", action_name_.c_str());
        goto exit;
    }
    while (1 != asr_data.build_fini)
        usleep(300 * 1000);
    if (MSP_SUCCESS != asr_data.errcode)
        goto exit;
    printf("%s: 离线识别语法网络构建完成，开始识别...\n", action_name_.c_str());	

    ret = run_asr(&asr_data);
    if (MSP_SUCCESS != ret) {
        printf("%s: 离线语法识别出错: %d \n", action_name_.c_str(), ret);
        goto exit;
    }

    MSPLogout();
    printf("%s: 本次任务完成,已退出登录...\n", action_name_.c_str());
    return true;

exit:
    MSPLogout();
    printf("%s: 请按任意键退出...\n", action_name_.c_str());
    //getchar();
    return false;
}

int main(int argc, char* argv[])
{
    // ros::init(argc, argv, 'rosch_asr');
    ros::init(argc, argv, "roch_asr_server");
    
    RochASRAction roch_asr("roch_asr_action");

    ros::spin();

    return 0;
}
