/*
* 讯飞离线语音合成服务端
*
* Author: Vance Wu
* Date: 2017-08-14
*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <xfei_tts/XFeiTTSAction.h>

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

typedef int SR_DWORD;
typedef short int SR_WORD ;

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int				size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int				fmt_size;				// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int				samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int				avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int				data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};

class XFeiTTSAction
{
protected:
    ros::NodeHandle nh_;

	actionlib::SimpleActionServer<xfei_tts::XFeiTTSAction> as_;
	std::string action_name_;
	xfei_tts::XFeiTTSResult result_;
	ros::Subscriber sub_;

public:
    XFeiTTSAction(std::string name);
    virtual ~XFeiTTSAction(void);

    void executeCB(const xfei_tts::XFeiTTSGoalConstPtr &goal);
	void PlayWav(const char* cmd);
	void makeTextToWav(const char* text, const char* filename);
	int text_to_speech(const char* src_text, const char* des_path, const char* params);

private:
    /* data */
};

XFeiTTSAction::XFeiTTSAction(std::string name):
    as_(nh_, name, boost::bind(&XFeiTTSAction::executeCB, this, _1), false),
    action_name_(name)
{
    as_.start();
}

XFeiTTSAction::~XFeiTTSAction(void)
{
}

//action_goal回调
void XFeiTTSAction::executeCB(const xfei_tts::XFeiTTSGoalConstPtr &goal)
{
    ros::Rate r(1);
	bool success = true;

	printf("\n%s: 正在执行中...\n", action_name_.c_str());
	printf("%s: 收到一条消息: %s\n", action_name_.c_str(), goal->xfei_tts_goal.c_str());
	
	std::stringstream play_path;
	const char* filename;
	filename = "/home/vance/sawyer_ws/src/roch_GPSR/xfei_tts/src/tts.wav";
    play_path << "play " << filename;
	
	if (as_.isPreemptRequested() || !ros::ok())	{
		printf("%s: Preempted(被占用)...\n", action_name_.c_str());
		as_.setPreempted();
		success = false;
		result_.xfei_tts_result = false;
	}
	
    makeTextToWav(goal->xfei_tts_goal.c_str(), filename); //语音合成
    PlayWav(play_path.str().c_str()); //语音播放

	if (success) 
		result_.xfei_tts_result = true;
	
	as_.setSucceeded(result_);	
}

//音频播放
void XFeiTTSAction::PlayWav(const char* cmd) 
{
    system(cmd);
	printf("%s: 播放完毕...\n", action_name_.c_str());
}


/* 文本合成 */
int XFeiTTSAction::text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)	{
		printf("%s: params is error!\n", action_name_.c_str());
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)	{
		printf("%s: params is error!\n", action_name_.c_str());
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSSessionBegin failed, error code: %d.\n", action_name_.c_str(), ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSTextPut failed, error code: %d.\n", action_name_.c_str(), ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("%s: 正在合成音频...\n", action_name_.c_str());
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) {
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
	}
	printf("\n");
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSAudioGet failed, error code: %d.\n", action_name_.c_str(), ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSSessionEnd failed, error code: %d.\n", action_name_.c_str(), ret);
	}

	return ret;
}


void XFeiTTSAction::makeTextToWav(const char* text, const char* filename)
{
    int         ret          = MSP_SUCCESS;
	const char* login_params = "appid = 5972ada2, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	*/
	const char* session_begin_params = "engine_type = local, text_encoding = UTF8, tts_res_path = fo|/home/vance/sawyer_ws/src/roch_GPSR/xfei_tts/cfg/common.jet;fo|/home/vance/sawyer_ws/src/roch_GPSR/xfei_tts/cfg/xiaoyan.jet, sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 2";
	// const char* filename = "tts_sample.wav"; //合成的语音文件名称
	// const char* text     = "亲爱的用户，您好，这是一个语音合成示例，感谢您对科大讯飞语音技术的支持！科大讯飞是亚太地区最大的语音上市公司，股票代码：002230"; //合成文本
	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
	if (MSP_SUCCESS != ret)	{
		printf("%s: MSP登录失败，错误代码: %d.\n", action_name_.c_str(), ret);
		goto exit ;//登录失败，退出登录
	}

	/* 文本合成 */
	printf("%s: 开始合成 ...\n", action_name_.c_str());
	ret = text_to_speech(text, filename, session_begin_params);
	if (MSP_SUCCESS != ret)
	{
		printf("%s: 合成失败，错误代码: %d.\n", action_name_.c_str(), ret);
	}
	printf("%s: 合成完毕！\n", action_name_.c_str());

exit:
	MSPLogout(); //退出登录
}




int main(int argc, char* argv[])
{
    ros::init(argc, argv, "xfei_tts_server");
	
	XFeiTTSAction xfei_tts("xfei_tts_action");

	ros::spin();
	return 0;
}
