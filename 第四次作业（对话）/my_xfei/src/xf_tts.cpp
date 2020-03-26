#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "/home/ljc/catkin_ws/src/my_xfei/include/qtts.h"
#include "/home/ljc/catkin_ws/src/my_xfei/include/msp_cmn.h"
#include "/home/ljc/catkin_ws/src/my_xfei/include/msp_errors.h"
#include "/home/ljc/catkin_ws/src/my_xfei/include/speech_recognizer.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

// #define SAYIT  system("cp  /Robot/voice/wav/say.wav /Robot/voice/wav/temp.wav>/Robot/cmd/Mplayer_cmd");system("echo loadfile /Robot/voice/wav/temp.wav>/Robot/cmd/Mplayer_cmd")
// typedef int SR_DWORD;
// typedef short int SR_WORD ;
bool wakeupFlag = false;
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int             size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int             fmt_size;               // = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int             samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int             avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int             data_size;              // = 纯数据长度 : FileSize - 44 
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
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path)
    {
        printf("params is error!\n");
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp)
    {
        printf("open %s error.\n", des_path);
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1) 
    {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
    }//合成状态synth_status取值请参阅《讯飞语音云API文档》
    printf("\n");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
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
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);
    }
    return ret;
}


void xfcallback(const std_msgs::String::ConstPtr& msg)
{
  char cmd[2000];
  const char* text;
  const char* answer;
  int         ret                  = MSP_SUCCESS;
  const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
  const char* filename             = "tts_sample.wav"; //合成的语音文件名称
  

  std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;
  text = msg->data.c_str();
//--------------
// // 发布语音识别结果的信号    
//   ros::NodeHandle nh;
//   ros::Publisher wakeUpSub = nh.advertise<std_msgs::Bool>("voiceWakeup", 1000); 
  if (msg->data.find("自我介绍")>-1)
{
            answer="我是机器人，用于聊天。";
}
else if (msg->data.find("爱好")>-1)
{
            answer="我喜欢聊天和打扫卫生。你呢";
}
else if (msg->data.find("年龄")>-1)
{
            answer="我三岁了。";
}
else if (msg->data.find("家乡")>-1)
{
            answer="我来自中国。";
}
else if (msg->data.find("动物")>-1)
{
            answer="我喜欢狗，不喜欢猫。";
}
else if (msg->data.find("故事")>-1)
{
            answer="给你讲个三只小猪吧，从前有三只猪，被狼吃了。";
}
else if (msg->data.find("hobby")>-1)
{
            answer="我喜欢聊天和打扫卫生。你呢";
}
else if (msg->data.find("age")>-1)
{
            answer="我三岁了。";
}
else if (msg->data.find("hometown")>-1)
{
            answer="我来自中国。";
}
else if (msg->data.find("animal")>-1)
{
            answer="我喜欢狗，不喜欢猫。";
}
else if (msg->data.find("story")>-1)
{
            answer="给你讲个三只小猪吧，从前有三只猪，被狼吃了。";
}
else
{
            answer="嗯嗯，我知道了。";
}


//--------
//  text = msg->data.c_str(); 
  /* 文本合成 */
  printf("开始合成 ...\n");
//停止接听
//   wakeUpSub.publish(false);
  ret = text_to_speech(answer, filename, session_begin_params);
  if (MSP_SUCCESS != ret)
  {
       printf("text_to_speech failed, error code: %d.\n", ret);
  }
  printf("合成完毕\n");


  unlink("/tmp/cmd");  
  mkfifo("/tmp/cmd", 0777);  
  popen("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'","r");
  sleep(15);
  printf("Mplayer Run Success\n");
//恢复接听
  wakeupFlag = true;
//   wakeUpSub.publish(true);

}

void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}

int main(int argc, char* argv[])
{

    // 声明Publisher和Subscriber
    // 订阅唤醒语音识别的信号
    //ros::Subscriber wakeUpSub =         n.subscribe("voiceWakeup", 1000, WakeUp);   
    // 发布语音识别结果开启的信号   
        // 初始化ROS
    ros::init(argc,argv,"xf_tts");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); 
    // ros::Publisher wakeUpSub = n.advertise<std_msgs::Bool>("voiceWakeup", 1000); 
    sleep(10);
    ros::Subscriber sub =n.subscribe("voiceWords",1000,xfcallback);

	int         ret                  = MSP_SUCCESS;
	const char* login_params         = "appid = 58249817, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/

	/* 用户登录 */
    while(ros::ok())
    {
        // 语音识别唤醒
		// printf(wakeupFlag);
        if (!wakeupFlag){
            ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
            if (MSP_SUCCESS != ret)
            {
                printf("MSPLogin failed, error code: %d.\n", ret);
                /*goto exit ;*///登录失败，退出登录
                toExit();
            }
            printf("\n###########################################################################\n");
            printf("## 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的 ##\n");
            printf("## 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的  ##\n");
            printf("## 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。  ##\n");
            printf("###########################################################################\n\n");   

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
	


exit:
	printf("按任意键退出 ...\n");
	getchar();
	MSPLogout(); //退出登录

	return 0;
}




// int xf_tts(const char* text,const char *filename)
// {
//     int         ret                  = MSP_SUCCESS;
//     const char* login_params         = "appid = 58249817, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
//     const char* session_begin_params = "engine_type =local, text_encoding = UTF8, tts_res_path = fo|/Robot/voice/bin/msc/res/tts/xiaoyan.jet;fo|/Robot/voice/bin/msc/res/tts/common.jet, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
//     /* 用户登录 */
//     ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
//     if (MSP_SUCCESS != ret)
//     {
//         printf("MSPLogin failed, error code: %d.\n", ret);
//         goto exit ;//登录失败，退出登录
//     }
//     /* 文本合成 */
//     printf("开始合成 ...\n");
//     ret = text_to_speech(text, filename, session_begin_params);
//     if (MSP_SUCCESS != ret)
//     {
//         printf("text_to_speech failed, error code: %d.\n", ret);
//     }
//     printf("合成完毕\n");

// exit:
//     MSPLogout(); //退出登录
//     return 0;
// }
// void xfcallback(const std_msgs::String::ConstPtr& msg)
// {
//   char cmd[2000];
//   std::cout<<"I heard,I will say:"<<msg->data.c_str()<<std::endl;
//   xf_tts(msg->data.c_str(),"/Robot/voice/wav/say.wav");
//   sprintf(cmd,"echo %s>/Robot/cmd/saywords",msg->data.c_str());
//   popen(cmd,"r");
//   SAYIT;
// }
// int main(int argc,char **argv)
// {
//     unlink("/Robot/cmd/Mplayer_cmd");
//     mkfifo("/Robot/cmd/Mplayer_cmd", 0777);
//     popen("mplayer -quiet -slave -input file=/Robot/cmd/Mplayer_cmd -idle","r");
//     printf("Mplayer Run Success");
//     const char* filename        = "/Robot/voice/wav/say.wav"; //合成的语音文件名称
//     const char* text                 = "语音合成模块启动成功！"; //合成文本
//     xf_tts(text,filename);
//     SAYIT;
//     ros::init(argc,argv,"xf_tts");
//     ros::NodeHandle n;
//     ros::Subscriber sub =n.subscribe("xfsaywords",1000,xfcallback);
//     ros::spin();
//     return 0;
// }
