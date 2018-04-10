--------------------
##### For Roch GPSR
#####
##### Author: Vance Wu
##### Date: 2017.08.10
--------------------

    [Roch机器人](http://wiki.ros.org/roch_robot)参加2017年中国机器人大赛的代码，时间有限，仅完成部分工作。目前已实现的功能有：建图，导航，语音识别（asr），语音合成(tts)，任务理解，ROS状态机框架，人物跟随等。未完成部分：物体识别，物体抓取，人物识别等。
    
### 各功能包说明:
+ roch_asr: 命令词识别(离线)
+ roch_gpsr: GPSR任务框架和执行代码
+ roch_tts: 语音合成(在线)
+ xfei_tts: 语音合成(离线)

### 移植程序需要修改的地方:
* 1)先将roch_asr, roch_gpsr, roch_tts或xfei_tts各包中CMakeList.txt文件中的<b>添加可执行文件部分</b>(add_executable, target_link_libraries, add_dependencies)注释,先行编译一次,使Action的头文件得以生成.

* 2)将三个包中的CMakeList.txt文件及<b>可执行文件的源文件</b>中的路径,改为自己电脑上对应的路径.

* 3)讯飞语音的离线版功能包只有35天的体验期,过期后需要申请新的账号并下载新的离线功能包SDK.

* 4)将roch_asr和roch_tts或xfei_tts中可执行文件源代码中的appid改为自己的appid,将包内lib子文件夹中的libmsc.so文件用自己下载的SDK中的libmsc.so文件替换.

* 5)再次编译,即可使用.

* 需要安装的依赖包: mpayer, sox, tinyxml2等
  > sudo apt-get install mplayer sox libtinyxml2-dev
  
* 可能出现的错误及解决方法:
  - 编译出错：检查第1)步是否已生成Action的头文件; 检查和第2)步是否将所有的源文件和CMakeList.txt文件中的绝对路径是否已修改．
  - 使用出错：查看错误提示，到讯飞论坛中查看错误代码对应的错误．常见错误有：
  - 文件找不到: 检查第2)步
  - 语法文件出错: 检查语法文件书写格式是否正确
  - 登录失败:检查第3)步和第4)步
  - 库和ID不匹配:检查第4)步
    
### 执行指令
#### 远程登录
> ssh ubuntu@192.168.0.116    #sawyer-wifi

> ssh ubuntu@10.42.0.1        #hot-roch
 
#### 赛前建图
> roslaunch roch_gpsr roch_build_map.launch

> rosrun rosbag record /scan /tf

> rosrun map_server map_saver -f gpsr.yaml

#### 比赛开始
#### 开启Roch和xtion(ssh)
> roslaunch roch_gpsr roch_bringup.launch --screen

#### 开启GPSR(ssh)
> roslaunch roch_gpsr gpsr_server.launch

> roslaunch roch_gpsr gpsr_smach.launch

#### 远程监控
> roslaunch roch_viz view_navigation.launch --screen

#### 开启RealSense(ssh)
> roslaunch realsense_camera sr300_nodelet_rgbd.launch

### 注:
#### 参考网址：
[ROS语音交互——科大讯飞语音合成TTS（二）](http://www.cnblogs.com/CZM-/p/6204233.html)

[ROS语音交互（三）科大讯飞语音在ROS平台下使用](http://www.cnblogs.com/CZM-/p/6208415.html)

[ROS语音交互（四）接入图灵语义理解](http://www.cnblogs.com/CZM-/p/6211038.html)

[讯飞官方论坛,常见问题解答---常见错误码&问题描述](http://bbs.xfyun.cn/forum.php?mod=viewthread&tid=13056&extra=page%3D1)

[讯飞官方论坛,开放语义 --- 语法相关问题和答案汇总](http://bbs.xfyun.cn/forum.php?mod=viewthread&tid=24520&extra=page%3D1)

[讯飞官方论坛,识别语法分享--在线语法和离线语法编写指南！](http://bbs.xfyun.cn/forum.php?mod=viewthread&tid=7595&fromuid=44990)

[ROS+科大讯飞语音=让你的机器人能听会说系列](https://blog.csdn.net/zhouge94/article/details/52028698)

[ROS(indigo)语音工具 科大讯飞 百度 pocketsphinx julius rospeex 16.11.22更新 ROS中文语音](https://blog.csdn.net/zhangrelay/article/details/53022494)

#### 一些相关的github项目：
[官方github](https://github.com/RoboCupAtHome)

[turtlebot](https://github.com/FansaOrz/robocup-home_package_turtlebot)

[指令生成器](https://github.com/kyordhel/GPSRCmdGen/tree/GermanOpen2017)

[2015年日本队伍的代码](https://github.com/hibikino-musashi-athome),本项目框架的主要参考

#### 建图过程用RPlidar,保持以下环境变量:
> export ROCH_LASER=rplidar
> export ROCH_LASER_ENABLE=true
> export ROCH_3D_SENSOR=false
> export ROCH_3D_SENSOR_NAV_ENABLE=false
#### 比赛过程不使用RPlidar,保持以下环境变量:
> export ROCH_LASER_ENABLE=false
> export ROCH_3D_SENSOR=true
> export ROCH_3D_SENSOR_NAV_ENABLE=true

