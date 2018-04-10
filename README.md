##------------------
## For Roch GPSR
##
## Author: Vance Wu
## Date: 2017.08.10
##------------------

## 远程登录
ssh ubuntu@192.168.0.116    #sawyer-wifi
ssh ubuntu@10.42.0.1        #hot-roch
    
## 赛前建图
roslaunch roch_gpsr roch_build_map.launch
rosrun rosbag record /scan /tf
rosrun map_server map_saver -f gpsr.yaml

## 比赛开始
## 开启Roch和xtion(ssh)
roslaunch roch_gpsr roch_bringup.launch --screen
## 开启GPSR(ssh)
roslaunch roch_gpsr gpsr_server.launch
roslaunch roch_gpsr gpsr_smach.launch
## 远程监控
roslaunch roch_viz view_navigation.launch --screen

## 开启RealSense(ssh)
roslaunch realsense_camera sr300_nodelet_rgbd.launch

# 注:
## 建图过程用RPlidar,保持以下环境变量:
export ROCH_LASER=rplidar
export ROCH_LASER_ENABLE=true
export ROCH_3D_SENSOR=false
export ROCH_3D_SENSOR_NAV_ENABLE=false
## 比赛过程不使用RPlidar,保持以下环境变量:
export ROCH_LASER_ENABLE=false
export ROCH_3D_SENSOR=true
export ROCH_3D_SENSOR_NAV_ENABLE=true




# 各功能包说明:
+ roch_asr: 命令词识别(离线)
+ roch_gpsr: GPSR任务框架和执行代码
+ roch_tts: 语音合成(在线)
+ xfei_tts: 语音合成(离线)


# 移植程序需要修改的地方:
* 1)先将roch_asr, roch_gpsr, roch_tts或xfei_tts各包中CMakeList.txt文件中的[添加可执行文件部分](add_executable, target_link_libraries, add_dependencies)注释,先行编译一次,使Action的头文件得以生成.

* 2)将三个包中的CMakeList.txt文件及[可执行文件的源文件]中的路径,改为自己电脑上对应的路径.

* 3)讯飞语音的离线版功能包只有35天的体验期,过期后需要申请新的账号并下载新的离线功能包SDK.

* 4)将roch_asr和roch_tts或xfei_tts中可执行文件源代码中的appid改为自己的appid,将包内lib子文件夹中的libmsc.so文件用自己下载的SDK中的libmsc.so文件替换.

* 5)再次编译,即可使用.

* 需要安装的依赖包: mpayer, sox, tinyxml2等
  - sudo apt-get install mplayer sox libtinyxml2-dev
  
* 可能出现的错误及解决方法:
  - 编译出错：检查第1)步是否已生成Action的头文件; 检查和第2)步是否将所有的源文件和CMakeList.txt文件中的绝对路径是否已修改．
  - 使用出错：查看错误提示，到讯飞论坛中查看错误代码对应的错误．常见错误有：
    % 文件找不到: 检查第2)步
    % 语法文件出错: 检查语法文件书写格式是否正确
    % 登录失败:检查第3)步和第4)步
    % 库和ID不匹配:检查第4)步
