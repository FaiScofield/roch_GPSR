#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *


#--------------------------------------------------
#全局変量
#--------------------------------------------------
settings = termios.tcgetattr(sys.stdin)

# pub_scan_mode = rospy.Publisher('/scan/mode', String, queue_size = 1)
# pub_cam_pan_angle = rospy.Publisher('/dy_servo_cam_pan/angle', Float64, queue_size = 1)
# pub_cam_tilt_angle = rospy.Publisher('/dy_servo_cam_tilt/angle', Float64, queue_size = 1)
# pub_mic_pan_angle = rospy.Publisher('/dy_servo_mic_pan/angle', Float64, queue_size = 1)
# pub_mic_tilt_angle = rospy.Publisher('/dy_servo_mic_tilt/angle', Float64, queue_size = 1)
pub_cmd_vel = rospy.Publisher('/roch_velocity_controller/cmd_vel', Twist, queue_size = 1000)


#--------------------------------------------------
# 单线程语音合成(有阻塞,直至说话完毕)
#--------------------------------------------------
def commonf_speech_single(speech_str):
    rospy.loginfo("语音合成(有阻塞):" + speech_str)

    tts_action_client = actionlib.SimpleActionClient('xfei_tts_action', XFeiTTSAction)
    tts_action_client.wait_for_server()
    
    goal = XFeiTTSGoal()
    goal.xfei_tts_goal = speech_str
    tts_action_client.send_goal(goal)
    tts_action_client.wait_for_result()

#--------------------------------------------------
# 多线程语音合成(无阻塞)
#--------------------------------------------------
def commonf_speech_multi(speech_str):
    rospy.loginfo("语音合成(无阻塞):" + speech_str)

    tts_action_client = actionlib.SimpleActionClient('xfei_tts_action', XFeiTTSAction)
    tts_action_client.wait_for_server()

    goal = XFeiTTSGoal()
    goal.xfei_tts_goal = speech_str
    tts_action_client.send_goal(goal)
    #speech_syn_action_client.wait_for_result() #不阻塞


#--------------------------------------------------
# 单线程播放音乐(有阻塞,直至说播放完毕)
#--------------------------------------------------
def commonf_actionf_sound_effect_single(sound_effect_goal):
        sound_effect_action_client = actionlib.SimpleActionClient('sound_effect_action', SoundEffectAction)
        sound_effect_action_client.wait_for_server()

        goal = SoundEffectGoal()
        goal.sound_effect_goal = sound_effect_goal
        sound_effect_action_client.send_goal(goal)
        sound_effect_action_client.wait_for_result()

#--------------------------------------------------
# 单线程播放音乐(无阻塞)
#--------------------------------------------------
def commonf_actionf_sound_effect_multi(sound_effect_goal):
        sound_effect_action_client = actionlib.SimpleActionClient('sound_effect_action', SoundEffectAction)
        sound_effect_action_client.wait_for_server()

        goal = SoundEffectGoal()
        goal.sound_effect_goal = sound_effect_goal
        sound_effect_action_client.send_goal(goal)
        # sound_effect_action_client.wait_for_result()

#--------------------------------------------------
# 杀死节点进程
#--------------------------------------------------
def commonf_node_killer(node_name): 
    node_killer_p = Popen(['rosnode','list'], stdout = PIPE) 
    node_killer_p.wait() 
    node_list = node_killer_p.communicate() 
    n = node_list[0] 
    n = n.split("\n") 
    for i in range(len(n)): 
        tmp = n[i]   
        if tmp.find(node_name) == 1: 
            call(['rosnode', 'kill', n[i]]) 
            break


#--------------------------------------------------
#--------------------------------------------------
# def commonf_task_stepin():
#         rospy.loginfo('##### 进入任务 #####')


# --------------------------------------------------
# --------------------------------------------------
# def commonf_task_stepout():
#         rospy.loginfo('##### 退出任务 #####')


#--------------------------------------------------
# 发布速度指令
#--------------------------------------------------
def commonf_pubf_cmd_vel(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    global pub_cmd_vel

    cmd_vel = Twist()
    cmd_vel.linear.x = linear_x
    cmd_vel.linear.y = linear_y
    cmd_vel.linear.z = linear_z
    cmd_vel.angular.x = angular_x
    cmd_vel.angular.y = angular_y
    cmd_vel.angular.z = angular_z
    pub_cmd_vel.publish(cmd_vel)
    # rospy.sleep(0.05)
    # pub_cmd_vel.publish(cmd_vel)    # ??


#--------------------------------------------------
# 开启监听并识别命令词,保存识别到的文件
#--------------------------------------------------
def commonf_actionf_speech_asr(grammer):
    asr_action_client = actionlib.SimpleActionClient('roch_asr_action', RochASRAction)
    asr_action_client.wait_for_server()

    result = RochASRResult()
    goal = RochASRGoal()
    goal.roch_asr_goal = grammer
    asr_action_client.send_goal(goal)

    if (asr_action_client.wait_for_result()):   # bool
        rospy.loginfo("语音识别结束,结果为:%s", asr_action_client.get_result().roch_asr_result) # bool
    else:
        rospy.loginfo("语音识别处理超时...")
    return asr_action_client.get_result().roch_asr_result # bool


#--------------------------------------------------
# 读取命令文件,解析任务
#--------------------------------------------------
def commonf_actionf_task_rec(grammer):
    task_rec_client = actionlib.SimpleActionClient('task_rec_action', TaskRecAction)
    task_rec_client.wait_for_server()

    result = TaskRecResult()
    goal = TaskRecGoal()
    goal.task_rec_goal = grammer
    task_rec_client.send_goal(goal)

    if (task_rec_client.wait_for_result()):   # bool
        rospy.loginfo("任务识别结束,结果为:%s", task_rec_client.get_result().task_rec_result) # bool
    else:
        rospy.loginfo("任务识别处理超时...")
    return task_rec_client.get_result().task_rec_result # bool


#--------------------------------------------------
# 对解析后的任务进行确认
#--------------------------------------------------
def commonf_actionf_task_confirm():
    # grammer = rospy.get_param('/param/gpsr/grammer')
    if (commonf_actionf_speech_asr('confirm')): # bool
        return commonf_actionf_task_rec('confirm')
    else:
        return False

#--------------------------------------------------
# 发送目标位置以控制机器人移动
#--------------------------------------------------
def commonf_actionf_move_base(x, y, yaw):
    rospy.loginfo('Goal pos x: ' + str(x) + ' y: ' + str(y) + ' yaw: ' + str(yaw))
    commonf_speech_single('我要行动了!')

    move_base_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_action_client.wait_for_server()

    quaternion = quaternion_from_euler(0.0, 0.0, yaw)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    move_base_action_client.send_goal(goal)

    move_base_action_client.wait_for_result()

    # rospy.set_param('/param/gpsr/slam/goal/x', float(x)) #float
    # rospy.set_param('/param/gpsr/slam/goal/y', float(y)) #float
    # rospy.set_param('/param/gpsr/slam/goal/yaw', float(yaw)) #float

    # call(['rosrun', 'roch_gpsr', 'slam_goto.py'])

    # move_base_action_client.cancel_goal() # 注释后报错消失 Received comm state PREEMPTING when in simple state DONE with SimpleActionClient in NS /move_base
    commonf_speech_single('已到达目的地!')
    # return True


#--------------------------------------------------
# 返回位置id在参数表'/param/gpsr/location/all'中的索引值(需根据实际情况修改)
#--------------------------------------------------
def get_location_dic_order(location_id): # 传入int
    location_dic = { # 字典内容全部为整型
        1100:0,
        1101:1,
        1102:2,
        1103:3,
        1104:4,
        1105:5,
        1200:6,
        1201:7,
        1202:8,
        1300:9,
        1301:10,
        1302:11,
        1303:12,
        1400:13,
        1401:14,
        1402:15,
        1403:16,
        1500:17,
        1600:18}
    if location_id in location_dic:
        location_order = location_dic[location_id]
        rospy.set_param('/param/gpsr/location/dic', True)
        return location_order
    else:
        rospy.set_param('/param/gpsr/location/dic', False)
        return 0

#--------------------------------------------------
# 返回物品id在参数表'/param/gpsr/item/all'中的索引值(需根据实际情况修改)
#--------------------------------------------------
def get_item_dic_order(item_id): # 传入int
    item_dic = { # 字典内容全部为整型
        2000:0,
        2001:1,
        2002:2,
        2003:3,
        2004:4,
        2005:5,
        2006:6,
        2007:7,
        2008:8,
        2009:9,
        2010:10}
    if item_id in item_dic:
        item_order = item_dic[item_id]
        rospy.set_param('/param/gpsr/item/dic', True)
        return item_order
    else:
        rospy.set_param('/param/gpsr/item/dic', False)
        return 0
