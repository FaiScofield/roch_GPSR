#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ 2016 ROS node of state machine for GPSR
#
#author: Yutaro ISHIDA
#date: 16/03/21
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *
from common_function import *


rospy.sleep(1) # Wait for the param node to start up

#--------------------------------------------------
# 初始状态机
#--------------------------------------------------
class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        return 'exit1'

#--------------------------------------------------
# 等待开始信号
#--------------------------------------------------
class WaitStartSig(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        commonf_actionf_sound_effect_single('found')
        # commonf_actionf_awaken #待添加唤醒程序
        commonf_speech_single('我在这')
        return 'exit1'

#--------------------------------------------------
# 交互询问
#--------------------------------------------------
class SRec_AskCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1', 'err_in_speech_rec'])

    def execute(self, userdata):
        commonf_actionf_sound_effect_multi('speech_begin')
        if commonf_actionf_speech_asr("confirm"): # 识别任务成功,进入下一步
           return 'exit1'
        else:
            commonf_speech_single('对不起我没听清,请再说一遍.')
            return 'err_in_speech_rec'

#--------------------------------------------------
# 任务解析
#--------------------------------------------------
class TaskRec(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1', 'err_in_task_rec'])

    def execute(self, userdata):
        rospy.loginfo('TaskRec: 正在对任务进行解析...')
        if commonf_actionf_task_rec("confirm"):
            return 'exit1'
        else:
            commonf_speech_single('对不起我没有理解你说的内容,请再说一遍.')
            return 'err_in_task_rec'


#--------------------------------------------------
# 任务确认
#--------------------------------------------------
class TaskConfirm(smach.State):
    def __init__(self):
        smach.State.__init__ (self, outcomes=['exit1', 'get_task_again', 'err_in_task_confirm'])

    def execute(self, userdata):
        rospy.loginfo('TaskConfirm: 正在对任务进行确认...')
        rospy.set_param('/param/gpsr/grammer', "confirm") #确认时设为'confirm'

        task_text = rospy.get_param('/param/gpsr/task/text')
        # unicode string转成byte string,否则报错
        text = "你刚才是说" + task_text.encode('utf-8') + "对吗?"
        commonf_speech_single(text)
        commonf_actionf_sound_effect_multi('speech_begin')
        if commonf_actionf_task_confirm():#有语音输入
            task_confirm = rospy.get_param('/param/gpsr/task/confirm')
            if task_confirm == 'yes':
                commonf_speech_single('好的,马上就去')
                return 'exit1'
            else:
                commonf_speech_single('不对吗?那就麻烦您再说一遍了.')
                return 'get_task_again'
        else:#无语音输入
            commonf_speech_single('对不起，我刚才没有听到，能不能麻烦您再跟我确认一遍。')
            return 'err_in_task_confirm'

#--------------------------------------------------
# 到达指定地点
#--------------------------------------------------     
class SLAM_GotoPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        # commonf_actionf_move_base(1.0, 1.0, 1.0)   #待修改
        commonf_speech_single('邹城您好，我来拿优盘，请您把优盘放到我身上，谢谢')
        rospy.sleep(5)
        return 'exit1'

#--------------------------------------------------
# 确认物品是否拿到
#-------------------------------------------------- 
class GetItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1', 'again'])

    def execute(self, userdata):       
        commonf_speech_single('请问您把优盘放到我身上了吗？')
        if commonf_actionf_task_confirm():#有语音输入
            task_confirm = rospy.get_param('/param/gpsr/task/confirm')
            if task_confirm == 'yes':
                commonf_speech_single('好的,谢谢您的配合，我要走了')
                return 'exit1'
            else:
                commonf_speech_single('那我再等您一会')
                rospy.sleep(5)
                return 'again'
        else:#无语音输入
            commonf_speech_single('对不起，我刚才没有听到')
            return 'again'
        return 'exit1'

#--------------------------------------------------
# 返回
#--------------------------------------------------
class SLAM_Goback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        # commonf_actionf_move_base(0.0, 0.0, 0.0)   #待修改
        commonf_speech_single('我回来了，优盘已经拿到，请您拿走')
        rospy.sleep(5)
        return 'exit1'



#--------------------------------------------------
# 主函数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    sm = smach.StateMachine(outcomes=['exit'])

    # Play startup sound
    # commonf_actionf_sound_effect_single('launch')

    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
    commonf_speech_multi ('正在启动中')
    rospy.loginfo('GPSR开启')
    rospy.loginfo('请确定初始状态:DetectDoorOpen, SLAM_GotoCommandPosition, SRec_AskCommand, SolveCommand, SLAM_LeaveArea...')
    rospy.loginfo('如果要从最初始的状态开始,请直接按回车')
    start_state = raw_input('\n****** 请输入开始状态名称: ****** >> ')
    if not start_state:
        start_state = 'WaitStartSig'

    rospy.loginfo('状态机开启...')
    with sm:
        smach.StateMachine.add('init', init(),
                               transitions={'exit1':start_state})
        smach.StateMachine.add('WaitStartSig', WaitStartSig(),
                               transitions={'exit1':'SRec_AskCommand'})
        smach.StateMachine.add('SRec_AskCommand', SRec_AskCommand(),
                               transitions={'exit1':'TaskRec',
                                            'err_in_speech_rec':'SRec_AskCommand'})
        smach.StateMachine.add('TaskRec', TaskRec(),
                               transitions={'exit1':'TaskConfirm',
                                            'err_in_task_rec':'SRec_AskCommand'})
        smach.StateMachine.add('TaskConfirm', TaskConfirm(),
                               transitions={'exit1':'SLAM_GotoPosition',
                                            'get_task_again':'SRec_AskCommand',
                                            'err_in_task_confirm':'TaskConfirm'})
        smach.StateMachine.add('SLAM_GotoPosition', SLAM_GotoPosition(),
                               transitions={'exit1':'GetItem'})
        smach.StateMachine.add('GetItem', GetItem(),
                               transitions={'again':'GetItem',
                                            'exit1':'SLAM_Goback'})
        smach.StateMachine.add('SLAM_Goback', SLAM_Goback(),
                               transitions={'exit1':'exit'})


    sis = smach_ros.IntrospectionServer('sm', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    # commonf_speech_multi('任务结束')
    raw_input('##### Type Ctrl + c key to end #####')


    while not rospy.is_shutdown():
        rospy.spin()
