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


# import st_CallPerson
# import st_FollowPerson
# import st_GraspItem
# import st_HandItem
# import st_Img_FindItem
# import st_Img_FindPerson
# import st_PlaceItem
# import st_SLAM_Move
# import st_SRec_AnswerQuestion
# import st_SRec_AskPerson
# import st_TelltoPerson


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
        raw_input('##### Type enter key to start #####')
        commonf_actionf_sound_effect_single('found')
        commonf_speech_single('状态机已开启')
        return 'exit1'


#--------------------------------------------------
# 检测门是否开启
#--------------------------------------------------
class DetectDoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        #commonf_speech_single('开门,查个水表!')
        call(['rosrun', 'roch_gpsr','detect_dooropen.py'])
        return 'exit1'


#--------------------------------------------------
# 到达指定地点
#--------------------------------------------------     
class SLAM_GotoCommandPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1'])

    def execute(self, userdata):
        # 到指定地点id1500，序号17
        place = rospy.get_param("/param/gpsr/location/all")
        commonf_actionf_move_base(place[17]['pos']['X'], place[17]['pos']['Y'], place[17]['pos']['Yaw'])
        commonf_speech_single('您好,我是嘉嘉,很高兴为您服务!')
        return 'exit1'

#--------------------------------------------------
# 询问任务,语音识别
#--------------------------------------------------
class SRec_AskCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1', 'err_in_speech_rec'])

    def execute(self, userdata):
        rospy.set_param('/param/gpsr/grammer', "mission") #询问任务,必须设为'mission'
        commonf_speech_single('请在听到滴滴两声后跟我说话,谢谢.')
        commonf_actionf_sound_effect_multi('speech_begin')
        grammer = rospy.get_param('/param/gpsr/grammer')
        if commonf_actionf_speech_asr(grammer): # 识别任务成功,进入下一步
           return 'exit1'
        else:
            commonf_speech_single('对不起我没有听到你说什么,请再说一次.')
            return 'err_in_speech_rec'

#--------------------------------------------------
# 任务解析
#--------------------------------------------------
class TaskRec(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit1', 'err_in_task_rec'])

    def execute(self, userdata):
        rospy.loginfo('TaskRec: 正在对任务进行解析...')
        grammer = rospy.get_param('/param/gpsr/grammer')
        if commonf_actionf_task_rec(grammer):
            return 'exit1'
        else:
            commonf_speech_single('对不起我没有理解你说的内容,请靠近我的话筒再对我说一次.')
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
                commonf_speech_single('好的,我知道了')
                rospy.set_param('/param/gpsr/task/order', 0) #设定任务进度,共3步
                return 'exit1'
            else:
                commonf_speech_single('不对吗?那就麻烦您再说一遍了.')
                return 'get_task_again'
        else:#无语音输入
            commonf_speech_single('对不起，我刚才没有听到，能不能麻烦您再跟我确认一遍。')
            return 'err_in_task_confirm'

#--------------------------------------------------
# 任务分配
#--------------------------------------------------
class SolveCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['solve_task1', 'solve_task2', 'solve_task3', 'exit1'])

    def execute(self, userdata):
        task_order = rospy.get_param("/param/gpsr/task/order") + 1
        rospy.set_param("/param/gpsr/task/order", task_order)

        if task_order == 1:
            return 'solve_task1'
        if task_order == 2:
            return 'solve_task2'
        if task_order == 3:
            return 'solve_task3'
        else:
            return 'exit1'

#--------------------------------------------------
# 执行第一个任务
#--------------------------------------------------
class SolveTask1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pass', 'exit1'])

    def execute(self, userdata):
        commonf_speech_single('正在执行第一个任务.')
        pas = rospy.get_param('/param/gpsr/task1/pass')              # bool
        move = rospy.get_param('/param/gpsr/task1/move')              # bool
        loc_match = rospy.get_param("/param/gpsr/task1/location/match")   # bool
        loc_detail = rospy.get_param("/param/gpsr/task1/location/detail") # int
        if not pas:
            if loc_match and move:
                place_order = get_location_dic_order(loc_detail)# int
                if rospy.get_param("/param/gpsr/location/dic"): # 位置在字典里
                    place = rospy.get_param('/param/gpsr/location/all')
                    place_pose = place[place_order] # list
                    commonf_actionf_move_base(place_pose['pos']['X'], place_pose['pos']['Y'], place_pose['pos']['Yaw'])
                    return 'exit1'
                else:# 位置不在字典里
                    commonf_speech_single('你说的这个地方我不知道在哪,我先做下一个任务')
                    return 'pass'
            else: # 地点不匹配
                commonf_speech_single('你说的前两个地点不匹配,我先做下一个任务')
                return 'pass'
        else:
            commonf_speech_single('第一个任务有问题,我先做下一个任务')
            return 'pass'


#--------------------------------------------------
# 执行第二个任务
#--------------------------------------------------
class SolveTask2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pass', 'exit1'])

    def execute(self, userdata):
        commonf_speech_single('正在执行第二个任务.')
        pas = rospy.get_param('/param/gpsr/task2/pass')              # bool
        move = rospy.get_param('/param/gpsr/task2/move')              # bool
        loc_match = rospy.get_param("/param/gpsr/task2/location/match")   # bool
        loc_detail = rospy.get_param("/param/gpsr/task2/location/detail") # int
        count = rospy.get_param('/param/gpsr/task2/count') # bool
        find = rospy.get_param('/param/gpsr/task2/find') # bool
        grasp = rospy.get_param('/param/gpsr/task2/grasp') # bool
        item_exist = rospy.get_param('/param/gpsr/task2/item/exist') # bool
        item_detail = rospy.get_param('/param/gpsr/task2/item/detail') # int

        if not pas:
            # task2 移动的情况
            if loc_match and move:
                place_order = get_location_dic_order(loc_detail)# int
                if rospy.get_param("/param/gpsr/location/dic"): # 位置在字典里
                    place = rospy.get_param('/param/gpsr/location/all')
                    place_pose = place[place_order]         # list
                    commonf_actionf_move_base(place_pose['pos']['X'], place_pose['pos']['Y'], place_pose['pos']['Yaw'])
                    return 'exit1'
                else:# 位置不在字典里
                    commonf_speech_single('你说的这个地方我不知道在哪,我先做下一个任务')
                    return 'pass'
            # task2 抓取的情况
            elif grasp and item_exist:
                # 待添加代码
                commonf_speech_single('对不起，我没看到这里有东西')
                # commonf_speech_single('我先做下一个任务哦')

                return 'exit1'
            # task2 找人或东西的情况
            elif find:
                # 待添加代码
                commonf_speech_multi('嗨，小伙子。你在哪呢？')
                for i in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 2)
                    rospy.sleep(0.2)
                rospy.sleep(1)
                for j in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -2)
                    rospy.sleep(0.3)
                rospy.sleep(1)
                for k in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 2)
                    rospy.sleep(0.2)
                rospy.sleep(1)
                for l in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -2)
                    rospy.sleep(0.1)
                
                return 'exit1'
            # task2 数人或东西的情况
            elif count:
                # 待添加代码
                commonf_speech_multi('一二三四五，上山打老虎，老虎有几只，我来数一数。')
                for i in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 1)
                    rospy.sleep(0.2)
                rospy.sleep(1)
                for j in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -1)
                    rospy.sleep(0.3)
                rospy.sleep(1)
                for k in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 1)
                    rospy.sleep(0.2)
                rospy.sleep(1)
                for l in (0, 50):
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -1)
                    rospy.sleep(0.1)
                commonf_speech_single('一，二，三，这里有3个哦。')

                return 'exit1'
            else: # 移动地点不匹配的情况
                commonf_speech_single('你说的这两个地点不匹配哦,我先做下一个任务')
                return 'pass'
        else:
            return 'pass'

#--------------------------------------------------
# 执行第三个任务
#--------------------------------------------------
class SolveTask3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit1'])

    def execute(self, userdata):
        commonf_speech_single('正在执行第三个任务.')
        enable = rospy.get_param('/param/gpsr/task3/enable')                # bool
        move = rospy.get_param('/param/gpsr/task3/move')                    # bool
        loc_match = rospy.get_param("/param/gpsr/task3/location/match")     # bool
        loc_detail = rospy.get_param("/param/gpsr/task3/location/detail")   # int
        deliver = rospy.get_param('/param/gpsr/task3/deliver')              # bool
        give = rospy.get_param('/param/gpsr/task3/give')                    # bool
        lead = rospy.get_param('/param/gpsr/task3/lead')                    # bool
        put = rospy.get_param('/param/gpsr/task3/put')                      # bool
        tell = rospy.get_param('/param/gpsr/task3/tell')                    # bool

        if enable:
            # task3 移动的情况
            if loc_match and move:
                place_order = get_location_dic_order(loc_detail)# int
                if rospy.get_param("/param/gpsr/location/dic"): #位置在字典里
                    if lead: # 有引导人的情况
                        commonf_speech_single('请你跟紧我,别丢了!')
                    place = rospy.get_param('/param/gpsr/location/all')
                    place_pose = place[place_order]             # list
                    commonf_actionf_move_base(place_pose['pos']['X'], place_pose['pos']['Y'], place_pose['pos']['Yaw'])
                    if deliver or put: # 有运送东西的情况
                        #commonf_speech_single('东西运到了,请查收.我就先放在地上了.')
                        commonf_speech_single('对不起，我两手空空的回来了')
                        # 待添加代码, 放东西
                    if give: # 有递东西的情况
                        #commonf_speech_single('东西运到了,请伸手,我要松开了.')
                        commonf_speech_single('对不起，我两手空空的回来了')
                        # 待添加代码, 递东西
                    return 'done'
                else:# 位置不在字典里
                    commonf_speech_single('你说的这个地方我不知道在哪')
                    return 'exit1'
            # task3 交流的情况
            elif tell:
                commonf_speech_single('我好无聊,跟你聊聊天吧.我是来自浙江嘉兴学院的嘉嘉,很高兴认识你')
                return 'done'
            else: # 地点可能不匹配的情况
                commonf_speech_single('我不知道要去哪里.')
                return 'exit1'
        else:
            return 'exit1'

#--------------------------------------------------
# 完成任务，离开场地
#--------------------------------------------------
class SLAM_LeaveArea(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])

    def execute(self, userdata):
        commonf_speech_single('任务做完啦，我要走啦！')
        place_pose = place[18] # 退场的地址
        commonf_actionf_move_base(place_pose['pos']['X'], place_pose['pos']['Y'], place_pose['pos']['Yaw'])
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
                               transitions={'exit1':'DetectDoorOpen'})
        smach.StateMachine.add('DetectDoorOpen', DetectDoorOpen(),
                               transitions={'exit1':'SLAM_GotoCommandPosition'})
        smach.StateMachine.add('SLAM_GotoCommandPosition', SLAM_GotoCommandPosition(),
                               transitions={'exit1':'SRec_AskCommand'})
        smach.StateMachine.add('SRec_AskCommand', SRec_AskCommand(),
                               transitions={'exit1':'TaskRec',
                                            'err_in_speech_rec':'SRec_AskCommand'})
        smach.StateMachine.add('TaskRec', TaskRec(),
                               transitions={'exit1':'TaskConfirm',
                                            'err_in_task_rec':'SRec_AskCommand'})
        smach.StateMachine.add('TaskConfirm', TaskConfirm(),
                               transitions={'exit1':'SolveCommand',
                                            'get_task_again':'SRec_AskCommand',
                                            'err_in_task_confirm':'TaskConfirm'})
        smach.StateMachine.add('SolveCommand', SolveCommand(),
                               transitions={'solve_task1':'SolveTask1',
                                            'solve_task2':'SolveTask2',
                                            'solve_task3':'SolveTask3',
                                            'exit1':'SLAM_LeaveArea'})
        smach.StateMachine.add('SolveTask1', SolveTask1(),
                               transitions={'pass':'SolveCommand',
                                            'exit1':'SolveCommand'})
        smach.StateMachine.add('SolveTask2', SolveTask2(),
                               transitions={'pass':'SolveCommand',
                                            'exit1':'SolveCommand'})
        smach.StateMachine.add('SolveTask3', SolveTask3(),
                               transitions={'done':'SolveCommand',
                                            'exit1':'SLAM_LeaveArea'})
        smach.StateMachine.add('SLAM_LeaveArea', SLAM_LeaveArea(),
                               transitions={'exit1':'exit'})

        # smach.StateMachine.add('SLAM_Move', st_SLAM_Move.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('FollowPerson',st_FollowPerson.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('Img_FindItem',st_Img_FindItem.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('Img_FindPerson',st_Img_FindPerson.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('GraspItem',st_GraspItem.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('PlaceItem',st_PlaceItem.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('HandItem',st_HandItem.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('CallPerson',st_CallPerson.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('SRec_AskPerson',st_SRec_AskPerson.MainState(),
        #                        transitions={'exit':'SolveCommand'})
        # smach.StateMachine.add('TelltoPerson',st_TelltoPerson.MainState(),
        #                        transitions={'exit':'SolveCommand'})



    sis = smach_ros.IntrospectionServer('sm', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    # commonf_speech_multi('任务结束')
    raw_input('##### Type Ctrl + c key to end #####')


    while not rospy.is_shutdown():
        rospy.spin()
