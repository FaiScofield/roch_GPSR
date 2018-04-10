#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------------------------------
# 移动到指定地点的状态机
#
# author: Vance Wu
# date: 17/08/03
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *
from common_function import *


#--------------------------------------------------
# 状态机声明部分
#--------------------------------------------------
class MainState(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['exit'])
        with self:
            smach.StateMachine.add('SLAM_Move', SLAM_Move(),
                                   transitions = {'exit1':'exit'})


#--------------------------------------------------
# 状态机执行
#--------------------------------------------------
class SLAM_Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])

    def execute(self, userdata):
        # commonf_dbg_sm_stepin()
        order = rospy.get_param('/param/gpsr/task/order')
        if order == 1:
            pass_1 = rospy.get_param('/param/gpsr/task1/pass')
            move_1 = rospy.get_param('/param/gpsr/task1/move')
            match_1 = rospy.get_param("/param/gpsr/task1/location/match")
            detail_1 = rospy.get_param("/param/gpsr/task1/location/detail")
        
            i_command = rospy.get_param('/param/gpsr/task1/i_command')
        command = rospy.get_param('/param/gpsr/command/detail')[i_command]
        i_command = rospy.get_param('/param/gpsr/command/i_command')
        command = rospy.get_param('/param/gpsr/command/detail')[i_command]

        place_db = rospy.get_param('/param/gpsr/place/db')
        place=place_db[int(command['To'])-1] #idマッチング

        commonf_actionf_move_base(place['pos']['X'], place['pos']['Y'], place['pos']['Yaw'])

        # commonf_dbg_sm_stepout()
        return 'exit1'

