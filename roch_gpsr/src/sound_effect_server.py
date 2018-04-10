#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------------------------------
# 播放音效节点的服务端
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
# 类声明
#--------------------------------------------------
class SoundEffect(object):
    def __init__(self):
        self._sound_effect_action_server = actionlib.SimpleActionServer('sound_effect_action', SoundEffectAction, execute_cb = self.sound_effect)
        self._sound_effect_action_server.start()

    def sound_effect(self, goal):
        if goal.sound_effect_goal == 'speech_begin':
            # os.path.dirname(__file__)是代码文件存放的路径
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/sound/speech_begin.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        elif goal.sound_effect_goal == 'found':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/sound/found.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        elif goal.sound_effect_goal == 'task_understood':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/sound/task_understood.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        elif goal.sound_effect_goal == 'mission_complited':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/sound/mission_complited.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        else:
            rospy.logwarn('[sound_effect]: 没有对应的音效')
            result = SoundEffectResult(sound_effect_result = False)
            self._sound_effect_action_server.set_succeeded(result)


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    sound_effect = SoundEffect()

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():    
        main_rate.sleep()
