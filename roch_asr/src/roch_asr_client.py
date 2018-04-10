#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from roch_asr.msg import RochASRAction
from roch_asr.msg import RochASRGoal
# from roch_asr.msg import RochASRFeedback
# from roch_asr.msg import RochASRResult

def auto_speech_recog(asr_sig):
    rospy.init_node("roch_asr_client", anonymous=False)
    
    asr_action_client = actionlib.SimpleActionClient('roch_asr_action', RochASRAction)
    asr_action_client.wait_for_server()
    
    goal = RochASRGoal()
    goal.roch_asr_goal = asr_sig
    asr_action_client.send_goal(goal)
    finished = asr_action_client.wait_for_result() # bool

    if finished:
        result = asr_action_client.get_result()
        rospy.loginfo("语音识别结束,roch_asr_result的值为: %s", result.roch_asr_result)
    else:
        rospy.loginfo("Action did not finish before the time out.")


if __name__ == "__main__":
    try:
        auto_speech_recog("asr_yes")
    except Exception:
        print "done"
