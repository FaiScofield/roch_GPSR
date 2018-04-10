#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from roch_gpsr.msg import TaskRecAction
from roch_gpsr.msg import TaskRecGoal
# from roch_gpsr.msg import TaskRecFeedback
from roch_gpsr.msg import TaskRecResult

def task_rec_client(speech_str):
    rospy.init_node("task_rec_client", anonymous=False)
    
    tr_client = actionlib.SimpleActionClient('task_rec_action', TaskRecAction)
    tr_client.wait_for_server()
    
    goal = TaskRecGoal()
    goal.task_rec_goal = speech_str
    tr_client.send_goal(goal)
    finished = tr_client.wait_for_result()
    if finished:
        result = tr_client.get_result()
        rospy.loginfo("任务解析完成,结果task_rec_result: %s", result.task_rec_result)
    else:
        rospy.loginfo("任务解析超时...")

if __name__ == "__main__":
    try:
        rospy.loginfo("客户端开启...")
        task_rec_client("confirm")
    except Exception, e:
        print "done"