#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#SLAM移动ROS节点
#
#author: Vance Wu
#date: 17/07/31
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *
from common_function import *


#--------------------------------------------------
#全局变量
#--------------------------------------------------
move_base_cmd_vel = Twist()


#--------------------------------------------------
#--------------------------------------------------
def subf_move_base_cmd_vel(sub_move_base_cmd_vel):
    global move_base_cmd_vel
    move_base_cmd_vel = sub_move_base_cmd_vel


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if not rospy.is_shutdown:
        commonf_speech_multi('前往目的地中')

        rospy.Subscriber("/move_base/cmd_vel", Twist, subf_move_base_cmd_vel)

        target_x = float(rospy.get_param('/param/gpsr/slam_goal/x'))
        target_y = float(rospy.get_param('/param/gpsr/slam_goal/y'))
        target_yaw = float(rospy.get_param('/param/gpsr/slam_goal/yaw'))

        th_trans = 0.2

        tf_listener = tf.TransformListener()

        main_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                try:
                    (translation, rotation) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                except:
                    continue
                break

            euler = euler_from_quaternion([rotation[0], rotation[1], rotation[2], rotation[3]])
        
            if abs(target_x - translation[0]) > th_trans or abs(target_y - translation[1]) > th_trans:                
                if abs(move_base_cmd_vel.linear.x) < 0.1 and abs(move_base_cmd_vel.angular.z) < 0.261:
                    if move_base_cmd_vel.angular.z > 0:
                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.261)
                    else:
                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.261)
                if move_base_cmd_vel.angular.z > 0.5:
                    commonf_pubf_cmd_vel(move_base_cmd_vel.linear.x, 0, 0, 0, 0, 0.5)
                elif move_base_cmd_vel.angular.z < -0.5:
                    commonf_pubf_cmd_vel(move_base_cmd_vel.linear.x, 0, 0, 0, 0, -0.5)
                else:
                    commonf_pubf_cmd_vel(move_base_cmd_vel.linear.x, 0, 0, 0, 0, move_base_cmd_vel.angular.z)
            else:
                if th_trans == 0.2:
                    th_trans = 0.3
                #     x
                #     |
                #   1 | 4
                #     |
                # y-------
                #     |
                #   2 | 3
                #     |
                if target_yaw > 0:
                    if target_yaw < 1.57:
                        if euler[2] > 0:
                            if euler[2] < 1.57:
                                #目標: 1
                                #現在: 1                                
                                if abs(target_yaw - euler[2]) > 0.262:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                            else:
                                #目標: 1
                                #現在: 2
                                if abs(target_yaw - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                        else:
                            if euler[2] < -1.57:
                                #目標: 1
                                #現在: 3
                                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)    
                            else:
                                #目標: 1
                                #現在: 4
                                if abs(0 - target_yaw) + abs(0 - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                elif abs(0 - target_yaw) + abs(0 - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                else:
                                    break
                    else:
                        if euler[2] > 0:
                            if euler[2] < 1.57:
                                #目標: 2
                                #現在: 1
                                if abs(target_yaw - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                else:
                                    break
                            else:
                                #目標: 2
                                #現在: 2
                                if abs(target_yaw - euler[2]) > 0.262:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                        else:
                            if euler[2] < -1.57:
                                #目標: 2
                                #現在: 3
                                if abs(math.pi - target_yaw) + abs(-math.pi - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(math.pi - target_yaw) + abs(-math.pi - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                            else:
                                #目標: 2
                                #現在: 4
                                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)    
                else:
                    if target_yaw < -1.57:
                        if euler[2] > 0:
                            if euler[2] < 1.57:
                                #目標: 3
                                #現在: 1
                                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)    
                            else:
                                #目標: 3
                                #現在: 2
                                if abs(-math.pi - target_yaw) + abs(math.pi - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                elif abs(-math.pi - target_yaw) + abs(math.pi - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                else:
                                    break
                        else:
                            if euler[2] < -1.57:
                                #目標: 3
                                #現在: 3
                                if abs(target_yaw - euler[2]) > 0.262:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                            else:
                                #目標: 3
                                #現在: 4
                                if abs(target_yaw - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                    else:
                        if euler[2] > 0:
                            if euler[2] < 1.57:
                                #目標: 4
                                #現在: 1
                                if abs(0 - target_yaw) + abs(0 - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(math.pi - target_yaw) + abs(-math.pi - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break
                            else:
                                #目標: 4
                                #現在: 2
                                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)    
                        else:
                            if euler[2] < -1.57:
                                #目標: 4
                                #現在: 3
                                if abs(target_yaw - euler[2]) > 0.262:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                else:
                                    break
                            else:
                                #目標: 4
                                #現在: 4
                                if abs(target_yaw - euler[2]) > 0.262:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.349)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.349)
                                elif abs(target_yaw - euler[2]) > 0.087:
                                    if target_yaw > euler[2]:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.174)
                                    else:
                                        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.174)
                                else:
                                    break


            #main_rate.sleep()

        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

        commonf_speech_multi('已经达到目的地了呢.')

        sys.exit(0)
