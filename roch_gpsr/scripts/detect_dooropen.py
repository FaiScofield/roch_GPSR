#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
# 检测门是否打开
#
# author: Vance Wu
# date: 17/08/02
#--------------------------------------------------

import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *
from common_function import *


# 全局变量
scan_lrf = LaserScan()
flag_start = 0


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_lrf(sub_scan_lrf):
    global scan_lrf, flag_start
    scan_lrf = sub_scan_lrf
    flag_start = 1


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    # if rospy.get_param('/param/gpsr/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
    commonf_speech_multi('请开门，谢谢！')

    # rospy.Subscriber("/scan/lrf", LaserScan, subf_scan_lrf)
    rospy.Subscriber("/scan", LaserScan, subf_scan_lrf)

    main_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if flag_start == 1:
            # 算中心一束激光的返回距离值
            dis = scan_lrf.ranges[int(len(scan_lrf.ranges) / 2)]
            rospy.loginfo("中心一束激光的距离为:%f", dis)

            # 1米以内,原地等待
            if dis > 0.9:
                commonf_speech_single('我看到门开啦!我要进来了!')
                rospy.sleep(1)

#                for i in range(0, 110):
#                    commonf_pubf_cmd_vel(0.3, 0, 0, 0, 0, 0)
#                    rospy.loginfo("前进3米,速度发布中...")
#                    rospy.sleep(0.1)

#                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                sys.exit(0)
            else: # nan的情况也是停止
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

        main_rate.sleep()


