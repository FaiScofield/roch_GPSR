#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
# GPSR用参数
#
# author: Vance Wu
# date: 17/08/08
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('roch_gpsr') + '/scripts')

from common_import import *
from common_function import *
from common_param import *


if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    ##--------------------------------------------------
    ## SLAM移动所需参数
    ##--------------------------------------------------
    rospy.set_param('/param/gpsr/slam/goal/x', 0.0)     #float
    rospy.set_param('/param/gpsr/slam/goal/y', 0.0)     #float
    rospy.set_param('/param/gpsr/slam/goal/yaw', 0.0)   #float

    #--------------------------------------------------
    # 机械臂抓取所需参数
    #--------------------------------------------------
    rospy.set_param('/param/gpsr/item/id', 0) #int 待抓取对象id
    rospy.set_param('/param/gpsr/item/pos/x', 0.0) #float 相对base_link的x距离(必须)
    rospy.set_param('/param/gpsr/item/pos/y', 0.0) #float 相对base_link的y距离(必须)
    rospy.set_param('/param/gpsr/item/pos/z', 0.0) #float 相对base_link的z距离(必须)
    rospy.set_param('/param/gpsr/item/height', 0.72) #float 待抓取对象的高度
    rospy.set_param('/param/gpsr/item/find/cnt', 0) #int
    rospy.set_param('/param/gpsr/item/find/try/cnt', 0) #int

    ##--------------------------------------------------
    ## 设置物体集合
    ##--------------------------------------------------
    rospy.set_param('/param/gpsr/item/dic', False)
    rospy.set_param('/param/gpsr/item/all', [
        {'obj_id':2000, 'obj_name':'人', 'obj_class':'people'},
        {'obj_id':2001, 'obj_name':'纸巾', 'obj_class':'else'},
        {'obj_id':2002, 'obj_name':'杯子', 'obj_class':'else'},
        {'obj_id':2003, 'obj_name':'橙汁', 'obj_class':'drink'},
        {'obj_id':2004, 'obj_name':'牛奶', 'obj_class':'drink'},
        {'obj_id':2005, 'obj_name':'苹果', 'obj_class':'food'},
        {'obj_id':2006, 'obj_name':'橡皮擦', 'obj_class':'else'},
        {'obj_id':2007, 'obj_name':'绿茶', 'obj_class':'drink'},
        {'obj_id':2008, 'obj_name':'可乐', 'obj_class':'drink'},
        {'obj_id':2009, 'obj_name':'雪碧', 'obj_class':'drink'},
        {'obj_id':2010, 'obj_name':'青草膏', 'obj_class':'else'}    ])

    ##--------------------------------------------------
    ## 设置地点集合
    ##--------------------------------------------------
    rospy.set_param('/param/gpsr/location/dic', False)
    rospy.set_param('/param/gpsr/location/all', [
        {'place_id':1100, 'place_class':'livingroom', 'place_name':u'客厅', 'pos':{'X':4.91,'Y':-5.92,'Yaw':0}},
        {'place_id':1101, 'place_class':'livingroom', 'place_name':u'长沙发', 'pos':{'X':7.28,'Y':-5,'Yaw':0}},
        {'place_id':1102, 'place_class':'livingroom', 'place_name':u'短沙发', 'pos':{'X':7.41,'Y':-6.84,'Yaw':0}},
        {'place_id':1103, 'place_class':'livingroom', 'place_name':u'长桌', 'pos':{'X':6.18,'Y':-5.66,'Yaw':-1.57}},
        {'place_id':1104, 'place_class':'livingroom', 'place_name':u'圆桌', 'pos':{'X':5.26,'Y':-10.1,'Yaw':-1.8}},
        {'place_id':1105, 'place_class':'livingroom', 'place_name':u'电视机', 'pos':{'X':5.0,'Y':-8.0,'Yaw':-3.14}},

        {'place_id':1200, 'place_class':'dingingroom', 'place_name':u'餐厅', 'pos':{'X':3.0,'Y':-1.0,'Yaw':-1.57}},
        {'place_id':1201, 'place_class':'dingingroom', 'place_name':u'柜子', 'pos':{'X':3.41,'Y':-1.93,'Yaw':-0.0}},
        {'place_id':1202, 'place_class':'dingingroom', 'place_name':u'桌子', 'pos':{'X':2.0,'Y':-1.0,'Yaw':-1.57}},

        {'place_id':1300, 'place_class':'kitchen', 'place_name':u'厨房', 'pos':{'X':3.0,'Y':-9.0,'Yaw':-1.57}},
        {'place_id':1301, 'place_class':'kitchen', 'place_name':u'冰箱', 'pos':{'X':3.29,'Y':-8.41,'Yaw':0}},
        {'place_id':1302, 'place_class':'kitchen', 'place_name':u'架子', 'pos':{'X':3.46,'Y':-10.4,'Yaw':0}},
        {'place_id':1303, 'place_class':'kitchen', 'place_name':u'桌子', 'pos':{'X':1,'Y':-6.9,'Yaw':3.14}},

        {'place_id':1400, 'place_class':'bedroom', 'place_name':u'卧室', 'pos':{'X':5.6,'Y':-1.0,'Yaw':1.57}},
        {'place_id':1401, 'place_class':'bedroom', 'place_name':u'床', 'pos':{'X':6.35,'Y':-0.82,'Yaw':1.57}},
        {'place_id':1402, 'place_class':'bedroom', 'place_name':u'柜子', 'pos':{'X':7.55,'Y':-2.43,'Yaw':0}},
        {'place_id':1403, 'place_class':'bedroom', 'place_name':u'圆桌', 'pos':{'X':5.26,'Y':-2.18,'Yaw':-1.8}},

        {'place_id':1500, 'place_class':'here', 'place_name':u'原位', 'pos':{'X':1.3,'Y':0,'Yaw':0}},
        {'place_id':1600, 'place_class':'out', 'place_name':u'离开', 'pos':{'X':7.37,'Y': -11.2,'Yaw':-1.57}}   ])



    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main_rate.sleep()






