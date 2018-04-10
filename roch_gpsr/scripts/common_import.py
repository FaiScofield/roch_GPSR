#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------------------------------
# ROS (import)
#--------------------------------------------------
import rospy
import rosbag

import tf

import smach
import smach_ros

import actionlib

import cv2
#--------------------------------------------------
# ROS msg (from import)
#--------------------------------------------------

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

# from roch_tts.msg import RochTTSAction
# from roch_tts.msg import RochTTSGoal
# from roch_tts.msg import RochTTSFeedback
# from roch_tts.msg import RochTTSResult

from xfei_tts.msg import XFeiTTSAction
from xfei_tts.msg import XFeiTTSGoal
from xfei_tts.msg import XFeiTTSResult

from roch_asr.msg import RochASRAction
from roch_asr.msg import RochASRGoal
from roch_asr.msg import RochASRFeedback
from roch_asr.msg import RochASRResult

from roch_gpsr.msg import SoundEffectAction
from roch_gpsr.msg import SoundEffectGoal
from roch_gpsr.msg import SoundEffectFeedback
from roch_gpsr.msg import SoundEffectResult

from roch_gpsr.msg import TaskRecAction
from roch_gpsr.msg import TaskRecGoal
from roch_gpsr.msg import TaskRecFeedback
from roch_gpsr.msg import TaskRecResult

# from common_pkg.msg import ObjRecAction
# from common_pkg.msg import ObjRecGoal
# from common_pkg.msg import ObjRecFeedback
# from common_pkg.msg import ObjRecResult

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


#--------------------------------------------------
# 系统 (import)
#--------------------------------------------------
import sys
import os
import termios
import tty
import select
import time
import datetime
import threading
import math
import re
import socket

#import pygame # game pad library

import numpy
#--------------------------------------------------
# 系统(from import)
#--------------------------------------------------
from subprocess import call
from subprocess import Popen
from subprocess import PIPE

#from pygame.locals import * # to use the const grup of pygame.locals
