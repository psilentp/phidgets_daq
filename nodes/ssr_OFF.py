#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from phidgets_daq.msg import phidgetsDAQmsg, phidgetsDigitalOutput
import numpy as np

import time

def get_localtime():
    lt = time.localtime()
    return lt.tm_hour + lt.tm_min/60. + lt.tm_sec/3600.

rospy.init_node('phidgets_ssr_publisher', anonymous=True)


publisher = rospy.Publisher('/phidgets_daq/digital_output', phidgetsDigitalOutput)

rospy.sleep(2)

air_ssr_time_on = 0
odor_ssr_time_on = 0

air_ssr = 0
odor_ssr = 0

msg = phidgetsDigitalOutput()
msg.ports = [0]
msg.states = [0]


msg.ports = [0]
msg.states = [0]
publisher.publish(msg)

msg.ports = [1]
msg.states = [0]
publisher.publish(msg)
