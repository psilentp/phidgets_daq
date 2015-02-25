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

air_ssr = 0
odor_ssr = 0

msg = phidgetsDigitalOutput()
msg.ports = [0]
msg.states = [0]

lt = get_localtime()

##
air_on = rospy.get_param('/phidgets_ssr_publisher_localtimes/air_on')
air_off = rospy.get_param('/phidgets_ssr_publisher_localtimes/air_off')
odor_on = rospy.get_param('/phidgets_ssr_publisher_localtimes/odor_on')
odor_off = rospy.get_param('/phidgets_ssr_publisher_localtimes/odor_off')
##
print 'lt: ', lt
print air_on, air_off
print odor_on, odor_off

while air_ssr < 2 or odor_ssr < 2:
    lt = get_localtime()

    # air
    if lt > air_on and air_ssr == 0:
        air_ssr = 1
        msg.ports = [0]
        msg.states = [1]
        publisher.publish(msg)
        print lt, ' : air ssr turned ON'
    if lt > air_off and air_ssr == 1:
        air_ssr = 2
        msg.ports = [0]
        msg.states = [0]
        publisher.publish(msg)
        print lt, ' : air ssr turned OFF'
        
    # odor
    if lt > odor_on and odor_ssr == 0:
        odor_ssr = 1
        msg.ports = [1]
        msg.states = [1]
        publisher.publish(msg)
        print lt, ' : odor ssr turned ON'
    if lt > odor_off and odor_ssr == 1:
        odor_ssr = 2
        msg.ports = [1]
        msg.states = [0]
        publisher.publish(msg)
        print lt, ' : odor ssr turned OFF'
    
    rospy.sleep(1)
    
    
