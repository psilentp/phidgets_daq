#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from phidgets_daq.msg import phidgetsDAQmsg, phidgetsDigitalOutput
import numpy as np
import os
import time
import imp

def get_localtime():
    lt = time.localtime()
    return lt.tm_hour + lt.tm_min/60. + lt.tm_sec/3600.

rospy.init_node('phidgets_ssr_publisher', anonymous=True)
publisher = rospy.Publisher('/phidgets_daq/digital_output', phidgetsDigitalOutput)

msg = phidgetsDigitalOutput()
msg.ports = [0]
msg.states = [0]

# load configuration
odor_control_configuration_file = os.path.expanduser(rospy.get_param('/phidgets_daq/odor_control_configuration'))
print 'Loading: ', odor_control_configuration_file
odor_control_configuration = imp.load_source('odor_control_configuration', odor_control_configuration_file)
config = odor_control_configuration.Odor_Control_Configuration()

rospy.sleep(2)

# make sure everything is off
msg.ports = config.ports[0]
msg.states = [0 for i in range(len(msg.ports))]
publisher.publish(msg)
lt = get_localtime()
print lt, msg.ports, msg.states
print

rospy.sleep(2) 

# wait until it is local time (within 2 seconds)
if config.localtime_start != 'now':
    lt = get_localtime()
    while np.abs(lt - config.localtime_start) > .0005:
        #time.sleep(1)
        lt = get_localtime()
        #print lt
        
print "Local time: ", lt

# for each interval in config
for i, interval in enumerate(config.intervals_hrs):
    time.sleep(interval*3600)
    lt = get_localtime()
    msg.ports = config.ports[i]
    msg.states = config.states[i]
    print lt, msg.ports, msg.states
    publisher.publish(msg)
    
    
