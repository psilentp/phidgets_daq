#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from phidgets_daq.msg import phidgetsDAQmsg, phidgetsDigitalOutput
import numpy as np

rospy.init_node('phidgets_ssr', anonymous=True)


publisher = rospy.Publisher('/phidgets_daq/digital_output', phidgetsDigitalOutput)



hz = 3
rate = rospy.Rate(hz) # check for new values at twice the update rate

msg = phidgetsDigitalOutput()
msg.ports = [0]
msg.states = [0]
while not rospy.is_shutdown():
    if np.mean(msg.states) == 0:
        msg.states = [1]
    else:
        msg.states = [0]
    
    publisher.publish(msg)
    rate.sleep()
        
