#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from phidgets_daq.msg import phidgetsDAQmsg, phidgetsDigitalOutput
import time

def get_float_local_time_hours():
    t = time.localtime(time.time())
    lt = t.tm_hour + t.tm_min/60. + t.tm_sec/3600.
    return lt

class AlicatFlowController:
    def __init__(self,  pulse_interval=2400, 
                        pulse_length=600, 
                        first_pulse_time=-1,
                        first_pulse_delay=0,
                        publish_name='/alicat_flow_control'):
        self.pulse_interval = pulse_interval
        self.pulse_length = pulse_length
        self.publisher_alicat = rospy.Publisher(publish_name, Float32, queue_size=10)
        self.first_pulse_time = first_pulse_time
        self.first_pulse_delay = first_pulse_delay
        
        self.solenoid_states = [1,0]
        self.solenoid_ports = [0,1]
        self.phidgets_ssr_publisher = rospy.Publisher('/phidgets_daq/digital_output', phidgetsDigitalOutput, queue_size=30)

    def main(self,flow_rate=5):
        rospy.sleep(2)
        print 'running'
        self.publisher_alicat.publish(0)
        msg = phidgetsDigitalOutput()
        msg.ports = self.solenoid_ports
        msg.states = [0,0]
        self.phidgets_ssr_publisher.publish(msg)
        print 'off'
        
        # first pause until local time reached
        if self.first_pulse_time > 0:
            rate = rospy.Rate(0.25)
            print 'here'
            while not rospy.is_shutdown():
                lt = get_float_local_time_hours()
                if np.abs(lt-self.first_pulse_time) < 1:
                    break
    
        rospy.sleep(self.first_pulse_delay)
        rate = rospy.Rate(1 / float(self.pulse_interval) ) 
        while not rospy.is_shutdown():
        
            if type(flow_rate) is list:
                index = np.random.randint(len(flow_rate))
                f = flow_rate[index]
            else:
                f = flow_rate

            # phidgets solenoid
            msg = phidgetsDigitalOutput()
            msg.ports = self.solenoid_ports
            
            if self.solenoid_states == [1,0]:
                self.solenoid_states = [0,1]
            else:
                self.solenoid_states = [0,1]

            msg.states = self.solenoid_states
            lt = get_float_local_time_hours()
            print lt, ' solenoids: ', self.solenoid_states 
            self.phidgets_ssr_publisher.publish(msg)
            #
            
            # alicat flow rate
            self.publisher_alicat.publish(f)
            print 'flow rate: ', flow_rate
            #

            # pulse length            
            rospy.sleep(self.pulse_length)
            
            # turn off solenoids and alicat
            msg.states = [0,0]
            self.phidgets_ssr_publisher.publish(msg)
            self.publisher_alicat.publish(0)
            lt = get_float_local_time_hours()
            print lt, ' flow rate: ', 0
            
            
           
            
            
            rate.sleep()
            
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--first_pulse_time", type="int", dest="first_pulse_time", default=-1,
                        help="time (localtime in hours) to send first pulse, defaults to -1 which means now")
    parser.add_option("--first_pulse_delay", type="int", dest="first_pulse_delay", default=0,
                        help="number of seconds to wait before sending first pulse")
    parser.add_option("--pulse_interval", type="int", dest="pulse_interval", default=2400,
                        help="pulse interval")
    parser.add_option("--pulse_length", type="int", dest="pulse_length", default=600,
                        help="pulse length")
    parser.add_option("--publish_name", type="str", dest="publish_name", default='/alicat_flow_control',
                        help="topic to publish to")
    parser.add_option("--flow_rate", type="float", dest="flow_rate", default=5,
                        help="flow rate")
        
    
    (options, args) = parser.parse_args()
    
    rospy.init_node('alicat_phidgets_ros_flow_controller')
    alicat_flow_controller = AlicatFlowController(options.pulse_interval, options.pulse_length, options.first_pulse_time, options.first_pulse_delay, options.publish_name)
    alicat_flow_controller.main(flow_rate=options.flow_rate)
            
            
