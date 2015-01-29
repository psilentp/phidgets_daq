#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from phidgets_daq.msg import phidgetsDAQinterpreted, phidgetsDAQmsg 
from phidgets_daq.srv import phidgetsDAQservice, phidgetsDAQchannelnames, phidgetsDAQservice_alldata

# Other imports
import imp

###############################################################################
###############################################################################
class PhidgetDAQInterpreter:
    def __init__(self, interpreter):
        '''
        interpreter - an instantiated class of the form found in demo/phidgets_daq_interpreter_functions.py
        '''
        rospy.init_node('phidget_daq_interpreter', anonymous=True)
        self.interpreter = interpreter
        
        # most recent data
        self.most_recent_data = {}
        for channel in self.interpreter.channel_names.values():
            self.most_recent_data.setdefault(channel, [])        
        
        # subscribe to daq
        self.daq_subscription = rospy.Subscriber('/phidgets_daq/raw_data', phidgetsDAQmsg, self.interpret_new_data)
        
        # set up publishers
        self.publishers = {}
        for channel in self.interpreter.channel_names.values():
            topic_name = '/phidgets_daq/' + channel
            p = rospy.Publisher(topic_name, phidgetsDAQinterpreted, queue_size=3)
            self.publishers.setdefault(channel, p)
            
        # set up services (primarily for live plotting)
        self.service_daq = rospy.Service('/phidgets_daq/service', phidgetsDAQservice, self.phidgetsDAQservice_callback)
        self.service_daq_alldata = rospy.Service('/phidgets_daq/service_alldata', phidgetsDAQservice_alldata, self.phidgetsDAQservice_alldata_callback)
        self.service_channelnames = rospy.Service('/phidgets_daq/channel_names', phidgetsDAQchannelnames, self.phidgetsDAQchannelnames_callback)
            
        # spin it
        rospy.spin()
        
    def interpret_new_data(self, data):
        for c, raw_value in enumerate(data.data):
            channel, value = self.interpreter.interpret_channel(c, raw_value)
            self.most_recent_data[channel] = [data.time, value]
            self.publishers[channel].publish(data.time, value)
    
    def phidgetsDAQservice_callback(self, request):
        return self.most_recent_data[request.channel]
        
    def phidgetsDAQservice_alldata_callback(self, request):
        channels = []
        values = []
        t = None
        for channel, data in self.most_recent_data.items():
            print data
            channels.append(channel)
            values.append(data[1])
            if t is None:
                t = data[0]
        print t, channels, values
        return [t, channels, values]
    
    def phidgetsDAQchannelnames_callback(self, request):
        channels = [channel for channel in self.interpreter.channel_names.values()]
        return [channels]
            
if __name__ == '__main__':

    interpreter_path = rospy.get_param('/phidgets_daq/interpreter_path')
    phidgets_interpreter = imp.load_source('phidgets_interpreter', interpreter_path)
    interpreter = phidgets_interpreter.DAQInterpreter()

    p = PhidgetDAQInterpreter(interpreter)

