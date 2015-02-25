#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam

import imp

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from phidgets_daq.srv import phidgetsDAQservice_alldata, phidgetsDAQchannelnames

import time, os, csv

def get_localtime_as_a_decimal():
    lt = time.localtime()
    lt_dec = lt.tm_hour + lt.tm_min/60. + lt.tm_sec/3600.
    return lt_dec

class RecordData:
    def __init__(self):
        self.filename = time.strftime("%Y%m%d_%H_%M_%S_data.csv", time.localtime())
        print 'filename: ', self.filename
        self.csvfile = open(os.path.expanduser(self.filename), 'w')
        self.datawrite = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        
        # get channel names
        service_name = '/phidgets_daq/channel_names'
        rospy.wait_for_service(service_name)
        self.get_phidgets_daq_channel_names = rospy.ServiceProxy(service_name, phidgetsDAQchannelnames)
        self.channels = self.get_phidgets_daq_channel_names().channels
        print 'Channels: ', self.channels
        
        self.row_data = ['ros_time_secs', 'ros_time_nsecs', 'time']
        for channel in self.channels:
            self.row_data.append(channel)
        self.datawrite.writerow(self.row_data)
        
        self.sub = rospy.Subscriber('/phidgets_daq/all_data', phidgetsDAQalldata, self.save_new_data)
        

    def save_new_data(self, data):
        row = []
        row.append(rospy.Time.secs)
        row.append(rospy.Time.nsecs)
        row.append(data.time)
        
        channel_data = {}
        for c, channel in enumerate(data.channels):
            channel_data.setdefault(channel, data.values[c])
        
        for channel in self.channels:
            row.append(channel_data[channel])
        
        self.datawrite.writerow(self.row_data)
    
    def run(self):
        rospy.spin()
        self.csvfile.close()
        
if __name__ == '__main__':
    record_data = RecordData()
    record_data.run()
        
