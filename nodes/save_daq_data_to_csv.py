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
        self.filename = rospy.get_param('/phidgets_daq/csv_data_filename', '~/phidgets_daq_data.csv')
        
        # recording interval
        self.localtimes_to_record_data = np.arange(0,24)
        self.time_to_sleep_for = 60 # seconds
        self.last_time_data_recorded = 0
        
        # phidgets daq service
        service_name = '/phidgets_daq/service_alldata'
        rospy.wait_for_service(service_name)
        self.get_phidgets_data = rospy.ServiceProxy(service_name, phidgetsDAQservice_alldata)

    def get_localtime_list(self):
        row = []
        lt = time.localtime()
        row.append('year')
        row.append(lt.tm_year)
        row.append('month')
        row.append(lt.tm_mon)
        row.append('day')
        row.append(lt.tm_mday)
        row.append('hour')
        row.append(lt.tm_hour)
        row.append('minute')
        row.append(lt.tm_min)
        return row

    def collect_row_data_to_write(self):
        row = self.get_localtime_list()
        new_data = self.get_phidgets_data()

        row.append('rostime')
        row.append(new_data.time)
        
        for c, channel in enumerate(new_data.channels):
            row.append(channel)
            row.append(new_data.values[c])
            
        return row
        

    def write_row_to_file(self, row):
        print 'filename: ', self.filename
        self.csvfile = open(os.path.expanduser(self.filename), 'a')
        self.datawrite = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        self.datawrite.writerow(row)
        self.csvfile.close()
        
    def run(self):
        while (not rospy.is_shutdown()):
            lt_dec = get_localtime_as_a_decimal()
            diff = np.min( np.abs(self.localtimes_to_record_data - lt_dec) )
            print lt_dec, diff
            if diff < 2*self.time_to_sleep_for/3600.:
                if (lt_dec - self.last_time_data_recorded) > 4*self.time_to_sleep_for/3600.:
                    # save data!
                    print 'saving data!'
                    row = self.collect_row_data_to_write()
                    self.write_row_to_file(row)
                    self.last_time_data_recorded = lt_dec
            
            time.sleep(self.time_to_sleep_for)
        
        
if __name__ == '__main__':
    record_data = RecordData()
    record_data.run()
        
