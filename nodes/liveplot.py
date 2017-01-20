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

class Scope:
    def __init__(self, fig, interpreter, maxt=2):
        self.interpreter = interpreter
        self.maxt = interpreter.maxt
        self.fig = fig
        
        # phidgets daq service
        service_name = '/phidgets_daq/service_alldata'
        rospy.wait_for_service(service_name)
        self.get_phidgets_data = rospy.ServiceProxy(service_name, phidgetsDAQservice_alldata)
        
        # get channel names
        service_name = '/phidgets_daq/channel_names'
        rospy.wait_for_service(service_name)
        self.get_phidgets_daq_channel_names = rospy.ServiceProxy(service_name, phidgetsDAQchannelnames)
        self.channels = self.get_phidgets_daq_channel_names().channels
        print 'Channels: ', self.channels
        
        self.channels = self.get_phidgets_daq_channel_names().channels
        print 'Channels: ', self.channels
        
        # set up axes
        self.axes = {}
        for c, channel in enumerate(self.channels):
            ax = fig.add_subplot(len(self.channels), 1, c+1)
            ax.set_ylim(self.interpreter.channel_param[channel]['min'], self.interpreter.channel_param[channel]['max'])
            self.axes.setdefault(channel, ax)
            
        self.data = {}
        self.lines = {}
        for c, channel in enumerate(self.channels):
            self.data.setdefault(channel, {})
            self.data[channel].setdefault('time', [0])
            self.data[channel].setdefault('values', [0])
            self.lines[channel] = Line2D(self.data[channel]['time'], self.data[channel]['values'])
            self.axes[channel].add_line(self.lines[channel])
            #self.axes[channel].set_ylim(-1,1)
            self.axes[channel].set_xlim(0, self.maxt)
            self.axes[channel].set_ylabel(channel)

    def update(self, y):
        lines = []
        new_data = self.get_phidgets_data()
        for c, channel in enumerate(new_data.channels):
            
            if new_data.time > self.data[channel]['time'][0] + self.maxt: # reset the arrays
                print new_data.time, self.data[channel]['time'][0], self.maxt
                self.data[channel]['time'] = [new_data.time]
                self.data[channel]['values'] = [new_data.values[c]]
                self.axes[channel].set_xlim(self.data[channel]['time'][0], self.data[channel]['time'][0]+self.maxt)
                self.axes[channel].figure.canvas.draw()

            else:
                self.data[channel]['time'].append(new_data.time)
                self.data[channel]['values'].append(new_data.values[c])
            
            self.lines[channel].set_data( self.data[channel]['time'], self.data[channel]['values'] )          
            lines.append(self.lines[channel])
                     
             
        return lines

if __name__ == '__main__':
    
    interpreter_path = rospy.get_param('/phidgets_daq/interpreter_path')
    phidgets_interpreter = imp.load_source('phidgets_interpreter', interpreter_path)
    interpreter = phidgets_interpreter.DAQInterpreter()

    fig = plt.figure()
    scope = Scope(fig, interpreter)
    ani = animation.FuncAnimation(fig, scope.update, interval=20, blit=True)
    plt.show()
