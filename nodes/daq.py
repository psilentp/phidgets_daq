#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from phidgets_daq.msg import phidgetsDAQmsg, phidgetsDigitalOutput
from phidgets_daq.srv import phidgetsDAQchannelnames

#Basic imports
from ctypes import *
import time
import numpy as np
import sys
import random

#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, OutputChangeEventArgs, SensorChangeEventArgs
from Phidgets.Devices.InterfaceKit import InterfaceKit

###############################################################################
###############################################################################
class PhidgetsDAQ:

    def __init__(self):
        rospy.init_node('phidgets_daq', anonymous=True)
        self.previous_update = rospy.Time.now()
        self.start_time = rospy.Time.now().to_sec()
        
        # get channel names
        service_name = '/phidgets_daq/channel_names'
        print 'Waiting for service: ', service_name
        print 'If nothing is happening, make sure phidgets_interpreter.py is running'
        rospy.wait_for_service(service_name)
        self.get_phidgets_daq_channel_names = rospy.ServiceProxy(service_name, phidgetsDAQchannelnames)
        self.channel_names = self.get_phidgets_daq_channel_names().channels
        self.channels = [i for i in range(len(self.channel_names))]
        print 'Channels: ', self.channels
        
        # parameters 
        try:
            self.update_rate_ms = rospy.get_param("/phidgets_daq/update_rate_ms")
        except:
            self.update_rate_ms = 2
            
        # initialize sensor times
        self.sensor_time = {}
        self.key_frame_interval = {}
        self.last_key_frame = {}
        for channel in self.channels:
            self.sensor_time.setdefault(channel, rospy.Time.now())
            self.key_frame_interval.setdefault(channel, rospy.Duration(.008))
            self.last_key_frame.setdefault(channel, rospy.Time.now())
            
        # initialize buffer
        self.buffer = {}
        for channel in self.channels:
            self.buffer.setdefault(channel, [])
            
        # publish data  
        self.publish_data = rospy.Publisher('/phidgets_daq/raw_data', phidgetsDAQmsg, queue_size=3)
        
    def initialize_digital_outputs(self, phidget):
        '''
        subscribes to /phidgets_daq/digital_outputs topic, and connects received messages with setting the output on the phidget
        '''
        self.phidget = phidget
        self.daq_subscription = rospy.Subscriber('/phidgets_daq/digital_output', phidgetsDigitalOutput, self.set_digital_output)

    def set_digital_output(self, digital_output):
        for i in range(len(digital_output.ports)):
            self.phidget.setOutputState(digital_output.ports[i], digital_output.states[i])    
    
    def buffer_is_full(self):
        vals = [len(val) for val in self.buffer.values()]
        if np.min(vals) > 0:
            return True
        else:
            return False
        
    def run(self):
        '''
        runs a ros loop that checks for a full buffer (e.g. all analog inputs have new data) at twice the update rate.  
        the full buffer is then publishes, so all sensor data goes out at the same time.
        '''
        hz = 1000/(self.update_rate_ms)
        rate = rospy.Rate(2*hz) # check for new values at twice the update rate
        while not rospy.is_shutdown():
            if self.buffer_is_full():
                times = []
                values = []
                for channel in self.channels:
                    data = self.buffer[channel].pop(0)
                    times.append(data[0].to_sec()-self.start_time)
                    values.append(data[1])
                    
                data = [int(v) for v in values]
                msg = phidgetsDAQmsg(time=np.mean(times), data=data)
                self.publish_data.publish(msg)
            rate.sleep()
        
    def interfaceKitSensorChanged(self, e):
            source = e.device
            serialnum = source.getSerialNum()
            channel = e.index
            if channel not in self.channels:
                return
            value = e.value
            now = rospy.Time.now()
            
            interval = now - self.previous_update
            self.previous_update = now 

            if interval > rospy.Duration(0.007): # it's a key frame (new packet)
                self.key_frame_interval[channel] = now - self.last_key_frame[channel]
                self.sensor_time[channel] = rospy.Time.now()
                self.last_key_frame[channel] = now
            else:
                self.sensor_time[channel] += (self.key_frame_interval[channel]/(8./self.update_rate_ms))
                        
            data = [self.sensor_time[channel], value]
            self.buffer[channel].append(data)
            #print data
            
def connect_to_phidget(SensorChangedFunction, update_rate_ms=2, channels=[0,1,2,3,4,5,6,7]):
    #Create an interfacekit object
    try:
        interfaceKit = InterfaceKit()
    except RuntimeError as e:
        print("Runtime Exception: %s" % e.details)
        print("Exiting....")
        exit(1)

    #Information Display Function
    def displayDeviceInfo():
        print("|------------|----------------------------------|--------------|------------|")
        print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
        print("|------------|----------------------------------|--------------|------------|")
        print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (interfaceKit.isAttached(), interfaceKit.getDeviceName(), interfaceKit.getSerialNum(), interfaceKit.getDeviceVersion()))
        print("|------------|----------------------------------|--------------|------------|")
        print("Number of Digital Inputs: %i" % (interfaceKit.getInputCount()))
        print("Number of Digital Outputs: %i" % (interfaceKit.getOutputCount()))
        print("Number of Sensor Inputs: %i" % (interfaceKit.getSensorCount()))

    #Event Handler Callback Functions
    def interfaceKitAttached(e):
        attached = e.device
        print("InterfaceKit %i Attached!" % (attached.getSerialNum()))

    def interfaceKitDetached(e):
        detached = e.device
        print("InterfaceKit %i Detached!" % (detached.getSerialNum()))

    def interfaceKitError(e):
        try:
            source = e.device
            print("InterfaceKit %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))

    #Main Program Code
    try:
        interfaceKit.setOnAttachHandler(interfaceKitAttached)
        interfaceKit.setOnDetachHandler(interfaceKitDetached)
        interfaceKit.setOnErrorhandler(interfaceKitError)
        interfaceKit.setOnSensorChangeHandler(SensorChangedFunction)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Opening phidget object....")

    try:
        interfaceKit.openPhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Waiting for attach....")

    try:
        interfaceKit.waitForAttach(10000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        try:
            interfaceKit.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        print("Exiting....")
        exit(1)
    else:
        displayDeviceInfo()

    print("Setting the data rate for each sensor index to ", update_rate_ms, " ms....")
    for i in range(interfaceKit.getSensorCount()):
        if i in channels:
            try:
                interfaceKit.setDataRate(i, update_rate_ms)
                interfaceKit.setSensorChangeTrigger(i, 0)
            except PhidgetException as e:
                print("Phidget Exception %i: %s" % (e.code, e.details)) 
        else:
            try:
                interfaceKit.setSensorChangeTrigger(i, 1000)
            except PhidgetException as e:
                print("Phidget Exception %i: %s" % (e.code, e.details)) 
        
    phidget = interfaceKit

    return phidget

if __name__ == '__main__':
    
    print 'connecting to phidget'
    phidgets_daq = PhidgetsDAQ()
    phidget = connect_to_phidget(phidgets_daq.interfaceKitSensorChanged,
                                 update_rate_ms=phidgets_daq.update_rate_ms, 
                                 channels=phidgets_daq.channels)
    
    phidgets_daq.initialize_digital_outputs(phidget)
    phidgets_daq.run()

