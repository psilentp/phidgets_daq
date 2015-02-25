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
        rospy.init_node('phidgets_ssr', anonymous=True)
        
    def initialize_digital_outputs(self, phidget):
        '''
        subscribes to /phidgets_daq/digital_outputs topic, and connects received messages with setting the output on the phidget
        '''
        self.phidget = phidget
        self.daq_subscription = rospy.Subscriber('/phidgets_daq/digital_output', phidgetsDigitalOutput, self.set_digital_output)

    def set_digital_output(self, digital_output):
        for i in range(len(digital_output.ports)):
            self.phidget.setOutputState(digital_output.ports[i], digital_output.states[i])    
    
    def run(self):
        rospy.spin()
        
def connect_to_phidget(SensorChangedFunction, serial_number=None):
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
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Opening phidget object....")

    try:
        if serial_number is not None:
            interfaceKit.openPhidget(serial_number)
        else:
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

    phidget = interfaceKit
    
    return phidget

if __name__ == '__main__':
    
    print 'connecting to phidget'
    phidgets_daq = PhidgetsDAQ()
    serial_number = rospy.get_param('/phidgets_ssr/serial_number', None)
    phidget = connect_to_phidget(serial_number)
    
    phidgets_daq.initialize_digital_outputs(phidget)
    phidgets_daq.run()

