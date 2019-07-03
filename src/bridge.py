#!/usr/bin/env python

import argparse
import sys

import time
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput, BridgeGain

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

class ROSPhidgetBridgeChannel():
    def __init__(self,
                 bridge_channel,
                 topic,
                 frame_id,
                                
                 bridge_serial = 572843,
                 bridge_hubport = 0,
                 bridge_hubportdevice = False,
                 data_rate_Hz = 60,
                 gain = BridgeGain.BRIDGE_GAIN_128,
                 force_conversion_multiple = 1.0,
                 force_conversion_offset = 0.0):

        self.data_rate_Hz = data_rate_Hz
        self.data_interval_ms = int((1.0/self.data_rate_Hz)*1000)
        self.publishcount = 0

        self.force_conversion_multiple = float(force_conversion_multiple)
        self.force_conversion_offset = float(force_conversion_offset)
        self.gain = gain
        
        self.pb_ch = VoltageRatioInput()
        self.pb_ch.setDeviceSerialNumber(bridge_serial)
        self.pb_ch.setHubPort(bridge_hubport)
        self.pb_ch.setIsHubPortDevice(bridge_hubportdevice)
        self.pb_ch.setChannel(int(bridge_channel))


        rospy.init_node("phidget_bridge_channel", anonymous=True)
        self.pub = rospy.Publisher(topic, WrenchStamped, queue_size=10)
        self.frame_id = frame_id

        self.pb_ch.setOnAttachHandler(self.onAttachHandler)
        self.pb_ch.setOnDetachHandler(self.onDetachHandler)
        self.pb_ch.setOnVoltageRatioChangeHandler(self.onVoltageRatioChangeHandler)
    
    def start(self):
        rospy.loginfo("phidget bridge channel node started")
        rospy.loginfo("waiting to be attached")
        self.pb_ch.openWaitForAttachment(1000)


    def force(self, voltage_ratio):
        return (voltage_ratio*self.force_conversion_multiple) + self.force_conversion_offset

    def onAttachHandler(self, channel):
        rospy.loginfo("phidget bridge serial "+str(channel.getDeviceSerialNumber())+" channel "+str(channel.getChannel())+" attached")
        channel.setDataInterval(self.data_interval_ms)
        channel.setVoltageRatioChangeTrigger(0.0)
        channel.setBridgeGain(self.gain)
        channel.setBridgeEnabled(True)

    def onDetachHandler(self, channel):
        rospy.loginfo("phidget bridge serial "+str(channel.getDeviceSerialNumber())+" channel "+str(channel.getChannel())+" detached")

    def onVoltageRatioChangeHandler(self, channel, voltageRatio):
        self.pub.publish(
            WrenchStamped(
                Header(
		    self.publishcount,
                    rospy.Time.now(),
                    self.frame_id),
                Wrench(
                    Vector3(self.force(voltageRatio), 0, 0),
                    Vector3(0,0,0))))

        self.publishcount += 1

if __name__ == "__main__":
    argv = rospy.myargv(sys.argv)
    pbch0 = ROSPhidgetBridgeChannel(argv[1],
                                   argv[2],
                                   argv[3],
                                   force_conversion_multiple = argv[4],
                                   force_conversion_offset = argv[5])
    pbch1 = ROSPhidgetBridgeChannel(argv[6],
                                   argv[7],
                                   argv[8],
                                   force_conversion_multiple = argv[9],
                                   force_conversion_offset = argv[10])
    pbch0.start()
    pbch1.start()
    rospy.spin()