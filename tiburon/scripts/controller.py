#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16,Float32,Float64
from tiburon.msg import ins_data
from dynamic_reconfigure.server import Server
from tiburon.cfg import depthparamsConfig
import datetime,time
from PID import PID

depthController = PID()
pitchController = PID()
yawController  = PID()

depthRecieved = 0
pitchAndYawRecieved = 0

def depthCallback(msg):
    if msg.data<=1300 and msg.data>980
        depthController.currentVal = msg.data
    if depthRecieved == 0:
        # Setting the checkpoint for depth as first recieved value
        depthController.checkpoint = depthController.currentVal
    depthRecieved = 1

def insCallback(msg):
    pitchController.currentVal = msg.YPR.y
    yawController.currentVal = msg.YPR.x
    if pitchAndYawRecieved == 0:
        # Setting the initial checkpoint for pitch as 0 and yaw as first recieved value
        pitchController.checkpoint = 0
        yawController.checkpoint = yawController.currentVal
    pitchAndYawRecieved = 1

depthDataSub=rospy.Subscriber("/depth_value",Float64,depthCallback)
insDataSub=rospy.Subscriber("/tiburon/ins_data",ins_data,insCallback)

def main():
    while(True):
        # If either value is still not recieved, do nothing
        if depthRecieved==0 or pitchAndYawRecieved==0:
            continue
        # Get the PID control signal for all
        depthU = depthController.getError()
        pitchU = pitchController.getError()
        yawU = yawController.getError()
        forwardU = 0

        





if __name__=='__main__':
    main()
