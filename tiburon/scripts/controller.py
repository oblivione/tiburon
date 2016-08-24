#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16,Float32,Float64
from tiburon.msg import ins_data
from dynamic_reconfigure.server import Server
from tiburon.cfg import depthparamsConfig
import datetime,time
from PID import PID

currentDepth, currentPitch, currentYaw = -1, -1, -1
depthCheckpoint, pitchCheckpoint, yawCheckpoint = -1, -1, -1
KpDepth, KiDepth, KdDepth = 0.00, 0.00, 0.00
KpYaw  , KiYaw  , KdYaw   = 0.00, 0.00, 0.00
KpPitch, KiPitch, KdPitch = 0.00, 0.00, 0.00

depthController = PID()
pitchController = PID()
yawController  = PID()

depthController.currentVal = -1
depthController.checkpoint 

def depthCallback(msg):
    if msg.data>1300 or msg.data<980
        currentDepth = currentDepth
    else
        currentDepth = msg.data

def insCallback(msg):
    pitchCheckpoint = msg.YPR.y
    yawCheckpoint   = msg.YPR.x


frontPitchPub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
backPitchPub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
depthDataSub=rospy.Subscriber("/depth_value",Float64,depthCallback)
insDataSub=rospy.Subscriber("/tiburon/ins_data",ins_data,insCallback)
