#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16,Float32,Float64
from tiburon.msg import ins_data
from dynamic_reconfigure.server import Server
from tiburon.cfg import depthparamsConfig
import datetime,time
from PID import PID
from vehicle import VehicleParams

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

        '''
        Bottom w.r.t. to bot is positive z
        Forward w.r.t. to bot is positive y
        Right wrt to forward direction of bot is positive x

        F1 - forward, F2 - backward, F3 - left, F4 - right

        Equations of motion are:-
        F1 cos θ + F2 cos θ + F3 sin θ + F4 sin θ = ma + B - mg = U1 + B - mg
        - F1 sin θ - F2 sin θ + F3 cos θ + F4 cos θ = 0 = U2
        - F1 x1 + F2 x2 = Ixx αxx = U3
        F3 x3 - F4 x4 = Izz αzz = U4

        Matrix form
        #  ==                                  ==   ==    ==         ==            ==
        |   cos θ    cos θ    sin θ    sin θ   | |   F1   |       |   U1 + B - mg  |
        |  -sin θ   -sin θ    cos θ    cos θ   | |   F2   |  ===  |       00       |
        |   -x1       x2       00       00     | |   F3   |  ===  |       U3       |
        |    00       00       x3      -x4     | |   F4   |       |       U4       |
         ==                                  ==   ==    ==         ==            ==

        '''





if __name__=='__main__':
    main()
