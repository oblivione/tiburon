#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16,Float32,Float64
from tiburon.msg import ins_data
from geometry_msgs.msg import Vector3
from dynamic_reconfigure.server import Server
from tiburon.cfg import yawPitchDepthConfig
import datetime,time
import math
import numpy as np
from PID import PID
from AngularPID import AngularPID
from vehicle import VehicleParams

depthController = PID()
forwardController = PID()
pitchController = AngularPID()
yawController  = AngularPID()

depthReceived = 0
pitchAndYawReceived = 0
velRecieved = 0

def velCallback(msg):
    global forwardController, velRecieved
    if pitchAndYawReceived:
        velx = msg.x
        velz = msg.z
        curPitch = pitchController.currentVal
        forwardController.currentVal = velx*math.cos(curPitch*math.pi/180) - velz*math.sin(curPitch*math.pi/180)
        velRecieved = 1

def depthCallback(msg):
    print "Current D",msg.data
    global depthController, depthReceived
    if msg.data<=1300 and msg.data>950:
        depthController.currentVal = msg.data
    if depthReceived == 0:
        # Setting the checkpoint for depth as first Received value
        depthController.checkpoint = depthController.currentVal
    depthReceived = 1

def insCallback(msg):
    print "Current P: ",msg.YPR.y,"Current Y: ",msg.YPR.x
    global pitchController, yawController, pitchAndYawReceived
    pitchController.currentVal = msg.YPR.y
    yawController.currentVal = msg.YPR.x
    if pitchAndYawReceived == 0:
        # Setting the initial checkpoint for pitch as 0 and yaw as first Received value
        pitchController.checkpoint = 0
        yawController.checkpoint = yawController.currentVal
    pitchAndYawReceived = 1

def depthSPUpdate(msg):
    depthController.checkpoint = msg.data

def yawSPUpdate(msg):
    yawController.checkpoint = msg.data

def pitchSPUpdate(msg):
    pitchController.checkpoint = msg.data

def velSPUpdate(msg):
    forwardController.checkpoint = msg.data

def callback(config,level):
    global depthController, pitchController, yawController
    depthController.Kp = config.kp_depth/400.0
    depthController.Kd = config.kd_depth
    pitchController.Kp = config.kp_pitch/30.0
    pitchController.Kd = config.ki_pitch
    yawController.Kp = config.kp_yaw/25.0
    yawController.Kd = config.kd_yaw
    forwardController.Kp = config.kp_yaw/50.0
    forwardController.Kd = config.kd_yaw
    if depthController.Ki != config.ki_depth:
        depthController.Ki = config.ki_depth/1000.0
        depthController.IError = 0.00
    if pitchController.Ki != config.ki_pitch:
        pitchController.Ki = config.ki_pitch/1000.0
        pitchController.IError = 0.00
    if yawController.Ki != config.ki_yaw:
        yawController.Ki = config.ki_yaw/1000.0
        yawController.IError = 0.00
    if forwardController.Ki != config.ki_for:
        forwardController.Ki = config.ki_for/1000.0
        forwardController.IError = 0.00
    depthController.checkpoint = config.setpoint_depth
    return config

depthDataSub=rospy.Subscriber("/depth_value",Float64,depthCallback,queue_size=1)
insDataSub=rospy.Subscriber("/tiburon/ins_data",ins_data,insCallback,queue_size=1)
deltaVelSub=rospy.Subscriber("/tiburon/true_velocity",Vector3,velCallback,queue_size=1)
frontPitchPub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
backPitchPub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
sideLeftSpeedPub=rospy.Publisher("/sideleftspeed",UInt16,queue_size=1)
sideRightSpeedPub=rospy.Publisher("/siderightspeed",UInt16,queue_size=1)
depthSetpointSub=rospy.Subscriber("/depthSP",UInt16,depthSPUpdate,queue_size=1)
yawSetpointSub=rospy.Subscriber("/yawSP",Float64,yawSPUpdate,queue_size=1)
pitchSetpointSub=rospy.Subscriber("/pitchSP",Float64,pitchSPUpdate,queue_size=1)
velSetpointSub=rospy.Subscriber("/velSP",Float64,velSPUpdate,queue_size=1)

def main():
    global frontPitchPub, backPitchPub, sideLeftSpeedPub, sideRightSpeedPub
    global depthDataSub, insDataSub
    global depthController, pitchController, yawController

    vp = VehicleParams()
    rospy.init_node("tiburonController",anonymous=False)
    srv=Server(yawPitchDepthConfig,callback)
    rate = rospy.Rate(200) # 200 Hz
    thruster1 = thruster2 = thruster3 = thruster4 = 0.0
    frontSpeeds = []
    backSpeeds = []
    leftSpeeds = []
    rightSpeeds = []
    pos = 0
    for i in range(10):
        frontSpeeds.append(1500)
        backSpeeds.append(1500)
        leftSpeeds.append(1500)
        rightSpeeds.append(1500)
    printcounter = 0
    while not rospy.is_shutdown():
        # If either value is still not Received, do nothing
        if depthReceived==0 or pitchAndYawReceived==0 or velRecieved == 0:
            continue
        # Get the PID control signal for all
        depthU = depthController.getError()
        pitchU = pitchController.getError()
        yawU = yawController.getError()
        forwardU = forwardController.getError()
        targetFrontSpeed = targetBackSpeed = targetLeftSpeed = targetRightSpeed = 1500
        '''
        Bottom w.r.t. to bot is positive z
        Forward w.r.t. to bot is positive y
        Left wrt to forward direction of bot is positive x

        F1 - forward, F2 - backward, F3 - left, F4 - right

        Equations of motion are:-
          F1 cos t + F2 cos t + F3 sin t + F4 sin t = ma + B - mg    = U1 + B - mg
        - F1 sin t - F2 sin t + F3 cos t + F4 cos t = 0              = U2
        - F1 x1    + F2 x2                          = Ixx Axx - B xB = U3 - B xB
                                F3 x3    - F4 x4    = Izz Azz        = U4 + Jw1 + Jw2

        Matrix form
         ==                                  ==   ==    ==         ==            ==
        |   cos t    cos t    sin t    sin t   | |   F1   |       |   U1 + B - mg  |
        |  -sin t   -sin t    cos t    cos t   | |   F2   |  ===  |       00       |
        |   -x1       x2       00       00     | |   F3   |  ===  |   U3 - B xB    |
        |    00       00       x3      -x4     | |   F4   |       | U4 + Jw1 + Jw2 |
         ==                                  ==   ==    ==         ==            ==

        '''

        _sin = math.sin(pitchController.currentVal/180 * math.pi)
        _cos = math.cos(pitchController.currentVal/180 * math.pi)
        A = np.array([[_cos, _cos, _sin, _sin], [-_sin, -_sin, _cos, _cos], [-vp.x1, vp.x2, 0, 0], [0, 0, vp.x3, -vp.x4]])
        b = np.array([depthU+ vp.B - vp.weight, forwardU, pitchU + vp.B * vp.xB, yawU + vp.J*thruster1 + vp.J*thruster2])
        F1,F2,F3,F4 = np.linalg.solve(A,b)

        if F1>=vp.thrusterMin:
            targetFrontSpeed = (((F1 - vp.thrusterMin)/vp.thrusterRatioF)**0.5)+1520
        elif F1<=-vp.thrusterMin*0.7:
            targetFrontSpeed = 1480 - (((-F1-vp.thrusterMin*0.7)/vp.thrusterRatioB)**0.5)

        if F2>=vp.thrusterMin:
            targetBackSpeed = (((F2 - vp.thrusterMin)/vp.thrusterRatioF)**0.5)+1520
        elif F2<=-vp.thrusterMin*0.7:
            targetBackSpeed = 1480 - (((-F2-vp.thrusterMin*0.7)/vp.thrusterRatioB)**0.5)

        if F3>=vp.thrusterMin:
            targetLeftSpeed = (((F3 - vp.thrusterMin)/vp.thrusterRatioF)**0.5)+1520
        elif F3<=-vp.thrusterMin*0.7:
            targetLeftSpeed = 1480 - (((-F3-vp.thrusterMin*0.7)/vp.thrusterRatioB)**0.5)

        if F4>=vp.thrusterMin:
            targetRightSpeed = (((F4 - vp.thrusterMin)/vp.thrusterRatioF)**0.5)+1520
        elif F4<=-vp.thrusterMin*0.7:
            targetRightSpeed = 1480 - (((-F4-vp.thrusterMin*0.7)/vp.thrusterRatioB)**0.5)

        if(targetFrontSpeed>1650):
            targetFrontSpeed = 1650
        elif(targetFrontSpeed<1350):
            targetFrontSpeed = 1350

        if(targetBackSpeed>1650):
            targetBackSpeed = 1650
        elif(targetBackSpeed<1350):
            targetBackSpeed = 1350

        if(targetLeftSpeed>1650):
            targetLeftSpeed = 1650
        elif(targetLeftSpeed<1350):
            targetLeftSpeed = 1350

        if(targetRightSpeed>1650):
            targetRightSpeed = 1650
        elif(targetRightSpeed<1350):
            targetRightSpeed = 1350

        frontSpeeds[pos]=targetFrontSpeed
        backSpeeds[pos]=targetBackSpeed
        leftSpeeds[pos]=targetLeftSpeed
        rightSpeeds[pos]=targetRightSpeed
        pos = (pos+1)%10
        targetFrontSpeed = targetBackSpeed = targetLeftSpeed = targetRightSpeed = 0
        for i in range(10):
            targetFrontSpeed += frontSpeeds[i]
            targetBackSpeed +=backSpeeds[i]
            targetLeftSpeed +=leftSpeeds[i]
            targetRightSpeed +=rightSpeeds[i]

        #frontPitchPub.publish(int(targetFrontSpeed/10))
        #backPitchPub.publish(int(targetBackSpeed/10))
        frontPitchPub.publish(1500)
        backPitchPub.publish(1500)
        sideLeftSpeedPub.publish(int(targetLeftSpeed/10))
        sideRightSpeedPub.publish(int(targetRightSpeed/10))
        rate.sleep()


if __name__=='__main__':
    main()
