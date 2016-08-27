#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16,Float32,Float64
from tiburon.msg import ins_data
from dynamic_reconfigure.server import Server
from tiburon.cfg import yawPitchDepthConfig
import datetime,time
import math
import numpy as np
from PID import PID
from AngularPID import AngularPID
from vehicle import VehicleParams

depthController = PID()
pitchController = AngularPID()
yawController  = AngularPID()

depthReceived = 0
pitchAndYawReceived = 0

def depthCallback(msg):
    global depthController, depthReceived
    if msg.data<=1300 and msg.data>980
        depthController.currentVal = msg.data
    if depthReceived == 0:
        # Setting the checkpoint for depth as first Received value
        depthController.checkpoint = depthController.currentVal
    depthReceived = 1

def insCallback(msg):
    global pitchController, yawController, pitchAndYawReceived
    pitchController.currentVal = msg.YPR.y
    yawController.currentVal = msg.YPR.x
    if pitchAndYawReceived == 0:
        # Setting the initial checkpoint for pitch as 0 and yaw as first Received value
        pitchController.checkpoint = 0
        yawController.checkpoint = yawController.currentVal
    pitchAndYawReceived = 1

def callback(config,level):
    global depthController, pitchController, yawController
    if depthController.Kp != config.kp_depth or depthController.Ki != config.ki_depth or depthController.Kd !=config.kd_depth:
        depthController.Kp = config.kp_depth
        depthController.Ki = config.ki_depth
        depthController.Kd = config.kd_depth
        depthController.IError = 0.00
    if pitchController.Kp != config.kp_pitch or pitchController.Ki != config.ki_pitch or pitchController.Kd !=config.kd_pitch:
        pitchController.Kp = config.kp_pitch
        pitchController.Ki = config.ki_pitch
        pitchController.Kd = config.ki_pitch
        pitchController.IError = 0.00
    if yawController.Kp != config.kp_yaw or yawController.Ki != config.ki_yaw or yawController.Kd !=config.kd_yaw:
        yawController.Kp = config.kp_yaw
        yawController.Ki = config.ki_yaw
        yawController.Kd = config.kd_yaw
        yawController.IError = 0.00

    depthController.checkpoint = config.setpoint_depth
    pitchController.checkpoint = config.setpoint_pitch
    yawController.checkpoint = config.setpoint_yaw

depthDataSub=rospy.Subscriber("/depth_value",Float64,depthCallback)
insDataSub=rospy.Subscriber("/tiburon/ins_data",ins_data,insCallback)
frontPitchPub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
backPitchPub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
sideLeftSpeedPub=rospy.Publisher("/sideleftspeed",UInt16,queue_size=1)
sideRightSpeedPub=rospy.Publisher("/siderightspeed",UInt16,queue_size=1)

def main():
    global frontPitchPub, backPitchPub, sideLeftSpeedPub, sideRightSpeedPub
    global depthDataSub, insDataSub
    global depthController, pitchController, yawController

    vp = VehicleParams()
    rospy.init_node("tiburonController",anonymous=True)
    srv=Server(yawPitchDepthConfig,callback)
    thruster1,thruster2,thruster3,thruster4 = 0.00,0.00,0.00,0.00
    rate = rospy.Rate(200) # 200 Hz
    while(True):
        # If either value is still not Received, do nothing
        if depthReceived==0 or pitchAndYawReceived==0:
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
        - F1 sin θ - F2 sin θ + F3 cos θ + F4 cos θ = 0           = U2
        - F1 x1    + F2 x2                          = Ixx αxx     = U3
                                F3 x3    - F4 x4    = Izz αzz     = U4

        Matrix form
         ==                                  ==   ==    ==         ==            ==
        |   cos θ    cos θ    sin θ    sin θ   | |   F1   |       |   U1 + B - mg  |
        |  -sin θ   -sin θ    cos θ    cos θ   | |   F2   |  ===  |       00       |
        |   -x1       x2       00       00     | |   F3   |  ===  |       U3       |
        |    00       00       x3      -x4     | |   F4   |       |       U4       |
         ==                                  ==   ==    ==         ==            ==

        '''

        _sin = math.sin(pitchController.currentVal/180 * math.pi)
        _cos = math.cos(pitchController.currentVal/180 * math.pi)
        A = np.array([[_cos, _cos, _sin, _sin], [-_sin, -_sin, _cos, _cos], [-vp.x1, vp.x2, 0, 0], [0, 0, vp.x3, vp.x4]])
        b = np.array([depthU+ vp.B - vp.mass*vp.gravity, 0, pitchU, yawU])
        F1,F2,F3,F4 = np.linalg.solve(A,b)

        if F1 > 0:
            thruster1 = thruster1 + 0.1
        else if F1 < 0:
            thruster1 = thruster1 - 0.1

        if F2 > 0:
            thruster2 = thruster2 + 0.1
        else if F2 < 0:
            thruster2 = thruster2 - 0.1

        if F3 > 0:
            thruster3 = thruster3 + 0.1
        else if F3 < 0:
            thruster3 = thruster3 - 0.1

        if F4 > 0:
            thruster4 = thruster4 + 0.1
        else if F4 < 0:
            thruster4 = thruster4 - 0.1

        if thruster1 > 500:
            thruster1 = 500
        if thruster1 < -500:
            thruster1 = -500

        if thruster2 > 500:
            thruster2 = 500
        if thruster2 < -500:
            thruster2 = -500

        if thruster3 > 300:
            thruster3 = 300
        if thruster3 < -300:
            thruster3 = -300

        if thruster4 > 300:
            thruster4 = 300
        if thruster4 < -300:
            thruster4 = -300

        frontPitchPub.publish(1500 + thruster1)
        backPitchPub.publish(1500 + thruster2)
        sideLeftSpeedPub.publish(1500 + thruster3)
        sideRightSpeedPub.publish(1500 + (thruster4/0.8))
        rate.sleep()


if __name__=='__main__':
    main()
