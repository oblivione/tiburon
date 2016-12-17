#!/usr/bin/env python

import rospy
import time
import os
from std_msgs.msg import UInt16

connected = False
globalStatus = 0
globalControllerStatus = 0
uwstatusVal = 0
resetted = 0

def resetCallback(msg):
    global resetted
    resetted = msg.data

def controllerStatusCallback(msg):
    global globalControllerStatus
    globalControllerStatus = msg.data

def statusCallback(msg):
    global globalStatus
    globalStatus = msg.data

def uwstatusCallback(msg):
    global uwstatusVal
    if msg.data == 10:
        uwstatusVal+=1
    else:
        uwstatusVal=0

def main():
    rospy.init_node('auv_init')
    thrusterPub = rospy.Publisher('thrusterstate', UInt16, queue_size=1)
    depthSensorPub = rospy.Publisher('depthSensorState', UInt16, queue_size=1)
    status = rospy.Subscriber('auvStatus', UInt16, statusCallback, queue_size=1)
    uwstatus = rospy.Subscriber('underwaterStatus', UInt16, uwstatusCallback, queue_size=1)
    resetSub = rospy.Subscriber('arduinoReset', UInt16, resetCallback, queue_size=1)
    controllerStatus = rospy.Subscriber('controllerStatus', UInt16, controllerStatusCallback, queue_size = 1)
    frontPitchPub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
    backPitchPub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
    sideLeftSpeedPub=rospy.Publisher("/sideleftspeed",UInt16,queue_size=1)
    sideRightSpeedPub=rospy.Publisher("/siderightspeed",UInt16,queue_size=1)
    resetCmdPub = rospy.Publisher('resetCommand',UInt16,queue_size=1)
    while(resetted!=1):
        print 'Waiting for reset'
        resetCmdPub.publish(1)
        time.sleep(1)
    while(globalStatus!=1):
        print 'Waiting for rosserial'
        time.sleep(1)
    while(globalStatus!=2):
        print 'Waiting to switch on relay'
        thrusterPub.publish(2)
        time.sleep(1)
    while(globalControllerStatus!=4):
        print 'Waiting for thruster initialization'
        frontPitchPub.publish(1500)
        backPitchPub.publish(1500)
        sideLeftSpeedPub.publish(1500)
        sideRightSpeedPub.publish(1500)
        time.sleep(1)
    print 'Check thrusters here'
    time.sleep(1)
    frontPitchPub.publish(1540)
    time.sleep(1)
    frontPitchPub.publish(1500)
    time.sleep(1)
    backPitchPub.publish(1540)
    time.sleep(1)
    backPitchPub.publish(1500)
    time.sleep(1)
    sideLeftSpeedPub.publish(1540)
    time.sleep(1)
    sideLeftSpeedPub.publish(1500)
    time.sleep(1)
    sideRightSpeedPub.publish(1540)
    time.sleep(1)
    sideRightSpeedPub.publish(1500)
    time.sleep(1)
    for i in range(10):
        frontPitchPub.publish(1500)
        backPitchPub.publish(1500)
        sideLeftSpeedPub.publish(1500)
        sideRightSpeedPub.publish(1500)

    while(resetted!=1):
        print 'Waiting for reset'
        resetCmdPub.publish(1)
        time.sleep(1)
    while(globalStatus!=1):
        print 'Waiting for rosserial'
        time.sleep(1)
    while(uwstatusVal<=100):
        print 'Waiting for underwater'
        time.sleep(1)
    while(globalStatus!=2):
        print 'Waiting to switch on relay'
        thrusterPub.publish(2)
        time.sleep(1)
    while(globalStatus!=3):
        print 'Waiting to switch on depthSensor'
        depthSensorPub.publish(1)
        time.sleep(1)
    os.system('rosrun tiburon mainController.py &')
    time.sleep(10)
    os.system('roslaunch tiburon visionStack.launch')
    while(True):
        print 'Waiting'
        time.sleep(1)
    print "Done. Exiting!"

if __name__=='__main__':
    main()
