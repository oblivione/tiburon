#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import UInt16

connected = False
globalStatus = 0
globalControllerStatus = 0

def controllerStatusCallback(msg):
    global globalControllerStatus
    globalControllerStatus = msg.data

def statusCallback(msg):
    global globalStatus
    globalStatus = msg.data


def main():
    rospy.init_node('auv_init')
    thrusterPub = rospy.Publisher('thrusterstate', UInt16, queue_size=1)
    depthSensorPub = rospy.Publisher('depthSensorState', UInt16, queue_size=1)
    status = rospy.Subscriber('auvStatus', UInt16, statusCallback, queue_size=1)
    controllerStatus = rospy.Subscriber('controllerStatus', UInt16, controllerStatusCallback, queue_size = 1)
    frontPitchPub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
    backPitchPub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
    sideLeftSpeedPub=rospy.Publisher("/sideleftspeed",UInt16,queue_size=1)
    sideRightSpeedPub=rospy.Publisher("/siderightspeed",UInt16,queue_size=1)
    while(globalStatus!=1):
        print 'Waiting for rosserial'
        time.sleep(1)
    while(globalStatus!=2):
        print 'Waiting to switch on relay'
        thrusterPub.publish(2)
        time.sleep(1)
    while(globalStatus!=3):
        print 'Waiting to switch on depthSensor'
        depthSensorPub.publish(1)
        time.sleep(1)
    while(globalControllerStatus!=4):
        print 'Waiting for thruster initialization'
        frontPitchPub.publish(1500)
        backPitchPub.publish(1500)
        sideLeftSpeedPub.publish(1500)
        sideRightSpeedPub.publish(1500)
        time.sleep(1)
    print "Done. Exiting!"

if __name__=='__main__':
    main()
