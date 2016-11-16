#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import UInt16

def frontCallback(data):
    os.system("mono ~/pololu/UscCmd --servo 0,"+str(data.data*4))

def backCallback(data):
    os.system("mono ~/pololu/UscCmd --servo 1,"+str((3000-data.data)*4))

def leftCallback(data):
    os.system("mono ~/pololu/UscCmd --servo 2,"+str((3000-data.data)*4))

def rightCallback(data):
    os.system("mono ~/pololu/UscCmd --servo 3,"+str(data.data*4))

def main():
    rospy.init_node('thruster_controller')
    forwardSub = rospy.Subscriber('frontpitchspeed',UInt16,frontCallback,queue_size=1)
    backSub = rospy.Subscriber('backpitchspeed',UInt16,backCallback,queue_size=1)
    leftSub = rospy.Subscriber('sideleftspeed',UInt16,leftCallback,queue_size=1)
    rightSub = rospy.Subscriber('siderightspeed',UInt16,rightCallback,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    main()
