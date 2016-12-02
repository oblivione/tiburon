#!/usr/bin/env python

from serial import Serial
from time import sleep
from std_msgs.msg import UInt16, Float64

import rospy
ser = None

def thrusterStateCallback(msg):
    ser.write("T"+str(msg.data)+'#\n')
    print "T"+str(msg.data)+'#'

def depthSensorStateCallback(msg):
    ser.write("P"+str(msg.data)+'#\n')
    print "P"+str(msg.data)+'#'

def connectionStateCallback(msg):
    ser.write("C"+str(msg.data)+'#\n')
    print "C"+str(msg.data)+'#'

def main():
    global ser
    rospy.init_node('serial_node')
    ser = Serial('/dev/auv_nano',9600,timeout=1)
    sleep(3)
    thrusterSub = rospy.Subscriber('thrusterstate', UInt16, thrusterStateCallback,queue_size=1)
    depthSensorSub = rospy.Subscriber('depthSensorState', UInt16, depthSensorStateCallback,queue_size=1)
    checkConnectionSub = rospy.Subscriber('tiburonConnection', UInt16, connectionStateCallback,queue_size=1)
    depthSensorDataPub = rospy.Publisher('depth_value', Float64, queue_size=1)
    statusConnection = rospy.Publisher('auvStatus', UInt16, queue_size=1)
    underwaterStatus = rospy.Publisher('underwaterStatus', UInt16, queue_size=1)
    while(True):
        inp = ser.readline()
        if inp!="":
            print inp
            for i in range(len(inp)):
                if(inp[i]=='D'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    depthSensorDataPub.publish(float(temp))
                if(inp[i]=='S'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    statusConnection.publish(int(temp) % 2**16)
                if(inp[i]=='U'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    underwaterStatus.publish(int(temp) % 2**16)

if __name__ == '__main__':
    main()
