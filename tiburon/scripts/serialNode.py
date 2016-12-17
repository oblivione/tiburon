#!/usr/bin/env python

from serial import Serial
from time import sleep
from std_msgs.msg import UInt16, Float64
from geometry_msgs.msg import Vector3

import rospy
ser = None
batterymsg = Vector3()

def resetCallback(msg):
    ser.write('R\n')
    print "R"

def thrusterStateCallback(msg):
    ser.write("T"+str(msg.data)+'#\n')
    print "T"+str(msg.data)+'#'

def depthSensorStateCallback(msg):
    ser.write("P"+str(msg.data)+'#\n')
    print "P"+str(msg.data)+'#'

def connectionStateCallback(msg):
    ser.write("C"+str(msg.data)+'#\n')
    print "C"+str(msg.data)+'#'

def ledCallback(msg):
    ser.write("L"+str(msg.data)+'#\n')


def main():
    global ser
    rospy.init_node('serial_node')
    ser = Serial('/dev/auv_nano',9600,timeout=1)
    sleep(3)
    batterymsg.x = batterymsg.y = batterymsg.z = 0
    thrusterSub = rospy.Subscriber('thrusterstate', UInt16, thrusterStateCallback,queue_size=1)
    ledSub = rospy.Subscriber('ledState', UInt16, ledCallback,queue_size=1)
    depthSensorSub = rospy.Subscriber('depthSensorState', UInt16, depthSensorStateCallback,queue_size=1)
    checkConnectionSub = rospy.Subscriber('tiburonConnection', UInt16, connectionStateCallback,queue_size=1)
    resetCmdSub = rospy.Subscriber('resetCommand',UInt16,resetCallback,queue_size=1)
    resetPub = rospy.Publisher('arduinoReset', UInt16, queue_size=1)
    depthSensorDataPub = rospy.Publisher('depth_value', Float64, queue_size=1)
    statusConnection = rospy.Publisher('auvStatus', UInt16, queue_size=1)
    underwaterStatus = rospy.Publisher('underwaterStatus', UInt16, queue_size=1)
    batteryLevelPub = rospy.Publisher('batteryLevel',Vector3,queue_size=1)
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
                if(inp[i]=='X'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    batterymsg.x = float(temp)*12.6/789.0
                    batteryLevelPub.publish(batterymsg)
                if(inp[i]=='Y'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    batterymsg.y = float(temp)*16.8/612.0
                    batteryLevelPub.publish(batterymsg)
                if(inp[i]=='Z'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    batterymsg.z = float(temp)*24.8/446.0
                if(inp[i]=='R'):
                    i+=1
                    temp=""
                    while(True):
                        if(inp[i]!='#'):
                            temp+=inp[i]
                            i+=1
                        else:
                            break
                    resetPub.publish(int(temp))


if __name__ == '__main__':
    main()
