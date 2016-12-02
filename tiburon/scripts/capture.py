#!/usr/bin/env python
import cv2
import sys
import subprocess
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from tiburon.msg import ins_data
import datetime

depthVal = pitchVal = yawVal = 0.0

def depthCallback(msg):
    global depthVal
    depthVal = msg.data

def yawAndPitchCallback(msg):
    global pitchVal,yawVal
    pitchVal = msg.YPR.y
    yawVal = msg.YPR.x

'''
def main():
    if len(sys.argv)<2:
        print "Usage: python capture.py <no of cameras> <camera nos>"
        return
    count = int(sys.argv[1])
    if len(sys.argv)<count+2:
        print "Enter all camera nos"
        return
    capture1=capture2=capture3=None
    if count>=1:
        capture1 = cv2.VideoCapture(int(sys.argv[2]))
        if count>=2:
            capture2 = cv2.VideoCapture(int(sys.argv[3]))
            if count>=3:
                capture3 = cv2.VideoCapture(int(sys.argv[4]))
    pub1 = rospy.Publisher('auv_cam2',Image,queue_size=1)
    pub2 = rospy.Publisher('auv_cam1',Image,queue_size=1)
    pub3 = rospy.Publisher('auv_cam3',Image,queue_size=1)
    depthDataSub = rospy.Subscriber('/depth_value',Float64,depthCallback)
    yawAndPitchSub = rospy.Subscriber('/tiburon/ins_data',ins_data,yawAndPitchCallback)
    fourcc1 = fourcc2 = fourcc3 = cv2.cv.CV_FOURCC(*'MJPG')
    out1 = cv2.VideoWriter('1_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc1,30.0,(640,480))
    out2 = cv2.VideoWriter('2_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc2,30.0,(640,480))
    out3 = cv2.VideoWriter('3_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc3,30.0,(640,480))
    rospy.init_node('image_capture',anonymous=False)
    rate = rospy.Rate(60)
    bridge = CvBridge()
    img1 = None
    img2 = None
    img3 = None
    font = cv2.FONT_HERSHEY_SIMPLEX
    while not rospy.is_shutdown():

        try:
            ret1=ret2=ret3=False
            if(count>=1):
                ret1, readFrame1 = capture1.read()
                readFrame1 = cv2.flip(cv2.flip(readFrame1,0),1)
            if(count>=2):
                ret2, readFrame2 = capture2.read()
            if(count>=3):
                ret3, readFrame3 = capture3.read()
            if(ret1==True):
                cv2.putText(readFrame1,'D: '+str(depthVal),(10,50), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame1,'P: '+str(pitchVal),(10,70), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame1,'Y: '+str(yawVal),(10,90), font, 0.7,(0,255,255),2)
                out1.write(readFrame1)
                img1 = bridge.cv2_to_imgmsg(readFrame1, "bgr8")
                pub1.publish(img1)
            if(ret2==True):
                cv2.putText(readFrame2,'D: '+str(depthVal),(10,50), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame2,'P: '+str(pitchVal),(10,70), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame2,'Y: '+str(yawVal),(10,90), font, 0.7,(0,255,255),2)
                out2.write(readFrame2)
                img2 = bridge.cv2_to_imgmsg(readFrame2, "bgr8")
                pub2.publish(img2)
            if(ret3==True):
                cv2.putText(readFrame3,'D: '+str(depthVal),(10,50), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame3,'P: '+str(pitchVal),(10,70), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame3,'Y: '+str(yawVal),(10,90), font, 0.7,(0,255,255),2)
                out3.write(readFrame3)
                img3 = bridge.cv2_to_imgmsg(readFrame3, "bgr8")
                pub3.publish(img3)
            rate.sleep()
        except KeyboardInterrupt:
            break
'''

def numFromName(linkName):
    cmd = "readlink -f /dev/"+linkName
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
    out = process.communicate()[0]
    nums = [int(x) for x in out if x.isdigit()]
    return nums[0]

def main():
    cap1 = cv2.VideoCapture(numFromName('bottomCamera'))
    cap2 = cv2.VideoCapture(numFromName('leftCamera'))
    pub1 = rospy.Publisher('auvBottomCamera',Image,queue_size=1)
    pub2 = rospy.Publisher('auvLeftCamera',Image,queue_size=1)
    depthDataSub = rospy.Subscriber('/depth_value',Float64,depthCallback)
    yawAndPitchSub = rospy.Subscriber('/tiburon/ins_data',ins_data,yawAndPitchCallback)
    fourcc1 = fourcc2 = fourcc3 = cv2.cv.CV_FOURCC(*'MJPG')
    out1 = cv2.VideoWriter('1_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc1,30.0,(640,480))
    out2 = cv2.VideoWriter('2_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc2,30.0,(640,480))
    rospy.init_node('image_capture',anonymous=False)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    img1 = None
    img2 = None
    font = cv2.FONT_HERSHEY_SIMPLEX
    while not rospy.is_shutdown():
        try:
            ret1=ret2=False
            ret1, readFrame1 = cap1.read()
            readFrame1 = cv2.flip(cv2.flip(readFrame1,0),1)
            ret2, readFrame2 = cap2.read()
            if(ret1==True):
                img1 = bridge.cv2_to_imgmsg(readFrame1, "bgr8")
                pub1.publish(img1)
                cv2.putText(readFrame1,'D: '+str(depthVal),(10,50), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame1,'P: '+str(pitchVal),(10,70), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame1,'Y: '+str(yawVal),(10,90), font, 0.7,(0,255,255),2)
                out1.write(readFrame1)
            if(ret2==True):
                readFrame2 = cv2.transpose(readFrame2)
                readFrame2 = cv2.flip(readFrame2,1)
                img2 = bridge.cv2_to_imgmsg(readFrame2, "bgr8")
                pub2.publish(img2)
                cv2.putText(readFrame2,'D: '+str(depthVal),(10,50), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame2,'P: '+str(pitchVal),(10,70), font, 0.7,(0,255,255),2)
                cv2.putText(readFrame2,'Y: '+str(yawVal),(10,90), font, 0.7,(0,255,255),2)
                out2.write(readFrame2)
            rate.sleep()
        except KeyboardInterrupt:
            break
        print "Done"

if __name__ == '__main__':
    main()
