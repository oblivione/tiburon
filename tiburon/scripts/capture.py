#!/usr/bin/env python
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import datetime

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
    pub1 = rospy.Publisher('auv_cam1',Image,queue_size=1)
    pub2 = rospy.Publisher('auv_cam2',Image,queue_size=1)
    pub3 = rospy.Publisher('auv_cam3',Image,queue_size=1)
    fourcc1 = fourcc2 = fourcc3 = cv2.cv.CV_FOURCC(*'MJPG')
    out1 = cv2.VideoWriter('1_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc1,30.0,(640,480))
    out2 = cv2.VideoWriter('2_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc2,30.0,(640,480))
    out3 = cv2.VideoWriter('3_AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',fourcc3,30.0,(640,480))
    rospy.init_node('image_capture',anonymous=True)
    rate = rospy.Rate(60)
    bridge = CvBridge()
    img1 = None
    img2 = None
    img3 = None
    while not rospy.is_shutdown():
        try:
            ret1=ret2=ret3=False
            if(count>=1):
                ret1, readFrame1 = capture1.read()
            if(count>=2):
                ret2, readFrame2 = capture2.read()
            if(count>=3):
                ret3, readFrame3 = capture3.read()
            if(ret1==True):
		out1.write(readFrame1)
                img1 = bridge.cv2_to_imgmsg(readFrame1, "bgr8")
                pub1.publish(img1)
            if(ret2==True):
		out2.write(readFrame2)
                img2 = bridge.cv2_to_imgmsg(readFrame2, "bgr8")
                pub2.publish(img2)
            if(ret3==True):
                out3.write(readFrame3)
                img3 = bridge.cv2_to_imgmsg(readFrame3, "bgr8")
                pub3.publish(img3)
            rate.sleep()
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()
