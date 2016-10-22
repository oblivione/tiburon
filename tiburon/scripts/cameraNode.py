import sys
import cv2
import numpy as np
import datetime
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PyQt4 import QtGui, QtCore, Qt


bridge = CvBridge()
frame = np.array([])
fourcc = cv2.cv.CV_FOURCC(*'MJPG')


def captureNextFrame(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    frame=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    for i in range(10):
        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.waitKey(1)

def main():
    rospy.init_node('cameraNode', anonymous=True)
    rospy.Subscriber('auv_cam1', Image, captureNextFrame)
    rospy.spin()


if __name__=='__main__':
    main()
