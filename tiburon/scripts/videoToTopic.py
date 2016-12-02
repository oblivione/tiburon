#!/usr/bin/env python
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    pub1 = rospy.Publisher(sys.argv[2],Image,queue_size=1)
    rospy.init_node('videoToTopic',anonymous=True)
    rate = rospy.Rate(60)
    bridge = CvBridge()
    fName = sys.argv[1]
    cap = cv2.VideoCapture(fName)
    while(cap.isOpened()):
        ret, frame = cap.read()
        img1 = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub1.publish(img1)
        rate.sleep()

    cap.release()


if __name__=='__main__':
    main()
