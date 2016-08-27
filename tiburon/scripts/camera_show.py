#!/usr/bin/env python
# This file generates a UI that shows the live stream from AUV-NITR.
# It can also be used to capture screenshots and videos from the
# attached cameras.

import sys
import cv2
import numpy as np
import datetime
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PyQt4 import QtGui, QtCore, Qt

# ui.py is an auto-generated python file. Use QTCreator to make a
# ui and convert it to a py file using the following command:-
# pyuic4 mainWindow.ui -o ui.py
from camera_ui import Ui_MainWindow

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Video():
    def __init__(self,camno):
        #init subsciber here
        self.bridge = CvBridge()
        self.currentFrame=np.array([])
        self.fourcc = cv2.cv.CV_FOURCC(*'MJPG')
        rospy.init_node('image_show', anonymous=True)
        rospy.Subscriber('auv_cam'+str(camno), Image, self.captureNextFrame)

    def captureNextFrame(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.currentFrame=cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB)


    def convertFrame(self):
        try:
            self.currentFrame=cv2.resize(self.currentFrame,(320,240))
	        # First two parameters of shape[] are height and width
            height,width=self.currentFrame.shape[:2]
	        # Converting last read frame to QImage
            img=QtGui.QImage(self.currentFrame,width,height,QtGui.QImage.Format_RGB888)
	        # Convert QImage to QPixmap to display in UI
            img=QtGui.QPixmap.fromImage(img)
            return img
        except:
            return None

    def addFrameToVideo(self):
        self.out.write(self.cv_image)


class Gui(QtGui.QMainWindow):
    def __init__(self,parent=None):
	    # Initialize the UI
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
	    # Declare VideoCapture object
        self.video1 = Video(1)
        self.video2 = Video(2)
        self.video3 = Video(3)
        self.isRecording1 = False
        self.isRecording2 = False
        self.isRecording3 = False
        self.ui.recButton1.clicked.connect(self.onClicked_recButton1)
        self.ui.capButton1.clicked.connect(self.onClicked_capButton1)
        self.ui.recButton2.clicked.connect(self.onClicked_recButton2)
        self.ui.capButton2.clicked.connect(self.onClicked_capButton2)
        self.ui.recButton3.clicked.connect(self.onClicked_recButton3)
        self.ui.capButton3.clicked.connect(self.onClicked_capButton3)
	    # Create a timer and connect it to function play()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play)
	    # QTimer.start(int msec) starts or restarts the timer
	    # at an interval of msec milliseconds
        self._timer.start(10)
        self.update()

    def play(self):
        try:
	        # videoFrame is a QLabel. See ui.py for better understanding
            if(self.ui.cam1.isChecked()):
                self.ui.videoFrame1.setPixmap(self.video1.convertFrame())
                self.ui.videoFrame1.setScaledContents(True)
                if self.isRecording1 == True:
                    self.video1.addFrameToVideo()
            if(self.ui.cam2.isChecked()):
                self.ui.videoFrame2.setPixmap(self.video2.convertFrame())
                self.ui.videoFrame2.setScaledContents(True)
                if self.isRecording2 == True:
                    self.video2.addFrameToVideo()
            if(self.ui.cam3.isChecked()):
                self.ui.videoFrame3.setPixmap(self.video3.convertFrame())
                self.ui.videoFrame3.setScaledContents(True)
                if self.isRecording3 == True:
                    self.video3.addFrameToVideo()
        except TypeError:
            #print "No frame"
            return

    def onClicked_recButton1(self):
        # Check if recording is on or not and update accordingly
        if self.isRecording1 == True:
            self.ui.recButton1.setText(_translate("MainWindow", "Record", None))
            self.video1.out.release()
            self.isRecording1 = False
        else:
            self.ui.recButton1.setText(_translate("MainWindow", "Stop", None))
            self.video1.out = cv2.VideoWriter('AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',self.video1.fourcc, 20.0, (640,480))
            self.isRecording1 = True

    def onClicked_recButton2(self):
        # Check if recording is on or not and update accordingly
        if self.isRecording2 == True:
            self.ui.recButton2.setText(_translate("MainWindow", "Record", None))
            self.video2.out.release()
            self.isRecording2 = False
        else:
            self.ui.recButton2.setText(_translate("MainWindow", "Stop", None))
            self.video2.out = cv2.VideoWriter('AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',self.video2.fourcc, 20.0, (640,480))
            self.isRecording2 = True

    def onClicked_recButton3(self):
        # Check if recording is on or not and update accordingly
        if self.isRecording3 == True:
            self.ui.recButton3.setText(_translate("MainWindow", "Record", None))
            self.video3.out.release()
            self.isRecording3 = False
        else:
            self.ui.recButton3.setText(_translate("MainWindow", "Stop", None))
            self.video3.out = cv2.VideoWriter('AUV_REC_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.avi',self.video3.fourcc, 20.0, (640,480))
            self.isRecording3 = True


    def onClicked_capButton1(self):
        cv2.imwrite('AUV_CAP_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.jpg',self.video1.cv_image)

    def onClicked_capButton2(self):
        cv2.imwrite('AUV_CAP_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.jpg',self.video2.cv_image)

    def onClicked_capButton3(self):
        cv2.imwrite('AUV_CAP_'+datetime.datetime.now().strftime('%Y%m%d%H%M%S')+'.jpg',self.video3.cv_image)

def main():
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
