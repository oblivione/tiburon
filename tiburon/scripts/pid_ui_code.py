#!/usr/bin/env python
from pid_ui import Ui_MainWindow
import sys
from PyQt4 import QtGui,QtCore
import rospy
from tiburon.msg import ins_data
from tiburon.cfg import pidConfig,depthparamsConfig,pitchparamsConfig
from std_msgs.msg import String,UInt16,Float64,Float32
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.msg import Config
import signal
import os

'''
NOTE:
Run : server_yaw, server_pitch, server_depth
Then run:
rosrun rqt_reconfigure rqt_reconfigure --> used to change kp,ki,kd,setpoint values of yaw,pitch,depth
rosbag files or real data --> updates the value of ckpoint
'''

setpoint_depth=0.00
setpoint_pitch=0.00

class pidPanel(QtGui.QMainWindow):
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.depthX=[]
        self.depthY=[]
        self.setpoint_d=[]
        self.pitchX = []
        self.pitchY = []
        self.setpoint_p = []
        self.pubsNsubs()
        self.graph_depth()
        self.graph_pitch()

        self.kp_yaw=0.00
        self.ki_yaw=0.00
        self.kd_yaw=0.00
        self.ckpoint_yaw=0.00

        self.kp_pitch=0.00
        self.ki_pitch=0.00
        self.kd_pitch=0.00
        self.ckpoint_pitch=0.00

        self.kp_depth=0.00
        self.ki_depth=0.00
        self.kd_depth=0.00
        self.ckpoint_depth=0.00

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(10)

    def update(self):
        global setpoint_depth
        global setpoint_pitch

        self.ui.ckpoint_lineEdit.setText(str(self.ckpoint_yaw))
        self.ui.kp_yaw_lineEdit.setText(str(self.kp_yaw)) #kp
        self.ui.ki_yaw_lineEdit.setText(str(self.ki_yaw)) #ki
        self.ui.kd_yaw_lineEdit.setText(str(self.kd_yaw)) #kd

        self.ui.kp_depth_lineEdit.setText(str(self.kp_depth)) #kp
        self.ui.ki_depth_lineEdit.setText(str(self.ki_depth)) #ki
        self.ui.kd_depth_lineEdit.setText(str(self.kd_depth)) #kd
        self.ui.setpoint_depth_lineEdit.setText(str(setpoint_depth))

        self.ui.kp_pitch_lineEditlineEdi.setText(str(self.kp_pitch)) #kp
        self.ui.ki_pitch_lineEdit.setText(str(self.ki_pitch)) #ki
        self.ui.kd_pitch_lineEdit.setText(str(self.kd_pitch)) #kd
        self.ui.setpoint_pitch_lineEdit.setText(str(setpoint_pitch))

    def yawCallback(self,msg):
        self.ckpoint_yaw=msg.YPR.x

    #doubles is an array whose contents can be printed by msg.doubles
    #The required kp,ki,kd value of yaw is stored in the given positions

    def srv_y_callback(self,msg):
        #print msg.doubles
        self.kp_yaw = msg.doubles[3].value
        self.ki_yaw = msg.doubles[4].value
        self.kd_yaw = msg.doubles[7].value

    #def depthCallback(self,msg):
    #    self.ui.ckpoint_depth_lineEdit.setText(str(msg.data))

    def srv_d_callback(self,msg):
        global setpoint_depth
        self.kp_depth = msg.doubles[3].value
        self.ki_depth = msg.doubles[0].value
        self.kd_depth = msg.doubles[2].value
        setpoint_depth = msg.doubles[1].value

    def srv_p_callback(self,msg):
        #print msg.doubles
        global setpoint_pitch
        self.kp_pitch = msg.doubles[1].value
        self.ki_pitch = msg.doubles[3].value
        self.kd_pitch = msg.doubles[2].value
        setpoint_pitch = msg.doubles[4].value

    def pubsNsubs(self):
        self.yaw = rospy.Subscriber("/tiburon/ins_data",ins_data,self.yawCallback)
        self.yaw_srv = rospy.Subscriber("/server_yaw/parameter_updates",Config,self.srv_y_callback)
        #self.depth = rospy.Subscriber("/depth_value",Float64,self.depthCallback)
        self.depth_srv = rospy.Subscriber("/server_depth/parameter_updates",Config,self.srv_d_callback)
        #self.pitch = rospy.Subscriber("/tiburon/ins_data",ins_data,self.pitchCallback)
        self.pitch_srv = rospy.Subscriber("/server_pitch/parameter_updates",Config,self.srv_p_callback)

    def graph_depth(self):
        rospy.Subscriber('depth_value',Float64,self.depthCallback)
        self.startTimeNow = rospy.get_rostime()
        self.startTime = self.startTimeNow.secs + 10**-9*self.startTimeNow.nsecs
        #self.ui.graphicsView_3.setTitle('Depth Plot') #Causes seg fault
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_d)
        self._timer.start(100)

    def play_d(self):
        self.ui.graphicsView_3.plot(self.depthX, self.depthY)
        self.ui.graphicsView_3.plot(self.depthX, self.setpoint_d)

    def depthCallback(self,msg):
        global setpoint_depth
        self.ui.ckpoint_depth_lineEdit.setText(str(msg.data))
        self.depthTimeNow = rospy.get_rostime()
        self.depthTime = self.depthTimeNow.secs + 10**-9*self.depthTimeNow.nsecs
        self.depthY.append(msg.data)
        self.depthX.append(self.depthTime-self.startTime)
        self.setpoint_d.append(setpoint_depth)

    def graph_pitch(self):
        rospy.Subscriber("/tiburon/ins_data",ins_data,self.pitchCallback)
        #self.startTimeNow_2 = rospy.get_rostime()
        #self.startTime_2 = self.startTimeNow_2.secs + 10**-9*self.startTimeNow_2.nsecs
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_p)
        self._timer.start(100)
        #print self.pitchX, self.pitchY

    def play_p(self):
        self.ui.graphicsView_2.plot(self.pitchX, self.pitchY)
        self.ui.graphicsView_2.plot(self.pitchX, self.setpoint_p)

    def pitchCallback(self,msg):
        global setpoint_pitch
        self.ui.ckpoint_pitch_lineEdit.setText(str(msg.YPR.y))
        self.pitchTimeNow = rospy.get_rostime()
        self.pitchTime = self.pitchTimeNow.secs + 10**-9*self.pitchTimeNow.nsecs
        self.pitchY.append(msg.YPR.y)
        self.pitchX.append(self.pitchTime-self.startTime)
        self.setpoint_p.append(setpoint_pitch)

def main():
    rospy.init_node("pid_ui_code")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    pid = pidPanel()
    pid.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
