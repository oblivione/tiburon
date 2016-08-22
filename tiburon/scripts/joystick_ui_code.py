#!/usr/bin/env python
from joystick_ui import Ui_MainWindow
import sys
import rospy
from PyQt4 import QtGui,QtCore
from std_msgs.msg import UInt16
import signal
import os
from vn_100.msg import joystick_data

class JoystickPanel(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.node_init()
        self.subs()
        self.runmode = 0
        self.thrusterstate = 0
        self.leftspeed = 1500
        self.rightspeed = 1500
        self.forwardspeed = 1000
        self.backwardspeed = 1000
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(10)

    def node_init(self):
        os.system('rosrun joy joy_node &') #SET THE CORRECT jsX FIRST-->refer thruster_control_joy.cpp
        os.system('rosrun tiburon thruster_joy_exe &')

    def update(self):
        if(self.thrusterstate == 1):
            self.ui.updates_lineEdit.setText("THRUSTER INITIALIZED!")
        elif(self.thrusterstate == 2):
            self.ui.updates_lineEdit.setText("THRUSTER ON!")
        elif(self.thrusterstate == 3):
            self.ui.updates_lineEdit.setText("THRUSTER OFF!")


        if(self.runmode == 1):
            self.ui.runmode_lineEdit.setText("ON")
        elif(self.runmode == 0):
            self.ui.runmode_lineEdit.setText("OFF")

        self.ui.siderightspeed_lineEdit.setText(str(self.rightspeed))
        self.ui.sideleftspeed_lineEdit.setText(str(self.leftspeed))
        self.ui.backpitchspeed_lineEdit.setText(str(self.backwardspeed))
        self.ui.frontpitchspeed_lineEdit.setText(str(self.forwardspeed))

    def thrusterstateCallback(self, msg):
        self.thrusterstate = msg.data

    def runmodeCallback(self, msg):
        self.runmode = msg.data

    def siderightspeedCallback(self, msg):
        self.rightspeed = msg.data

    def sideleftspeedCallback(self, msg):
        self.leftspeed = msg.data

    def backpitchspeedCallback(self, msg):
        self.backwardspeed = msg.data

    def frontpitchspeedCallback(self, msg):
        self.forwardspeed = msg.data

    def subs(self):
        self.frontpitchspeed_sub = rospy.Subscriber("/frontpitchspeed", UInt16, self.frontpitchspeedCallback)
        self.backpitchspeed_sub = rospy.Subscriber("/backpitchspeed", UInt16, self.backpitchspeedCallback)
        self.sideleftspeed_sub = rospy.Subscriber("/sideleftspeed", UInt16, self.sideleftspeedCallback)
        self.siderightspeed_sub = rospy.Subscriber("/siderightspeed", UInt16, self.siderightspeedCallback)
        self.runmode_sub = rospy.Subscriber("/runmode", UInt16, self.runmodeCallback)
        self.button_sub = rospy.Subscriber("/thrusterstate", UInt16, self.thrusterstateCallback)

def main():
    rospy.init_node("joystick_ui_code")
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    joystick = JoystickPanel()
    joystick.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
