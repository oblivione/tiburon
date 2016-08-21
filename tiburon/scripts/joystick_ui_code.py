#!/usr/bin/env python
from joystick_ui import Ui_MainWindow
import sys
import rospy
from PyQt4 import QtGui,QtCore
from std_msgs.msg import UInt16
import signal
import os
from tiburon.msg import joystick_data

class JoystickPanel(QtGui.QMainWindow): 
    
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        #self.node_init()
        self.subs()
    
    #def node_init(self):
        #os.system('rosrun joy joy_node &') #SET THE CORRECT jsX FIRST-->refer thruster_control_joy.cpp
        #os.system('rosrun tiburon thruster_joy_exe &') 
    '''
    NOTE:
    Due to creation of 6 subscribers, program was unable to handle so many threads causing 
    unpredictable segmentation fault at any time during execution.
    Therefore the following commented code was discarded
    '''
    

    '''
    def thrusterstateCallback(self, msg):
        print "tcb"
        if(msg.data == 1):
            self.ui.updates_lineEdit.setText("THRUSTER INITIALIZED!")
        elif(msg.data == 2):
            self.ui.updates_lineEdit.setText("THRUSTER ON!")
        elif(msg.data == 3):
            self.ui.updates_lineEdit.setText("THRUSTER OFF!")

    def runmodeCallback(self, msg):
        print "rcb"
        if(msg.data == 1):
            self.ui.runmode_lineEdit.setText("ON")
        else:
            self.ui.runmode_lineEdit.setText("OFF")
    
    def siderightspeedCallback(self, msg):
        print "srcb"
        self.ui.siderightspeed_lineEdit.setText(str(msg.data))

    def sideleftspeedCallback(self, msg):
        print "slcb"
        self.ui.sideleftspeed_lineEdit.setText(str(msg.data))

    def backpitchspeedCallback(self, msg):
        print "bpcb"
        self.ui.backpitchspeed_lineEdit.setText(str(msg.data))

    def frontpitchspeedCallback(self, msg):
        print "fpcb"
        self.ui.frontpitchspeed_lineEdit.setText(str(msg.data))
    '''
    
    def callback(self,msg):
        if(msg.runmode == 1):
            self.ui.runmode_lineEdit.setText("ON")
        else:
            self.ui.runmode_lineEdit.setText("OFF")

        if(msg.thrusterButton == 1):
            self.ui.updates_lineEdit.setText("THRUSTER INITIALIZED!")
        elif(msg.thrusterButton == 2):
            self.ui.updates_lineEdit.setText("THRUSTER ON!")
        elif(msg.thrusterButton == 3):
            self.ui.updates_lineEdit.setText("THRUSTER OFF!")
        
        self.ui.siderightspeed_lineEdit.setText(str(msg.siderightspeed))
        self.ui.sideleftspeed_lineEdit.setText(str(msg.sideleftspeed))
        self.ui.backpitchspeed_lineEdit.setText(str(msg.backpitchspeed))  
        self.ui.frontpitchspeed_lineEdit.setText(str(msg.frontpitchspeed))

    def subs(self):
        self.joy_sub = rospy.Subscriber("/joydata", joystick_data, self.callback)
        #self.frontpitchspeed_sub = rospy.Subscriber("/frontpitchspeed", UInt16, self.frontpitchspeedCallback)
        #self.backpitchspeed_sub = rospy.Subscriber("/backpitchspeed", UInt16, self.backpitchspeedCallback)
        #self.sideleftspeed_sub = rospy.Subscriber("/sideleftspeed", UInt16, self.sideleftspeedCallback)
        #self.siderightspeed_sub = rospy.Subscriber("/siderightspeed", UInt16, self.siderightspeedCallback)
        #self.runmode_sub = rospy.Subscriber("/runmode", UInt16, self.runmodeCallback)
        #self.button_sub = rospy.Subscriber("/thrusterstate", UInt16, self.thrusterstateCallback)

def main():
    rospy.init_node("joystick_ui_code")
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    joystick = JoystickPanel()
    joystick.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
