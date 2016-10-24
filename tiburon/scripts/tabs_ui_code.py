#!/usr/bin/env python
from tabsUI import Ui_MainWindow
import sys
import rospy
from PyQt4 import QtGui,QtCore
import signal
from tiburon.msg import pid
from std_msgs.msg import String,UInt16,Float64,Float32
from tiburon.msg import ins_data
from geometry_msgs.msg import Vector3 

pub=rospy.Publisher("pid_values",pid,queue_size=1)

#TODO : set slider value according to the value typed (According to radio button signal)

frontpitchspeed_val, backpitchspeed_val, sideleftspeed_val, siderightspeed_val = 0,0,0,0
setpoint_yaw, setpoint_pitch, setpoint_depth, setpoint_forward = 0,0,0,0

class Panel(QtGui.QMainWindow):
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
        self.yawX = []
        self.yawY = []
        self.setpoint_y = []
        self.forwardX = []
        self.forwardY = []
        self.setpoint_f = []

        #TODO: set slider according to line edit
        self.ui.kp_lineEdit.textEdited.connect(self.update_kp_slider)
        
        self.signals()
        self.setslidervalues()        
        self.msg = pid()

        #speed
        self.speed()

        #tab 2: graphs
        self.graph()

        #button
        self.ui.resetButton.clicked.connect(self.reset)

        #updates sliders,lineEdit on changing radio buttons
        #it also updates the speed, table
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(10)

    def reset(self):
        self.msg.kp_yaw = 0
        self.msg.kp_pitch = 0
        self.msg.kp_depth = 0
        self.msg.kp_forward = 0
        self.msg.ki_yaw = 0
        self.msg.ki_pitch = 0
        self.msg.ki_forward = 0
        self.msg.ki_depth = 0
        self.msg.kd_yaw = 0
        self.msg.kd_pitch = 0
        self.msg.kd_depth = 0
        self.msg.kd_forward = 0
  
    def setslidervalues(self):
        self.ui.kp_slider.setRange(0,20)
        self.ui.ki_slider.setRange(0,20)
        self.ui.kd_slider.setRange(0,20)
        self.ui.setpoint_slider.setRange(-90,1300)

        self.ui.kp_slider.setValue(0)
        self.ui.ki_slider.setValue(0)
        self.ui.kd_slider.setValue(0)
        self.ui.setpoint_slider.setValue(0)

        self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
        self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
        self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
        self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))

    def signals(self):
        self.connect(self.ui.kp_slider,QtCore.SIGNAL("valueChanged(int)"),self.kp)
        self.connect(self.ui.ki_slider,QtCore.SIGNAL("valueChanged(int)"),self.ki)
        self.connect(self.ui.kd_slider,QtCore.SIGNAL("valueChanged(int)"),self.kd)
        self.connect(self.ui.setpoint_slider,QtCore.SIGNAL("valueChanged(int)"),self.setpoint)
        #update slider according to value entered
        #self.connect(self.ui.kp_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_kp_slider)
        #self.connect(self.ui.ki_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_ki_slider)
        #self.connect(self.ui.kd_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_kd_slider)
        #self.connect(self.ui.setpoint_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_setpoint_slider)

    @QtCore.pyqtSlot(str)
    def update_kp_slider(self):
        print "entered"
        print self.ui.kp_lineEdit.text()
        self.ui.kp_slider.setValue(self.ui.kp_lineEdit.text())
    '''
    def update_ki_slider(self):
    def update_kd_slider(self):
    def update_setpoint_slider(self):
    '''

    def kp(self):
        if(self.ui.yaw_radioButton.isChecked()):
            self.msg.kp_yaw = self.ui.kp_slider.value()
            pub.publish(self.msg)
            self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
        elif(self.ui.pitch_radioButton.isChecked()):
            self.msg.kp_pitch = self.ui.kp_slider.value()
            pub.publish(self.msg)
            self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
        if(self.ui.depth_radioButton.isChecked()):
            self.msg.kp_depth = self.ui.kp_slider.value()
            pub.publish(self.msg)
            self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
        if(self.ui.forward_radioButton.isChecked()):
            self.msg.kp_forward = self.ui.kp_slider.value()
            pub.publish(self.msg)
            self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
    def ki(self):
        if(self.ui.yaw_radioButton.isChecked()):
            self.msg.ki_yaw = self.ui.ki_slider.value()
            pub.publish(self.msg)
            self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
        elif(self.ui.pitch_radioButton.isChecked()):
            self.msg.ki_pitch = self.ui.ki_slider.value()
            pub.publish(self.msg)
            self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
        if(self.ui.depth_radioButton.isChecked()):
            self.msg.ki_depth = self.ui.ki_slider.value()
            pub.publish(self.msg)
            self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
        if(self.ui.forward_radioButton.isChecked()):
            self.msg.ki_forward = self.ui.ki_slider.value()
            pub.publish(self.msg)
            self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
    def kd(self):
        if(self.ui.yaw_radioButton.isChecked()):
            self.msg.kd_yaw = self.ui.kd_slider.value()
            pub.publish(self.msg)
            self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
        elif(self.ui.pitch_radioButton.isChecked()):
            self.msg.kd_pitch = self.ui.kd_slider.value()
            pub.publish(self.msg)
            self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
        if(self.ui.depth_radioButton.isChecked()):
            self.msg.kd_depth = self.ui.kd_slider.value()
            pub.publish(self.msg)
            self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
        if(self.ui.forward_radioButton.isChecked()):
            self.msg.kd_forward = self.ui.kd_slider.value()
            pub.publish(self.msg)
            self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
    def setpoint(self):
        if(self.ui.yaw_radioButton.isChecked()):
            global setpoint_yaw
            self.msg.setpoint_yaw = self.ui.setpoint_slider.value()
            pub.publish(self.msg)
            setpoint_yaw = self.msg.setpoint_yaw
            self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))
        elif(self.ui.pitch_radioButton.isChecked()):
            global setpoint_pitch
            self.msg.setpoint_pitch = self.ui.setpoint_slider.value()
            pub.publish(self.msg)
            setpoint_pitch = self.msg.setpoint_pitch
            self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))
        elif(self.ui.depth_radioButton.isChecked()):
            global setpoint_depth
            self.msg.setpoint_depth = self.ui.setpoint_slider.value()
            pub.publish(self.msg)
            setpoint_depth = self.msg.setpoint_depth
            self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))
        elif(self.ui.forward_radioButton.isChecked()):
            global setpoint_forward
            self.msg.setpoint_forward = self.ui.setpoint_slider.value()
            pub.publish(self.msg)
            setpoint_forward = self.msg.setpoint_forward
            self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))
    
    def update(self):
        #update speed
        global frontpitchspeed_val
        global backpitchspeed_val
        global sideleftspeed_val
        global siderightspeed_val
        self.ui.speed_up_lineEdit.setText(str(frontpitchspeed_val))
        self.ui.speed_down_lineEdit.setText(str(backpitchspeed_val))
        self.ui.speed_left_lineEdit.setText(str(sideleftspeed_val))
        self.ui.speed_right_lineEdit.setText(str(siderightspeed_val))
        #update sliders,lineEdit according to radio button
        if(self.ui.yaw_radioButton.isChecked()):
            self.ui.kp_lineEdit.setText(str(self.msg.kp_yaw))
            self.ui.ki_lineEdit.setText(str(self.msg.ki_yaw))
            self.ui.kd_lineEdit.setText(str(self.msg.kd_yaw))
            self.ui.setpoint_lineEdit.setText(str(self.msg.setpoint_yaw))
            self.ui.kp_slider.setValue(self.msg.kp_yaw)
            self.ui.ki_slider.setValue(self.msg.ki_yaw)
            self.ui.kd_slider.setValue(self.msg.kd_yaw)
            self.ui.setpoint_slider.setValue(self.msg.setpoint_yaw)
            self.ui.setpoint_slider.setRange(-30,30)
        elif(self.ui.pitch_radioButton.isChecked()):
            self.ui.kp_lineEdit.setText(str(self.msg.kp_pitch))
            self.ui.ki_lineEdit.setText(str(self.msg.ki_pitch))
            self.ui.kd_lineEdit.setText(str(self.msg.kd_pitch))
            self.ui.setpoint_lineEdit.setText(str(self.msg.setpoint_pitch))
            self.ui.kp_slider.setValue(self.msg.kp_pitch)
            self.ui.ki_slider.setValue(self.msg.ki_pitch)
            self.ui.kd_slider.setValue(self.msg.kd_pitch)
            self.ui.setpoint_slider.setValue(self.msg.setpoint_pitch)
            self.ui.setpoint_slider.setRange(-90,90)
        elif(self.ui.depth_radioButton.isChecked()):
            self.ui.kp_lineEdit.setText(str(self.msg.kp_depth))
            self.ui.ki_lineEdit.setText(str(self.msg.ki_depth))
            self.ui.kd_lineEdit.setText(str(self.msg.kd_depth))
            self.ui.setpoint_lineEdit.setText(str(self.msg.setpoint_depth))
            self.ui.kp_slider.setValue(self.msg.kp_depth)
            self.ui.ki_slider.setValue(self.msg.ki_depth)
            self.ui.kd_slider.setValue(self.msg.kd_depth)
            self.ui.setpoint_slider.setValue(self.msg.setpoint_depth)
            self.ui.setpoint_slider.setRange(970,1300)
        elif(self.ui.forward_radioButton.isChecked()):
            self.ui.kp_lineEdit.setText(str(self.msg.kp_depth))
            self.ui.ki_lineEdit.setText(str(self.msg.ki_depth))
            self.ui.kd_lineEdit.setText(str(self.msg.kd_depth))
            self.ui.setpoint_lineEdit.setText(str(self.msg.setpoint_forward))
            self.ui.kp_slider.setValue(self.msg.kp_forward)
            self.ui.ki_slider.setValue(self.msg.ki_forward)
            self.ui.kd_slider.setValue(self.msg.kd_forward)
            self.ui.setpoint_slider.setValue(self.msg.setpoint_forward)
            self.ui.setpoint_slider.setRange(0,10)
        #update table
        self.ui.tableWidget.setItem(0, 0, QtGui.QTableWidgetItem(str(self.msg.kp_yaw)))
        self.ui.tableWidget.setItem(1, 0, QtGui.QTableWidgetItem(str(self.msg.ki_yaw)))
        self.ui.tableWidget.setItem(2, 0, QtGui.QTableWidgetItem(str(self.msg.kd_yaw)))
        self.ui.tableWidget.setItem(3, 0, QtGui.QTableWidgetItem(str(self.msg.setpoint_yaw)))
        self.ui.tableWidget.setItem(0, 1, QtGui.QTableWidgetItem(str(self.msg.kp_pitch)))
        self.ui.tableWidget.setItem(1, 1, QtGui.QTableWidgetItem(str(self.msg.ki_pitch)))
        self.ui.tableWidget.setItem(2, 1, QtGui.QTableWidgetItem(str(self.msg.kd_pitch)))
        self.ui.tableWidget.setItem(3, 1, QtGui.QTableWidgetItem(str(self.msg.setpoint_pitch)))
        self.ui.tableWidget.setItem(0, 2, QtGui.QTableWidgetItem(str(self.msg.kp_depth)))
        self.ui.tableWidget.setItem(1, 2, QtGui.QTableWidgetItem(str(self.msg.ki_depth)))
        self.ui.tableWidget.setItem(2, 2, QtGui.QTableWidgetItem(str(self.msg.kd_depth)))
        self.ui.tableWidget.setItem(3, 2, QtGui.QTableWidgetItem(str(self.msg.setpoint_depth)))
        self.ui.tableWidget.setItem(0, 3, QtGui.QTableWidgetItem(str(self.msg.kp_forward)))
        self.ui.tableWidget.setItem(1, 3, QtGui.QTableWidgetItem(str(self.msg.ki_forward)))
        self.ui.tableWidget.setItem(2, 3, QtGui.QTableWidgetItem(str(self.msg.kd_forward)))
        self.ui.tableWidget.setItem(3, 3, QtGui.QTableWidgetItem(str(self.msg.setpoint_forward)))
                
    def speed(self):
        self.sub_front=rospy.Subscriber("frontpitchspeed",UInt16,self.frontcallback)
        self.sub_back=rospy.Subscriber("backpitchspeed",UInt16,self.backcallback)
        self.sub_left=rospy.Subscriber("sideleftspeed",UInt16,self.leftcallback)
        self.sub_right=rospy.Subscriber("siderightspeed",UInt16,self.rightcallback)
        #function update() handles the display part

    def frontcallback(self,msg):
        global frontpitchspeed_val
        frontpitchspeed_val = msg.data
    def backcallback(self,msg):
        global backpitchspeed_val
        backpitchspeed_val = msg.data
    def leftcallback(self,msg):
        global sideleftspeed_val
        sideleftspeed_val = msg.data
    def rightcallback(self,msg):
        global siderightspeed_val
        siderightspeed_val = msg.data

    def graph(self):
        self.graph_depth()
        self.graph_pitch()
        self.graph_yaw()
        #self.graph_forward() #UNCOMMENT IT AFTER ENSURING TOPIC TYPE AND forwardY parameter 

    def graph_depth(self):
        rospy.Subscriber('depth_value',Float64,self.depthCallback)
        self.startTimeNow = rospy.get_rostime()
        self.startTime = self.startTimeNow.secs + 10**-9*self.startTimeNow.nsecs
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_d)
        self._timer.start(25) #calls play_d ever 50 ms

    def play_d(self):
        self.ui.depth_graphicsView.plot(self.depthX, self.depthY)
        self.ui.depth_graphicsView.plot(self.depthX, self.setpoint_d)

    def depthCallback(self,msg):
        global setpoint_depth
        self.depthTimeNow = rospy.get_rostime()
        self.depthTime = self.depthTimeNow.secs + 10**-9*self.depthTimeNow.nsecs
        self.depthY.append(msg.data)
        self.depthX.append(self.depthTime-self.startTime)
        self.setpoint_d.append(setpoint_depth)

    def graph_pitch(self):
        rospy.Subscriber("/tiburon/ins_data",ins_data,self.pitchCallback)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_p)
        self._timer.start(25)
        
    def play_p(self):
        self.ui.pitch_graphicsView.plot(self.pitchX, self.pitchY)
        self.ui.pitch_graphicsView.plot(self.pitchX, self.setpoint_p)

    def pitchCallback(self,msg):
        global setpoint_pitch
        self.pitchTimeNow = rospy.get_rostime()
        self.pitchTime = self.pitchTimeNow.secs + 10**-9*self.pitchTimeNow.nsecs
        self.pitchY.append(msg.YPR.y)
        self.pitchX.append(self.pitchTime-self.startTime)
        self.setpoint_p.append(setpoint_pitch)

    def graph_yaw(self):
        rospy.Subscriber("/tiburon/ins_data",ins_data,self.yawCallback)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_y)
        self._timer.start(25)
        
    def play_y(self):
        self.ui.yaw_graphicsView.plot(self.yawX, self.yawY)
        self.ui.yaw_graphicsView.plot(self.yawX, self.setpoint_y)

    def yawCallback(self,msg):
        global setpoint_yaw
        self.yawTimeNow = rospy.get_rostime()
        self.yawTime = self.yawTimeNow.secs + 10**-9*self.yawTimeNow.nsecs
        self.yawY.append(msg.YPR.x)
        self.yawX.append(self.yawTime-self.startTime)
        self.setpoint_y.append(setpoint_yaw)

    
    def graph_forward(self):
        rospy.Subscriber("/true_velocity",Vector3,self.forwardCallback) #TODO WRITE THE CORRECT TOPIC TYPE
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play_f)
        self._timer.start(50)

    def play_f(self):
        self.ui.forward_graphicsView.plot(self.forwardX, self.forwardY)
        self.ui.forward_graphicsView.plot(self.forwardX, self.setpoint_y)

    def forwardCallback(self,msg):
        global setpoint_forward
        self.forwardTimeNow = rospy.get_rostime()
        self.forwardTime = self.forwardTimeNow.secs + 10**-9*self.forwardTimeNow.nsecs
        self.forwardY.append(msg.x) #TODO CHANGE PARAMETER ACCORDING TO TOPIC!!!!
        self.forwardX.append(self.forwardTime-self.startTime)
        self.setpoint_p.append(setpoint_forward)

def main():
    rospy.init_node("tabs_ui_code")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    obj = Panel()
    obj.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
