#!/usr/bin/env python
from tabsUI import Ui_MainWindow
import sys
import rospy
from PyQt4 import QtGui,QtCore
import signal
from tiburon.msg import pid
from std_msgs.msg import String,UInt16,Float64,Float32
from tiburon.msg import ins_data
from geometry_msgs.msg import Vector3  #CHANGE IF REQUIRED!!!!!!!!!
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.msg import Config

#currentIndexChanged  <- comboBox

pub=rospy.Publisher("pid_values",pid,queue_size=1)
#TODO : set slider value according to the value typed (According to radio button signal)
#TODO: why slider resets to max when value stops(subscribed value)

frontpitchspeed_val, backpitchspeed_val, sideleftspeed_val, siderightspeed_val = 0,0,0,0
setpoint_yaw, setpoint_pitch, setpoint_depth, setpoint_forward = 0.0,0.0,0.0,0.0
val_yaw, val_pitch, val_depth, val_forward = 0.0,0.0,0.0,0.0

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

        self.select_pitchX = []
        self.select_pitchY = []
        self.select_setpoint_p = []
        self.select_yawX = []
        self.select_yawY = []
        self.select_setpoint_y = []

        #update table
        self.sub = rospy.Subscriber("/tiburonController/parameter_updates",Config,self.update_table)

        #TODO: set slider according to line edit
        #self.ui.kp_lineEdit.textEdited.connect(self.update_kp_slider)
        
        #self.signals()
        self.setslidervalues()        
        self.msg = pid()

        #speed
        self.speed()

        #tab 2: graphs
        self.graph()

        #button
        self.ui.resetButton.clicked.connect(self.reset)

        self.c = 1 #selective graph(default:yaw)

    def update_table(self, msg):
        global setpoint_yaw
        global setpoint_pitch
        global setpoint_depth
        global setpoint_forward
        dict = {'kp_yaw' : 00, 'ki_yaw' : 10, 'kd_yaw' : 20, 'setpoint_yaw' : 30,
                'kp_pitch' : 01, 'ki_pitch' : 11, 'kd_pitch' : 21, 'setpoint_pitch' : 31,
                'kp_depth' : 02, 'ki_depth' : 12, 'kd_depth' : 22, 'setpoint_depth' : 32,
                'kp_for' : 03, 'ki_for' : 13, 'kd_for' : 23, 'setpoint_forward' : 33}
        for i in range(0,17):
            if msg.doubles[i].name == "desAccl":
                continue
            row = dict[msg.doubles[i].name] / 10
            col = dict[msg.doubles[i].name] % 10
            self.ui.tableWidget.setItem(row, col, QtGui.QTableWidgetItem(str(msg.doubles[i].value)))

        setpoint_yaw = float(self.ui.tableWidget.item(3,0).text())
        setpoint_pitch = float(self.ui.tableWidget.item(3,1).text())
        setpoint_depth = float(self.ui.tableWidget.item(3,2).text())
        setpoint_forward = float(self.ui.tableWidget.item(3,3).text())
    
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
        self.ui.kp_slider.setRange(0,100)
        self.ui.ki_slider.setRange(0,100)
        self.ui.kd_slider.setRange(0,100)
        self.ui.setpoint_slider.setRange(-180,1300)

        self.ui.kp_slider.setValue(0)
        self.ui.ki_slider.setValue(0)
        self.ui.kd_slider.setValue(0)
        self.ui.setpoint_slider.setValue(0)

        self.ui.kp_lineEdit.setText(str(self.ui.kp_slider.value()))
        self.ui.ki_lineEdit.setText(str(self.ui.ki_slider.value()))
        self.ui.kd_lineEdit.setText(str(self.ui.kd_slider.value()))
        self.ui.setpoint_lineEdit.setText(str(self.ui.setpoint_slider.value()))

    '''
    def signals(self):
        self.connect(self.ui.kp_slider,QtCore.SIGNAL("valueChanged(int)"),self.kp)
        self.connect(self.ui.ki_slider,QtCore.SIGNAL("valueChanged(int)"),self.ki)
        self.connect(self.ui.kd_slider,QtCore.SIGNAL("valueChanged(int)"),self.kd)
        self.connect(self.ui.setpoint_slider,QtCore.SIGNAL("valueChanged(int)"),self.setpoint)
        self.connect(self.ui.comboBox, QtCore.SIGNAL("currentIndexChanged(QString)"), self.graph_select)
        #update slider according to value entered
        #self.connect(self.ui.kp_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_kp_slider)
        #self.connect(self.ui.ki_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_ki_slider)
        #self.connect(self.ui.kd_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_kd_slider)
        #self.connect(self.ui.setpoint_lineEdit,QtCore.SIGNAL("textEdited("")",self.update_setpoint_slider)
    '''

    @QtCore.pyqtSlot(str)
    

    def update(self):
        #update speed
        global frontpitchspeed_val
        global backpitchspeed_val
        global sideleftspeed_val
        global siderightspeed_val
        global setpoint_yaw
        global setpoint_pitch
        global setpoint_depth
        global setpoint_forward
        global val_yaw
        global val_pitch
        global val_depth
        global val_forward
        
        self.ui.speed_up_lineEdit.setText(str(frontpitchspeed_val))
        self.ui.speed_down_lineEdit.setText(str(backpitchspeed_val))
        self.ui.speed_left_lineEdit.setText(str(sideleftspeed_val))
        self.ui.speed_right_lineEdit.setText(str(siderightspeed_val))

        self.ui.val_yaw_lineEdit.setText(str(val_yaw))
        self.ui.val_pitch_lineEdit.setText(str(val_pitch))
        self.ui.val_depth_lineEdit.setText(str(val_depth))
        #self.ui.val_forward_lineEdit.setText(str(val_forward))

        self.ui.setpoint_yaw_lineEdit.setText(str(setpoint_yaw))
        self.ui.setpoint_pitch_lineEdit.setText(str(setpoint_pitch))
        self.ui.setpoint_depth_lineEdit.setText(str(setpoint_depth))
        #self.ui.setpoint_forward_lineEdit.setText(str(setpoint_forward))

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
        rospy.Subscriber('/depth_value',Float64,self.depthCallback)
        rospy.Subscriber("/tiburon/ins_data",ins_data,self.pitchCallback)
        rospy.Subscriber("/tiburon/ins_data",ins_data,self.yawCallback)
        #rospy.Subscriber("/tiburon/true_velocity",Vector3,self.forwardCallback) #TODO WRITE THE CORRECT TOPIC TYPE        
        self.startTimeNow = rospy.get_rostime()
        self.startTime = self.startTimeNow.secs + 10**-9*self.startTimeNow.nsecs
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(200) #calls play_d ever 50 ms

    def play(self):
        self.update()
        self.ui.depth_graphicsView.plot(self.depthX, self.depthY)
        self.ui.depth_graphicsView.plot(self.depthX, self.setpoint_d) 
        self.ui.pitch_graphicsView.plot(self.pitchX, self.pitchY)
        self.ui.pitch_graphicsView.plot(self.pitchX, self.setpoint_p)
        self.ui.yaw_graphicsView.plot(self.yawX, self.yawY)
        self.ui.yaw_graphicsView.plot(self.yawX, self.setpoint_y)
        #self.ui.forward_graphicsView.plot(self.forwardX, self.forwardY)
        #self.ui.forward_graphicsView.plot(self.forwardX, self.setpoint_y)
        
        #selective graph
        if(self.c == 2):
            self.ui.selective_graph.plot(self.select_pitchX, self.select_pitchY)  
            self.ui.selective_graph.plot(self.select_pitchX, self.select_setpoint_p) 
        elif(self.c == 1):
            self.ui.selective_graph.plot(self.select_yawX, self.select_yawY) 
            self.ui.selective_graph.plot(self.select_yawX, self.select_setpoint_y) 

    def depthCallback(self,msg):
        global setpoint_depth
        global val_depth
        self.depthTimeNow = rospy.get_rostime()
        self.depthTime = self.depthTimeNow.secs + 10**-9*self.depthTimeNow.nsecs
        self.depthY.append(msg.data)
        self.depthX.append(self.depthTime-self.startTime)
        self.setpoint_d.append(setpoint_depth)
        val_depth = msg.data
            
    def pitchCallback(self,msg):
        global setpoint_pitch
        global val_pitch
        self.pitchTimeNow = rospy.get_rostime()
        self.pitchTime = self.pitchTimeNow.secs + 10**-9*self.pitchTimeNow.nsecs
        self.pitchY.append(msg.YPR.y)
        self.pitchX.append(self.pitchTime-self.startTime)
        self.setpoint_p.append(setpoint_pitch)
        val_pitch =  msg.YPR.y     

    def yawCallback(self,msg):
        global setpoint_yaw
        global val_yaw
        self.yawTimeNow = rospy.get_rostime()
        self.yawTime = self.yawTimeNow.secs + 10**-9*self.yawTimeNow.nsecs
        self.yawY.append(msg.YPR.x)
        self.yawX.append(self.yawTime-self.startTime)
        self.setpoint_y.append(setpoint_yaw)
        val_yaw = msg.YPR.x

    def forwardCallback(self,msg):
        global setpoint_forward
        self.forwardTimeNow = rospy.get_rostime()
        self.forwardTime = self.forwardTimeNow.secs + 10**-9*self.forwardTimeNow.nsecs
        self.forwardY.append(msg.x) #TODO CHANGE PARAMETER ACCORDING TO TOPIC!!!!
        self.forwardX.append(self.forwardTime-self.startTime)
        self.setpoint_f.append(setpoint_forward)
        val_forward = msg.x 
        

def main():
    rospy.init_node("tabs_ui_code")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    obj = Panel()
    obj.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
