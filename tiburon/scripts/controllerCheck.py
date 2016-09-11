#!/usr/bin/env python
from controllerCheckUI import Ui_MainWindow
from PyQt4 import QtGui,QtCore
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from tiburon.msg import ins_data
import rospy
import signal
import sys

class valueSetter(QtGui.QMainWindow):
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.depthSlider.setMinimum(950)
        self.ui.depthSlider.setMaximum(1050)
        self.ui.depthSlider.setValue(1000)
        self.ui.depthSlider.setTickInterval(1)
        self.ui.depthSlider.valueChanged.connect(self.depthPublish)
        self.ui.depthLabel.setText('D: '+str(self.ui.depthSlider.value()))

        self.ui.pitchSlider.setMinimum(-90)
        self.ui.pitchSlider.setMaximum(90)
        self.ui.pitchSlider.setValue(0)
        self.ui.pitchSlider.setTickInterval(1)
        self.ui.pitchSlider.valueChanged.connect(self.pitchAndYawPublish)
        self.ui.pitchLabel.setText('P: '+str(self.ui.pitchSlider.value()))

        self.ui.yawSlider.setMinimum(-90)
        self.ui.yawSlider.setMaximum(90)
        self.ui.yawSlider.setValue(0)
        self.ui.yawSlider.setTickInterval(1)
        self.ui.yawSlider.valueChanged.connect(self.pitchAndYawPublish)
        self.ui.yawLabel.setText('Y: '+str(self.ui.yawSlider.value()))

        self.ui.velSlider.setMinimum(-1500)
        self.ui.velSlider.setMaximum(1500)
        self.ui.velSlider.setValue(0)
        self.ui.velSlider.setTickInterval(1)
        self.ui.velSlider.valueChanged.connect(self.velPublish)
        self.ui.velLabel.setText('V: '+str(self.ui.velSlider.value()))

        self.depthPublisher = rospy.Publisher('/depth_value',Float64,queue_size=1)
        self.pitchAndYawPublisher = rospy.Publisher('tiburon/ins_data',ins_data,queue_size=1)
        self.velPublisher = rospy.Publisher('/tiburon/Delta_velocity',Vector3,queue_size=1)


    def depthPublish(self):
        self.msg = Float64()
        self.msg.data = self.ui.depthSlider.value()
        self.depthPublisher.publish(self.msg)
        self.ui.depthLabel.setText('D: '+str(self.ui.depthSlider.value()))

    def pitchAndYawPublish(self):
        self.msg = ins_data()
        self.msg.YPR.y = self.ui.pitchSlider.value()
        self.msg.YPR.x = self.ui.yawSlider.value()
        self.pitchAndYawPublisher.publish(self.msg)
        self.ui.pitchLabel.setText('P: '+str(self.ui.pitchSlider.value()))
        self.ui.yawLabel.setText('Y: '+str(self.ui.yawSlider.value()))

    def velPublish(self):
        self.msg = Vector3()
        self.msg.x = self.ui.velSlider.value()/100
        self.velPublisher.publish(self.msg)
        self.ui.velLabel.setText('V: '+str(self.ui.velSlider.value()))

def main():
    rospy.init_node("controllerCheck")
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app=QtGui.QApplication(sys.argv)
    window = valueSetter()
    window.show()
    sys.exit(app.exec_())


if __name__=='__main__':
    main()
