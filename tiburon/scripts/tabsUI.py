# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Sat Nov  5 12:39:24 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1231, 879)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.tabWidget = QtGui.QTabWidget(self.centralWidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 0, 1201, 761))
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.kp_slider = QtGui.QSlider(self.tab)
        self.kp_slider.setGeometry(QtCore.QRect(150, 120, 311, 29))
        self.kp_slider.setOrientation(QtCore.Qt.Horizontal)
        self.kp_slider.setObjectName(_fromUtf8("kp_slider"))
        self.ki_slider = QtGui.QSlider(self.tab)
        self.ki_slider.setGeometry(QtCore.QRect(150, 160, 311, 29))
        self.ki_slider.setOrientation(QtCore.Qt.Horizontal)
        self.ki_slider.setObjectName(_fromUtf8("ki_slider"))
        self.kd_slider = QtGui.QSlider(self.tab)
        self.kd_slider.setGeometry(QtCore.QRect(150, 200, 311, 31))
        self.kd_slider.setOrientation(QtCore.Qt.Horizontal)
        self.kd_slider.setObjectName(_fromUtf8("kd_slider"))
        self.setpoint_slider = QtGui.QSlider(self.tab)
        self.setpoint_slider.setGeometry(QtCore.QRect(150, 240, 311, 29))
        self.setpoint_slider.setOrientation(QtCore.Qt.Horizontal)
        self.setpoint_slider.setObjectName(_fromUtf8("setpoint_slider"))
        self.yaw_radioButton = QtGui.QRadioButton(self.tab)
        self.yaw_radioButton.setGeometry(QtCore.QRect(70, 60, 117, 22))
        self.yaw_radioButton.setObjectName(_fromUtf8("yaw_radioButton"))
        self.pitch_radioButton = QtGui.QRadioButton(self.tab)
        self.pitch_radioButton.setGeometry(QtCore.QRect(200, 60, 117, 22))
        self.pitch_radioButton.setObjectName(_fromUtf8("pitch_radioButton"))
        self.depth_radioButton = QtGui.QRadioButton(self.tab)
        self.depth_radioButton.setGeometry(QtCore.QRect(340, 60, 117, 22))
        self.depth_radioButton.setObjectName(_fromUtf8("depth_radioButton"))
        self.forward_radioButton = QtGui.QRadioButton(self.tab)
        self.forward_radioButton.setGeometry(QtCore.QRect(460, 60, 117, 22))
        self.forward_radioButton.setObjectName(_fromUtf8("forward_radioButton"))
        self.kp_lineEdit = QtGui.QLineEdit(self.tab)
        self.kp_lineEdit.setGeometry(QtCore.QRect(480, 120, 41, 31))
        self.kp_lineEdit.setReadOnly(False)
        self.kp_lineEdit.setObjectName(_fromUtf8("kp_lineEdit"))
        self.ki_lineEdit = QtGui.QLineEdit(self.tab)
        self.ki_lineEdit.setGeometry(QtCore.QRect(480, 160, 41, 31))
        self.ki_lineEdit.setReadOnly(False)
        self.ki_lineEdit.setObjectName(_fromUtf8("ki_lineEdit"))
        self.kd_lineEdit = QtGui.QLineEdit(self.tab)
        self.kd_lineEdit.setGeometry(QtCore.QRect(480, 200, 41, 31))
        self.kd_lineEdit.setReadOnly(False)
        self.kd_lineEdit.setObjectName(_fromUtf8("kd_lineEdit"))
        self.setpoint_lineEdit = QtGui.QLineEdit(self.tab)
        self.setpoint_lineEdit.setGeometry(QtCore.QRect(480, 240, 41, 31))
        self.setpoint_lineEdit.setReadOnly(False)
        self.setpoint_lineEdit.setObjectName(_fromUtf8("setpoint_lineEdit"))
        self.label = QtGui.QLabel(self.tab)
        self.label.setGeometry(QtCore.QRect(70, 120, 67, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(self.tab)
        self.label_2.setGeometry(QtCore.QRect(70, 160, 67, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.tab)
        self.label_3.setGeometry(QtCore.QRect(70, 200, 67, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.tab)
        self.label_4.setGeometry(QtCore.QRect(70, 240, 67, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.tab)
        self.label_5.setGeometry(QtCore.QRect(750, 40, 67, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.speed_up_lineEdit = QtGui.QLineEdit(self.tab)
        self.speed_up_lineEdit.setGeometry(QtCore.QRect(750, 90, 71, 31))
        self.speed_up_lineEdit.setReadOnly(True)
        self.speed_up_lineEdit.setObjectName(_fromUtf8("speed_up_lineEdit"))
        self.speed_left_lineEdit = QtGui.QLineEdit(self.tab)
        self.speed_left_lineEdit.setGeometry(QtCore.QRect(670, 140, 71, 31))
        self.speed_left_lineEdit.setReadOnly(True)
        self.speed_left_lineEdit.setObjectName(_fromUtf8("speed_left_lineEdit"))
        self.speed_down_lineEdit = QtGui.QLineEdit(self.tab)
        self.speed_down_lineEdit.setGeometry(QtCore.QRect(750, 190, 71, 31))
        self.speed_down_lineEdit.setReadOnly(True)
        self.speed_down_lineEdit.setObjectName(_fromUtf8("speed_down_lineEdit"))
        self.speed_right_lineEdit = QtGui.QLineEdit(self.tab)
        self.speed_right_lineEdit.setGeometry(QtCore.QRect(830, 140, 71, 31))
        self.speed_right_lineEdit.setReadOnly(True)
        self.speed_right_lineEdit.setObjectName(_fromUtf8("speed_right_lineEdit"))
        self.tableWidget = QtGui.QTableWidget(self.tab)
        self.tableWidget.setGeometry(QtCore.QRect(150, 360, 421, 151))
        self.tableWidget.setRowCount(4)
        self.tableWidget.setColumnCount(4)
        self.tableWidget.setObjectName(_fromUtf8("tableWidget"))
        self.label_6 = QtGui.QLabel(self.tab)
        self.label_6.setGeometry(QtCore.QRect(180, 330, 67, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_7 = QtGui.QLabel(self.tab)
        self.label_7.setGeometry(QtCore.QRect(280, 330, 67, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.label_8 = QtGui.QLabel(self.tab)
        self.label_8.setGeometry(QtCore.QRect(370, 330, 67, 17))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(self.tab)
        self.label_9.setGeometry(QtCore.QRect(470, 330, 67, 17))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.label_10 = QtGui.QLabel(self.tab)
        self.label_10.setGeometry(QtCore.QRect(70, 480, 67, 17))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_11 = QtGui.QLabel(self.tab)
        self.label_11.setGeometry(QtCore.QRect(70, 420, 67, 17))
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.label_12 = QtGui.QLabel(self.tab)
        self.label_12.setGeometry(QtCore.QRect(70, 390, 67, 17))
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.label_13 = QtGui.QLabel(self.tab)
        self.label_13.setGeometry(QtCore.QRect(70, 450, 67, 17))
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.resetButton = QtGui.QPushButton(self.tab)
        self.resetButton.setGeometry(QtCore.QRect(750, 390, 91, 71))
        self.resetButton.setObjectName(_fromUtf8("resetButton"))
        self.lineEdit = QtGui.QLineEdit(self.tab)
        self.lineEdit.setGeometry(QtCore.QRect(530, 120, 51, 31))
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.lineEdit_2 = QtGui.QLineEdit(self.tab)
        self.lineEdit_2.setGeometry(QtCore.QRect(530, 160, 51, 31))
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.lineEdit_3 = QtGui.QLineEdit(self.tab)
        self.lineEdit_3.setGeometry(QtCore.QRect(530, 200, 51, 31))
        self.lineEdit_3.setObjectName(_fromUtf8("lineEdit_3"))
        self.lineEdit_4 = QtGui.QLineEdit(self.tab)
        self.lineEdit_4.setGeometry(QtCore.QRect(530, 240, 51, 31))
        self.lineEdit_4.setObjectName(_fromUtf8("lineEdit_4"))
        self.label_18 = QtGui.QLabel(self.tab)
        self.label_18.setGeometry(QtCore.QRect(540, 90, 31, 17))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName(_fromUtf8("tab_2"))
        self.yaw_graphicsView = PlotWidget(self.tab_2)
        self.yaw_graphicsView.setGeometry(QtCore.QRect(60, 50, 351, 201))
        self.yaw_graphicsView.setObjectName(_fromUtf8("yaw_graphicsView"))
        self.label_14 = QtGui.QLabel(self.tab_2)
        self.label_14.setGeometry(QtCore.QRect(60, 20, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_14.setFont(font)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.label_15 = QtGui.QLabel(self.tab_2)
        self.label_15.setGeometry(QtCore.QRect(460, 20, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_15.setFont(font)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.label_16 = QtGui.QLabel(self.tab_2)
        self.label_16.setGeometry(QtCore.QRect(70, 350, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_16.setFont(font)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.label_17 = QtGui.QLabel(self.tab_2)
        self.label_17.setGeometry(QtCore.QRect(440, 350, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_17.setFont(font)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.pitch_graphicsView = PlotWidget(self.tab_2)
        self.pitch_graphicsView.setGeometry(QtCore.QRect(450, 50, 351, 201))
        self.pitch_graphicsView.setObjectName(_fromUtf8("pitch_graphicsView"))
        self.depth_graphicsView = PlotWidget(self.tab_2)
        self.depth_graphicsView.setGeometry(QtCore.QRect(60, 390, 351, 201))
        self.depth_graphicsView.setObjectName(_fromUtf8("depth_graphicsView"))
        self.forward_graphicsView = PlotWidget(self.tab_2)
        self.forward_graphicsView.setGeometry(QtCore.QRect(440, 390, 351, 201))
        self.forward_graphicsView.setObjectName(_fromUtf8("forward_graphicsView"))
        self.comboBox = QtGui.QComboBox(self.tab_2)
        self.comboBox.setGeometry(QtCore.QRect(840, 60, 211, 61))
        self.comboBox.setObjectName(_fromUtf8("comboBox"))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.selective_graph = PlotWidget(self.tab_2)
        self.selective_graph.setGeometry(QtCore.QRect(840, 150, 331, 281))
        self.selective_graph.setObjectName(_fromUtf8("selective_graph"))
        self.val_yaw_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.val_yaw_lineEdit.setGeometry(QtCore.QRect(120, 260, 91, 31))
        self.val_yaw_lineEdit.setObjectName(_fromUtf8("val_yaw_lineEdit"))
        self.setpoint_yaw_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.setpoint_yaw_lineEdit.setGeometry(QtCore.QRect(310, 260, 101, 31))
        self.setpoint_yaw_lineEdit.setObjectName(_fromUtf8("setpoint_yaw_lineEdit"))
        self.label_19 = QtGui.QLabel(self.tab_2)
        self.label_19.setGeometry(QtCore.QRect(70, 270, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_19.setFont(font)
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.label_20 = QtGui.QLabel(self.tab_2)
        self.label_20.setGeometry(QtCore.QRect(230, 270, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_20.setFont(font)
        self.label_20.setObjectName(_fromUtf8("label_20"))
        self.setpoint_pitch_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.setpoint_pitch_lineEdit.setGeometry(QtCore.QRect(710, 260, 91, 31))
        self.setpoint_pitch_lineEdit.setObjectName(_fromUtf8("setpoint_pitch_lineEdit"))
        self.val_pitch_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.val_pitch_lineEdit.setGeometry(QtCore.QRect(530, 260, 91, 31))
        self.val_pitch_lineEdit.setObjectName(_fromUtf8("val_pitch_lineEdit"))
        self.label_21 = QtGui.QLabel(self.tab_2)
        self.label_21.setGeometry(QtCore.QRect(480, 270, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_21.setFont(font)
        self.label_21.setObjectName(_fromUtf8("label_21"))
        self.label_22 = QtGui.QLabel(self.tab_2)
        self.label_22.setGeometry(QtCore.QRect(640, 270, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_22.setFont(font)
        self.label_22.setObjectName(_fromUtf8("label_22"))
        self.label_23 = QtGui.QLabel(self.tab_2)
        self.label_23.setGeometry(QtCore.QRect(80, 610, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_23.setFont(font)
        self.label_23.setObjectName(_fromUtf8("label_23"))
        self.label_24 = QtGui.QLabel(self.tab_2)
        self.label_24.setGeometry(QtCore.QRect(70, 650, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_24.setFont(font)
        self.label_24.setObjectName(_fromUtf8("label_24"))
        self.setpoint_depth_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.setpoint_depth_lineEdit.setGeometry(QtCore.QRect(140, 640, 241, 31))
        self.setpoint_depth_lineEdit.setObjectName(_fromUtf8("setpoint_depth_lineEdit"))
        self.val_depth_lineEdit = QtGui.QLineEdit(self.tab_2)
        self.val_depth_lineEdit.setGeometry(QtCore.QRect(130, 600, 251, 31))
        self.val_depth_lineEdit.setObjectName(_fromUtf8("val_depth_lineEdit"))
        self.lineEdit_11 = QtGui.QLineEdit(self.tab_2)
        self.lineEdit_11.setGeometry(QtCore.QRect(700, 610, 91, 31))
        self.lineEdit_11.setObjectName(_fromUtf8("lineEdit_11"))
        self.label_25 = QtGui.QLabel(self.tab_2)
        self.label_25.setGeometry(QtCore.QRect(630, 620, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_25.setFont(font)
        self.label_25.setObjectName(_fromUtf8("label_25"))
        self.lineEdit_12 = QtGui.QLineEdit(self.tab_2)
        self.lineEdit_12.setGeometry(QtCore.QRect(520, 610, 91, 31))
        self.lineEdit_12.setObjectName(_fromUtf8("lineEdit_12"))
        self.label_26 = QtGui.QLabel(self.tab_2)
        self.label_26.setGeometry(QtCore.QRect(470, 620, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_26.setFont(font)
        self.label_26.setObjectName(_fromUtf8("label_26"))
        self.tabWidget.addTab(self.tab_2, _fromUtf8(""))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 1231, 25))
        self.menuBar.setObjectName(_fromUtf8("menuBar"))
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(MainWindow)
        self.mainToolBar.setObjectName(_fromUtf8("mainToolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName(_fromUtf8("statusBar"))
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.yaw_radioButton.setText(_translate("MainWindow", "Yaw", None))
        self.pitch_radioButton.setText(_translate("MainWindow", "Pitch", None))
        self.depth_radioButton.setText(_translate("MainWindow", "Depth", None))
        self.forward_radioButton.setText(_translate("MainWindow", "Forward", None))
        self.label.setText(_translate("MainWindow", "kp", None))
        self.label_2.setText(_translate("MainWindow", "ki", None))
        self.label_3.setText(_translate("MainWindow", "kd", None))
        self.label_4.setText(_translate("MainWindow", "setpoint", None))
        self.label_5.setText(_translate("MainWindow", "Speed", None))
        self.label_6.setText(_translate("MainWindow", "Yaw", None))
        self.label_7.setText(_translate("MainWindow", "Pitch", None))
        self.label_8.setText(_translate("MainWindow", "Depth", None))
        self.label_9.setText(_translate("MainWindow", "Forward", None))
        self.label_10.setText(_translate("MainWindow", "setpoint", None))
        self.label_11.setText(_translate("MainWindow", "ki", None))
        self.label_12.setText(_translate("MainWindow", "kp", None))
        self.label_13.setText(_translate("MainWindow", "kd", None))
        self.resetButton.setText(_translate("MainWindow", "Reset", None))
        self.label_18.setText(_translate("MainWindow", "Set", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "PID", None))
        self.label_14.setText(_translate("MainWindow", "Yaw", None))
        self.label_15.setText(_translate("MainWindow", "Pitch", None))
        self.label_16.setText(_translate("MainWindow", "Depth", None))
        self.label_17.setText(_translate("MainWindow", "Forward", None))
        self.comboBox.setItemText(0, _translate("MainWindow", "YAW", None))
        self.comboBox.setItemText(1, _translate("MainWindow", "PITCH", None))
        self.label_19.setText(_translate("MainWindow", "Value", None))
        self.label_20.setText(_translate("MainWindow", "Setpoint", None))
        self.label_21.setText(_translate("MainWindow", "Value", None))
        self.label_22.setText(_translate("MainWindow", "Setpoint", None))
        self.label_23.setText(_translate("MainWindow", "Value", None))
        self.label_24.setText(_translate("MainWindow", "Setpoint", None))
        self.label_25.setText(_translate("MainWindow", "Setpoint", None))
        self.label_26.setText(_translate("MainWindow", "Value", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Graphs", None))

from pyqtgraph import PlotWidget
