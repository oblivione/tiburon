# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Thu Aug 18 22:56:58 2016
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
        MainWindow.resize(1246, 731)
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(222, 222, 222))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(126, 126, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(222, 222, 222))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ToolTipText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(222, 222, 222))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(126, 126, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(222, 222, 222))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ToolTipText, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(222, 222, 222))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(126, 126, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(94, 94, 94))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(189, 189, 189))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ToolTipText, brush)
        MainWindow.setPalette(palette)
        MainWindow.setAutoFillBackground(True)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.label = QtGui.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(450, 20, 231, 51))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("URW Bookman L"))
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.groupBox = QtGui.QGroupBox(self.centralWidget)
        self.groupBox.setGeometry(QtCore.QRect(670, 100, 441, 161))
        self.groupBox.setAutoFillBackground(True)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.graphicsView = PlotWidget(self.groupBox)
        self.graphicsView.setGeometry(QtCore.QRect(90, 0, 351, 161))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.groupBox_2 = QtGui.QGroupBox(self.centralWidget)
        self.groupBox_2.setGeometry(QtCore.QRect(670, 300, 441, 161))
        self.groupBox_2.setAutoFillBackground(True)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.graphicsView_2 = PlotWidget(self.groupBox_2)
        self.graphicsView_2.setGeometry(QtCore.QRect(90, 0, 351, 161))
        self.graphicsView_2.setObjectName(_fromUtf8("graphicsView_2"))
        self.groupBox_3 = QtGui.QGroupBox(self.centralWidget)
        self.groupBox_3.setGeometry(QtCore.QRect(670, 480, 441, 161))
        self.groupBox_3.setAutoFillBackground(True)
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.graphicsView_3 = PlotWidget(self.groupBox_3)
        self.graphicsView_3.setGeometry(QtCore.QRect(90, 0, 351, 161))
        self.graphicsView_3.setObjectName(_fromUtf8("graphicsView_3"))
        self.groupBox_4 = QtGui.QGroupBox(self.centralWidget)
        self.groupBox_4.setGeometry(QtCore.QRect(80, 100, 451, 181))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_4.setFont(font)
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.label_2 = QtGui.QLabel(self.groupBox_4)
        self.label_2.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.groupBox_4)
        self.label_3.setGeometry(QtCore.QRect(10, 60, 67, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.groupBox_4)
        self.label_4.setGeometry(QtCore.QRect(10, 90, 67, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.kp_yaw_lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.kp_yaw_lineEdit.setGeometry(QtCore.QRect(130, 30, 171, 27))
        self.kp_yaw_lineEdit.setObjectName(_fromUtf8("kp_yaw_lineEdit"))
        self.ki_yaw_lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.ki_yaw_lineEdit.setGeometry(QtCore.QRect(130, 60, 171, 27))
        self.ki_yaw_lineEdit.setObjectName(_fromUtf8("ki_yaw_lineEdit"))
        self.kd_yaw_lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.kd_yaw_lineEdit.setGeometry(QtCore.QRect(130, 90, 171, 27))
        self.kd_yaw_lineEdit.setObjectName(_fromUtf8("kd_yaw_lineEdit"))
        self.label_11 = QtGui.QLabel(self.groupBox_4)
        self.label_11.setGeometry(QtCore.QRect(10, 120, 91, 16))
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.ckpoint_lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.ckpoint_lineEdit.setGeometry(QtCore.QRect(130, 120, 171, 27))
        self.ckpoint_lineEdit.setObjectName(_fromUtf8("ckpoint_lineEdit"))
        self.label_14 = QtGui.QLabel(self.groupBox_4)
        self.label_14.setGeometry(QtCore.QRect(10, 150, 91, 16))
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.setpoint_yaw_lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.setpoint_yaw_lineEdit.setGeometry(QtCore.QRect(130, 150, 171, 27))
        self.setpoint_yaw_lineEdit.setObjectName(_fromUtf8("setpoint_yaw_lineEdit"))
        self.groupBox_5 = QtGui.QGroupBox(self.centralWidget)
        self.groupBox_5.setGeometry(QtCore.QRect(80, 300, 451, 181))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_5.setFont(font)
        self.groupBox_5.setObjectName(_fromUtf8("groupBox_5"))
        self.label_5 = QtGui.QLabel(self.groupBox_5)
        self.label_5.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.label_6 = QtGui.QLabel(self.groupBox_5)
        self.label_6.setGeometry(QtCore.QRect(10, 60, 67, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_7 = QtGui.QLabel(self.groupBox_5)
        self.label_7.setGeometry(QtCore.QRect(10, 90, 67, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.kp_pitch_lineEditlineEdi = QtGui.QLineEdit(self.groupBox_5)
        self.kp_pitch_lineEditlineEdi.setGeometry(QtCore.QRect(130, 30, 171, 27))
        self.kp_pitch_lineEditlineEdi.setObjectName(_fromUtf8("kp_pitch_lineEditlineEdi"))
        self.ki_pitch_lineEdit = QtGui.QLineEdit(self.groupBox_5)
        self.ki_pitch_lineEdit.setGeometry(QtCore.QRect(130, 60, 171, 27))
        self.ki_pitch_lineEdit.setObjectName(_fromUtf8("ki_pitch_lineEdit"))
        self.kd_pitch_lineEdit = QtGui.QLineEdit(self.groupBox_5)
        self.kd_pitch_lineEdit.setGeometry(QtCore.QRect(130, 90, 171, 27))
        self.kd_pitch_lineEdit.setObjectName(_fromUtf8("kd_pitch_lineEdit"))
        self.label_12 = QtGui.QLabel(self.groupBox_5)
        self.label_12.setGeometry(QtCore.QRect(10, 120, 81, 17))
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.ckpoint_pitch_lineEdit = QtGui.QLineEdit(self.groupBox_5)
        self.ckpoint_pitch_lineEdit.setGeometry(QtCore.QRect(130, 120, 171, 27))
        self.ckpoint_pitch_lineEdit.setObjectName(_fromUtf8("ckpoint_pitch_lineEdit"))
        self.setpoint_pitch_lineEdit = QtGui.QLineEdit(self.groupBox_5)
        self.setpoint_pitch_lineEdit.setGeometry(QtCore.QRect(130, 150, 171, 27))
        self.setpoint_pitch_lineEdit.setObjectName(_fromUtf8("setpoint_pitch_lineEdit"))
        self.label_15 = QtGui.QLabel(self.groupBox_5)
        self.label_15.setGeometry(QtCore.QRect(10, 150, 91, 16))
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.groupBox_6 = QtGui.QGroupBox(self.centralWidget)
        self.groupBox_6.setGeometry(QtCore.QRect(80, 480, 451, 181))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_6.setFont(font)
        self.groupBox_6.setObjectName(_fromUtf8("groupBox_6"))
        self.label_8 = QtGui.QLabel(self.groupBox_6)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(self.groupBox_6)
        self.label_9.setGeometry(QtCore.QRect(10, 60, 67, 17))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.label_10 = QtGui.QLabel(self.groupBox_6)
        self.label_10.setGeometry(QtCore.QRect(10, 90, 67, 17))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.kp_depth_lineEdit = QtGui.QLineEdit(self.groupBox_6)
        self.kp_depth_lineEdit.setGeometry(QtCore.QRect(130, 30, 171, 27))
        self.kp_depth_lineEdit.setObjectName(_fromUtf8("kp_depth_lineEdit"))
        self.ki_depth_lineEdit = QtGui.QLineEdit(self.groupBox_6)
        self.ki_depth_lineEdit.setGeometry(QtCore.QRect(130, 60, 171, 27))
        self.ki_depth_lineEdit.setObjectName(_fromUtf8("ki_depth_lineEdit"))
        self.kd_depth_lineEdit = QtGui.QLineEdit(self.groupBox_6)
        self.kd_depth_lineEdit.setGeometry(QtCore.QRect(130, 90, 171, 27))
        self.kd_depth_lineEdit.setObjectName(_fromUtf8("kd_depth_lineEdit"))
        self.label_13 = QtGui.QLabel(self.groupBox_6)
        self.label_13.setGeometry(QtCore.QRect(10, 120, 81, 17))
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.ckpoint_depth_lineEdit = QtGui.QLineEdit(self.groupBox_6)
        self.ckpoint_depth_lineEdit.setGeometry(QtCore.QRect(130, 120, 171, 27))
        self.ckpoint_depth_lineEdit.setObjectName(_fromUtf8("ckpoint_depth_lineEdit"))
        self.setpoint_depth_lineEdit = QtGui.QLineEdit(self.groupBox_6)
        self.setpoint_depth_lineEdit.setGeometry(QtCore.QRect(130, 150, 171, 27))
        self.setpoint_depth_lineEdit.setObjectName(_fromUtf8("setpoint_depth_lineEdit"))
        self.label_16 = QtGui.QLabel(self.groupBox_6)
        self.label_16.setGeometry(QtCore.QRect(10, 150, 91, 16))
        self.label_16.setObjectName(_fromUtf8("label_16"))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 1246, 25))
        self.menuBar.setObjectName(_fromUtf8("menuBar"))
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(MainWindow)
        self.mainToolBar.setObjectName(_fromUtf8("mainToolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName(_fromUtf8("statusBar"))
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "PID NITR AUV PANEL", None))
        self.groupBox.setTitle(_translate("MainWindow", "Yaw Graph", None))
        self.groupBox_2.setTitle(_translate("MainWindow", "Pitch Graph", None))
        self.groupBox_3.setTitle(_translate("MainWindow", "Depth Graph", None))
        self.groupBox_4.setTitle(_translate("MainWindow", "Yaw ", None))
        self.label_2.setText(_translate("MainWindow", "kp:", None))
        self.label_3.setText(_translate("MainWindow", "ki:", None))
        self.label_4.setText(_translate("MainWindow", "kd:", None))
        self.label_11.setText(_translate("MainWindow", "checkpoint:", None))
        self.label_14.setText(_translate("MainWindow", "setpoint:", None))
        self.groupBox_5.setTitle(_translate("MainWindow", "Pitch", None))
        self.label_5.setText(_translate("MainWindow", "kp:", None))
        self.label_6.setText(_translate("MainWindow", "ki:", None))
        self.label_7.setText(_translate("MainWindow", "kd:", None))
        self.label_12.setText(_translate("MainWindow", "checkpoint:", None))
        self.label_15.setText(_translate("MainWindow", "setpoint:", None))
        self.groupBox_6.setTitle(_translate("MainWindow", "Depth", None))
        self.label_8.setText(_translate("MainWindow", "kp:", None))
        self.label_9.setText(_translate("MainWindow", "ki:", None))
        self.label_10.setText(_translate("MainWindow", "kd:", None))
        self.label_13.setText(_translate("MainWindow", "checkpoint:", None))
        self.label_16.setText(_translate("MainWindow", "setpoint:", None))

from pyqtgraph import PlotWidget