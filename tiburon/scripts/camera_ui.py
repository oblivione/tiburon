# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created: Thu May 12 19:48:18 2016
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
        MainWindow.resize(692, 667)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.videoFrame1 = QtGui.QLabel(self.centralwidget)
        self.videoFrame1.setGeometry(QtCore.QRect(20, 20, 320, 240))
        self.videoFrame1.setFrameShape(QtGui.QFrame.WinPanel)
        self.videoFrame1.setObjectName(_fromUtf8("videoFrame1"))
        self.capButton1 = QtGui.QPushButton(self.centralwidget)
        self.capButton1.setGeometry(QtCore.QRect(80, 270, 91, 41))
        self.capButton1.setObjectName(_fromUtf8("capButton1"))
        self.recButton1 = QtGui.QPushButton(self.centralwidget)
        self.recButton1.setGeometry(QtCore.QRect(180, 270, 91, 41))
        self.recButton1.setObjectName(_fromUtf8("recButton1"))
        self.videoFrame2 = QtGui.QLabel(self.centralwidget)
        self.videoFrame2.setGeometry(QtCore.QRect(350, 20, 320, 240))
        self.videoFrame2.setFrameShape(QtGui.QFrame.WinPanel)
        self.videoFrame2.setObjectName(_fromUtf8("videoFrame2"))
        self.capButton2 = QtGui.QPushButton(self.centralwidget)
        self.capButton2.setGeometry(QtCore.QRect(410, 260, 91, 41))
        self.capButton2.setObjectName(_fromUtf8("capButton2"))
        self.recButton2 = QtGui.QPushButton(self.centralwidget)
        self.recButton2.setGeometry(QtCore.QRect(510, 260, 91, 41))
        self.recButton2.setObjectName(_fromUtf8("recButton2"))
        self.videoFrame3 = QtGui.QLabel(self.centralwidget)
        self.videoFrame3.setGeometry(QtCore.QRect(20, 320, 320, 240))
        self.videoFrame3.setFrameShape(QtGui.QFrame.WinPanel)
        self.videoFrame3.setObjectName(_fromUtf8("videoFrame3"))
        self.recButton3 = QtGui.QPushButton(self.centralwidget)
        self.recButton3.setGeometry(QtCore.QRect(180, 570, 91, 41))
        self.recButton3.setObjectName(_fromUtf8("recButton3"))
        self.capButton3 = QtGui.QPushButton(self.centralwidget)
        self.capButton3.setGeometry(QtCore.QRect(80, 570, 91, 41))
        self.capButton3.setObjectName(_fromUtf8("capButton3"))
        self.cam1 = QtGui.QCheckBox(self.centralwidget)
        self.cam1.setGeometry(QtCore.QRect(470, 400, 97, 22))
        self.cam1.setObjectName(_fromUtf8("cam1"))
        self.cam2 = QtGui.QCheckBox(self.centralwidget)
        self.cam2.setGeometry(QtCore.QRect(470, 430, 97, 22))
        self.cam2.setObjectName(_fromUtf8("cam2"))
        self.cam3 = QtGui.QCheckBox(self.centralwidget)
        self.cam3.setGeometry(QtCore.QRect(470, 460, 97, 22))
        self.cam3.setObjectName(_fromUtf8("cam3"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 692, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "AUV Video Stream", None))
        self.videoFrame1.setText(_translate("MainWindow", "TextLabel", None))
        self.capButton1.setText(_translate("MainWindow", "Capture", None))
        self.recButton1.setText(_translate("MainWindow", "Record", None))
        self.videoFrame2.setText(_translate("MainWindow", "TextLabel", None))
        self.capButton2.setText(_translate("MainWindow", "Capture", None))
        self.recButton2.setText(_translate("MainWindow", "Record", None))
        self.videoFrame3.setText(_translate("MainWindow", "TextLabel", None))
        self.recButton3.setText(_translate("MainWindow", "Record", None))
        self.capButton3.setText(_translate("MainWindow", "Capture", None))
        self.cam1.setText(_translate("MainWindow", "Camera 1", None))
        self.cam2.setText(_translate("MainWindow", "Camera 2", None))
        self.cam3.setText(_translate("MainWindow", "Camera 3", None))

