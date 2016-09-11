# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controllerCheck.ui'
#
# Created: Sat Sep  3 18:13:10 2016
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
        MainWindow.resize(498, 259)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.depthSlider = QtGui.QSlider(self.centralWidget)
        self.depthSlider.setGeometry(QtCore.QRect(80, 22, 161, 29))
        self.depthSlider.setOrientation(QtCore.Qt.Horizontal)
        self.depthSlider.setObjectName(_fromUtf8("depthSlider"))
        self.pitchSlider = QtGui.QSlider(self.centralWidget)
        self.pitchSlider.setGeometry(QtCore.QRect(80, 82, 160, 29))
        self.pitchSlider.setOrientation(QtCore.Qt.Horizontal)
        self.pitchSlider.setObjectName(_fromUtf8("pitchSlider"))
        self.yawSlider = QtGui.QSlider(self.centralWidget)
        self.yawSlider.setGeometry(QtCore.QRect(80, 140, 161, 31))
        self.yawSlider.setOrientation(QtCore.Qt.Horizontal)
        self.yawSlider.setObjectName(_fromUtf8("yawSlider"))
        self.velSlider = QtGui.QSlider(self.centralWidget)
        self.velSlider.setGeometry(QtCore.QRect(80, 182, 161, 31))
        self.velSlider.setOrientation(QtCore.Qt.Horizontal)
        self.velSlider.setObjectName(_fromUtf8("velSlider"))
        self.depthLabel = QtGui.QLabel(self.centralWidget)
        self.depthLabel.setGeometry(QtCore.QRect(270, 30, 67, 17))
        self.depthLabel.setObjectName(_fromUtf8("depthLabel"))
        self.pitchLabel = QtGui.QLabel(self.centralWidget)
        self.pitchLabel.setGeometry(QtCore.QRect(270, 90, 67, 17))
        self.pitchLabel.setObjectName(_fromUtf8("pitchLabel"))
        self.yawLabel = QtGui.QLabel(self.centralWidget)
        self.yawLabel.setGeometry(QtCore.QRect(270, 140, 67, 17))
        self.yawLabel.setObjectName(_fromUtf8("yawLabel"))
        self.velLabel = QtGui.QLabel(self.centralWidget)
        self.velLabel.setGeometry(QtCore.QRect(270, 182, 67, 17))
        self.velLabel.setObjectName(_fromUtf8("velLabel"))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 498, 25))
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
        MainWindow.setWindowTitle(_translate("MainWindow", "Controller Checker", None))
        self.depthLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.pitchLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.yawLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.velLabel.setText(_translate("MainWindow", "TextLabel", None))
