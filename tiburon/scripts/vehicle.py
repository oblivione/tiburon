#!/usr/bin/env python

class VehicleParams():
    def __init__(self):
        self.B = 16 # Buoyancy force
        self.weight = 16*0.9
        # 1- forward. 2-backward, 3-left, 4-right
        self.x1 = 0.30
        self.x2 = 0.20
        self.x3 = 0.25
        self.x4 = 0.25
        self.xB = 0.05

        self.thrusterMax = 2.72 #in kgf
        self.thrusterMin = 0.45 #in kgf
        self.thrusterMaxForRightThruster = 2.72 * 0.8
        self.thrusterMinForRightThruster = 0.45 * 0.8

        self.thrusterRatioF = (self.thrusterMax-self.thrusterMin)/((1900 - 1520)**2)
        self.thrusterRatioB = self.thrusterRatioF * 0.7
        self.thrusterRatioFRight = self.thrusterRatioF * 0.8
        self.thrusterRatioBRight = self.thrusterRatioB * 0.8
