#!/usr/bin/env python

class VehicleParams():
    def __init__(self):
        self.B = 18.65 # Buoyancy force
        self.weight = self.B - 0.9
        # 1- forward. 2-backward, 3-left, 4-right
        self.x1 = 0.264
        self.x2 = 0.236
        self.x3 = 0.25
        self.x4 = 0.25
        self.xB = 0.00

        self.thrusterMax = 2.72 #in kgf
        self.thrusterMin = 0.45 #in kgf

        self.thrusterRatioF = (self.thrusterMax-self.thrusterMin)/((1900 - 1520)**2)
        self.thrusterRatioB = self.thrusterRatioF * 0.7

        self.J = 0.00
