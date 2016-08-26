#!/usr/bin/env python

class VehicleParams():
    def __init__(self):
        self.B = 0.00 # Buoyancy force
        self.mass = 0.00
        self.gravity = 9.81
        # 1- forward. 2-backward, 3-left, 4-right
        self.x1 = 0.00
        self.x2 = 0.00
        self.x3 = 0.00
        self.x4 = 0.00
        # Assuming thruster force model is F = k w**2
        # t1F is k of first thruster in forward direction
        # self.t1F = 1.00
        # self.t1B = 1.00
        # self.t2F = 1.00
        # self.t2B = 1.00
        # self.t3F = 1.00
        # self.t3B = 1.00
        # self.t4F = 1.00
        # self.t4B = 1.00
