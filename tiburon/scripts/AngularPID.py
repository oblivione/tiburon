#!/usr/bin/env python
import datetime,time
import math
class AngularPID():
    def __init__(self):
        self.Kp = 0.00
        self.Ki = 0.00
        self.Kd = 0.00
        self.presentTime = 0.00
        self.pastTime = 0.00
        self.checkpoint = 0.00
        self.currentVal = 0.00
        self.error = 0.00
        self.prevVal = 0.00
        self.dError = 0.00
        self.IError = 0.00
        self.u = 0.00

    def getError(self):
        self.presentTime = float(time.time())
        if self.pastTime == 0.00:
            # to avoid derivative spike in the first iteration
            self.pastTime = self.presentTime

        self.error = math.atan2(math.sin((self.checkpoint - self.currentVal)/180 * math.pi),math.cos((self.checkpoint - self.currentVal)/180 * math.pi))

        self.timeDiff = self.presentTime - self.pastTime
        self.IError += self.error * self.timeDiff
        if self.timeDiff!=0 :
            self.dError = (0 - math.atan2(math.sin((self.currentVal - self.prevVal)/180 * math.pi),math.cos((self.currentVal - self.prevVal)/180 * math.pi)))/self.timeDiff
        self.pastTime = self.presentTime
        self.prevVal = self.currentVal
        self.u = self.Kp*self.error + self.Ki*self.IError + self.Kd*self.dError
        return self.u
