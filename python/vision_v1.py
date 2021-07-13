#!python3
# Author: Theodor Giles
# Created: 7/13/21
# Description:
# node for moving around data from the vision
# processing system

import serial
import time
import re
import math


X: int = 0
Y: int = 1


class vision:
    Error = [0.0, 0.0, 0.0]
    Previous_Error = [0.0, 0.0, 0.0]
    Error_Sum = [0.0, 0.0, 0.0]
    Error_Delta = [0.0, 0.0, 0.0]
    # gyro              position
    XOffset = 0.0
    YOffset = 0.0

    # this is a comment
    Kp = [0.1, 0.1, 0.0]  # constant to modify PID
    Ki = [0.1, 0.1, 0.0]  # constant to modify PID
    Kd = [0.1, 0.1, 0.0]  # constant to modify PID

    X_PID = 0.0
    X_P = 0.0
    X_I = 0.0
    X_D = 0.0

    Y_PID = 0.0
    Y_P = 0.0
    Y_I = 0.0
    Y_D = 0.0

    def __init__(self):
        pass

    def getXOffset(self):
        return self.XOffset

    def getYOffset(self):
        return self.YOffset

    # req for PID calculation
    def CalculateError(self):
        self.Error = [0, 0]
        self.Error_Sum = [0, 0]
        self.Error_Delta = [0, 0]
        self.Previous_Error = [0, 0]
        # previous error for error delta
        # gyro
        self.Previous_Error[X] = self.Error[X]
        self.Previous_Error[Y] = self.Error[Y]
        # error for proportional control
        self.Error[X] = self.XOffset
        self.Error[Y] = self.YOffset

        # sum of error for integral
        self.Error_Sum[X] = self.Error_Sum[X] + self.Error[X]
        self.Error_Sum[Y] = self.Error_Sum[Y] + self.Error[Y]

        # math for change in error to do derivative
        self.Error_Delta[X] = self.Error[X] - self.Previous_Error[X]
        self.Error_Delta[Y] = self.Error[Y] - self.Previous_Error[Y]

    # pid calculation
    def PID(self):
        # X PID variable setting
        self.X_P = (self.Error[X] * self.Kp[X])
        self.X_I = (self.Error_Sum[X] * self.Ki[X])
        self.X_D = (self.Error_Delta[X] * self.Kd[X])
        self.X_PID = self.X_P  # + self.X_I + self.X_D

        # Y PID variable setting
        self.Y_P = (self.Error[Y] * self.Kp[Y])
        self.Y_I = (self.Error_Sum[Y] * self.Ki[Y])
        self.Y_D = (self.Error_Delta[Y] * self.Kd[Y])
        self.Y_PID = self.Y_P  # + self.Y_I + self.Down_D

    def getXPID(self):
        return self.X_PID

    def getYPID(self):
        return self.Y_PID

    def getOffset(self):
        return [self.XOffset, self.YOffset]

    def getDistance(self):
        return self.Distance

    # end command/vehicle running
    def Terminate(self):
        pass
