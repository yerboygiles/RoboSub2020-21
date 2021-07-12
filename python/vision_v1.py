#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 8/12/20
# Description:
# gets data from the simulated/non-simulated pixhawk for attitude, positioning, and maybe some
# other cool tasks it can do

import serial
import time
import re
import math

# sitl is basically a simulation, can be "ran" from any computer kinda? I will figure out a way to make an incorporated
# 3d python sim for managing all this at some point
# sitl = dronekit_sitl.start_default()  # (sitl.start)
# connection_string = sitl.connection_string()
GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2


class vision:
    StringIn = ""
    Error = [0.0, 0.0]
    Previous_Error = [0.0, 0.0]
    Error_Sum = [0.0, 0.0]
    Error_Delta = [0.0, 0.0]
    # gyro              position
    XOffset = 0.0
    YOffset = 0.0

    # this is a comment
    Kp = [0.1, 0.1]  # constant to modify PID
    Ki = [0.1, 0.1]  # constant to modify PID
    Kd = [0.1, 0.1]  # constant to modify PID

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
        self.Previous_Error[0] = self.Error[0]
        self.Previous_Error[1] = self.Error[1]
        # error for proportional control
        self.Error[0] = self.XOffset
        self.Error[1] = self.YOffset

        # sum of error for integral
        self.Error_Sum[0] = self.Error_Sum[0] + self.Error[0]
        self.Error_Sum[1] = self.Error_Sum[1] + self.Error[1]

        # math for change in error to do derivative
        self.Error_Delta[0] = self.Error[0] - self.Previous_Error[0]
        self.Error_Delta[1] = self.Error[1] - self.Previous_Error[1]

    # pid calculation
    def PID(self):
        # X PID variable setting
        self.X_P = (self.Error[0] * self.Kp[0])
        self.X_I = (self.Error_Sum[0] * self.Ki[0])
        self.X_D = (self.Error_Delta[0] * self.Kd[0])
        self.X_PID = self.X_P  # + self.X_I + self.X_D

        # Y PID variable setting
        self.Y_P = (self.Error[1] * self.Kp[1])
        self.Y_I = (self.Error_Sum[1] * self.Ki[1])
        self.Y_D = (self.Error_Delta[1] * self.Kd[1])
        self.Y_PID = self.Y_P  # + self.Y_I + self.Down_D

    def getXPID(self):
        return self.X_PID

    def getYPID(self):
        return self.Y_PID

    # end command/vehicle running
    def Terminate(self):
        pass
