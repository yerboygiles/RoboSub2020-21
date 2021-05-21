#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 5/14/21
# Description:
# This program manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
# import random
import math
import serial
import bno055_data
import phidget9dof_data
import gyro_data_merger
# from threading import Thread

# ROBOSUB
A_TARGET = 1
A_POSITION = 2
A_GYRO = 3

MAX_THROTTLE = 15

GENERAL_THROTTLE = 17.5


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision=False, usinggyro=False, usingsim=False, resetheadingoncmd=False):
        # setting up board serial port
        print("Communicating with Arduino and it's peripherals...")
        self.UsingGyro = usinggyro
        self.serial = serial.Serial('/dev/ttyAMA0', 115200)
        if self.UsingGyro:
            self.Gyro_drone1 = bno055_data.BN055(self.serial)
            self.Gyro_queen = phidget9dof_data.Phidget9dof()
            self.Gyro_hive = gyro_data_merger.GyroMerger(self.Gyro_queen, self.Gyro_drone1)
        if resetheadingoncmd:
            self.YawOffset = self.Gyro_hive.StartingGyro[0]
            self.ResetHeadingOnCMD = resetheadingoncmd
        self.YawOffset = 0
        self.PitchOffset = 0
        self.RollOffset = 0

        self.EastOffset = 0
        self.NorthOffset = 0
        self.DownOffset = 0

        self.UsingVision = usingvision
        self.UsingSim = usingsim
        if self.UsingVision:
            import Theos_Really_Good_Detection_Script as obj_det
            self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            print("MovementCommander is using Vision AI...")
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingSim:
            from advancedtelemetry import Telemetry
            self.TelemetrySim = Telemetry()
            print("MovementCommander is using Telemetry...")
        else:
            print("MovementCommander is not using Telemetry...")
        # thruster hardpoint classes
        self.ThrusterLB = ThrusterDriver("LB")  # left back
        self.ThrusterLF = ThrusterDriver("LF")  # left front
        self.ThrusterRB = ThrusterDriver("RB")  # right back
        self.ThrusterRF = ThrusterDriver("RF")  # right front
        self.ThrusterBL = ThrusterDriver("BL")  # back left
        self.ThrusterBR = ThrusterDriver("BR")  # back right
        self.ThrusterFL = ThrusterDriver("FL")  # front left
        self.ThrusterFR = ThrusterDriver("FR")  # front right
        # power values to set to the thruster hardpoints
        # horizontally oriented
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vertically oriented
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0
        # boolean for toggling between data sent
        self.secondSetTrade = False
        # initialize thruster values to brake (self.PowerXX set to 0^)
        self.UpdateThrusters()

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.
        # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.BASIC_MOVEMENT_COMMANDS = [
            "FORWARDS",
            "BACKWARDS",
            "LEFT",
            "RIGHT"
        ]
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "GYRO TO",
            "POSITION TO"
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE",
            "RAM",
            "FIRE AT",
            "FOLLOW"
        ]
        # currently only for firing torpedoes, maybe a claw action later on?
        self.SUPPLEMENTARY_COMMANDS = [
            "FIRE TORPEDO AT"
        ]
        # name of object to target sent to TF/openCV AI
        self.TO_TARGET = ""

        # possible targets, matches up with labelmap.txt to make easier
        self.POSSIBLE_TARGETS = [
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate",
            "Cesar"]
        self.TargetList = []
        print("MovementCommander initialized...")

    def BasicWithTime(self, supplemental):
        DrivingWithTime = True
        while DrivingWithTime:
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < supplemental
            self.TradeWithArduino()

    def BasicLinear(self, supplemental):
        pass

    def BasicVectoring(self, supplemental):
        Vectoring = True
        i = 0
        print("Supplemental: ", supplemental)
        for SuppParse in str(supplemental).split(':'):
            print("SuppParse: ", SuppParse)
            if i == 0:
                self.YawOffset = float(SuppParse)
                # print("YawOffset: ", self.YawOffset)
            if i == 1:
                self.PitchOffset = float(SuppParse)
            if i == 2:
                self.RollOffset = float(SuppParse)
            if i > 2:
                break
            i = i + 1
        while Vectoring:
            self.CheckIfGyroDone(threshold=10, timethreshold=3)
            Vectoring = self.GyroRunning
            self.TradeWithArduino()

    def AdvancedVectoring(self):
        pass

    # Concept code, basically for checking if the Sub has already seen the detected object.
    def IsTargetInMemory(self, label, x, y, z):
        NewTarget = [label, x, y, z]
        InMemory = False
        for target in self.TargetList:
            # Determining how far something could be next to the said target,
            DistanceConfidence = math.sqrt(target[4]) * 1.5
            WithinX = abs(NewTarget[1] - target[1]) > DistanceConfidence
            WithinY = abs(NewTarget[2] - target[2]) > DistanceConfidence
            WithinZ = abs(NewTarget[3] - target[3]) > DistanceConfidence
            if (target[0] != NewTarget[0]) and WithinX and WithinY and WithinZ:
                InMemory = True
        return InMemory

    # Concept code, puts target into memory
    def SaveTargetToMemory(self, label, x, y, z, area):
        TargetInfo = [label, x, y, z, area]
        self.TargetList.append(TargetInfo)

    # handles list of commands
    def receiveCommands(self, commandlist):
        # going through commands in parsed list
        self.CommandIndex = 0
        # tell arduino to arm motors
        self.SendToArduino("START")
        print("Starting arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("MAXPOWER:20")
        print("Sending settings... Wait 3.")
        time.sleep(1)
        try:
            for command in commandlist:
                print("VectorCommander running: ", command)
                self.MainCommand = ""
                self.SuppCommand = ""
                j = 0
                for commandParsed in str(command).split(','):
                    if j == 0:
                        self.MainCommand = commandParsed
                    if j == 1:
                        self.SuppCommand = commandParsed
                    j = j + 1
                print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
                for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                    i = 0
                    if self.MainCommand == basiccommand:
                        self.InitialTime = time.perf_counter()
                        if self.UsingGyro:
                            self.BasicLinear(self.SuppCommand)
                        else:
                            self.BasicWithTime(self.SuppCommand)
                    i += 2
                for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                    i = 0
                    if self.MainCommand == advancedcommand:
                        self.InitialTime = time.perf_counter()
                        self.BasicVectoring(self.SuppCommand)
                    i += 2
                self.CommandIndex += 1
        except:
            self.Terminate()

    def CheckIfGyroDone(self, threshold=15, timethreshold=5):
        # if(self.Gyro.getYaw() < 0):
        self.GyroRunning = True
        integer = 0
        if (abs(self.Gyro_hive.getYaw() - abs(self.YawOffset)) < threshold) and (
                abs(self.Gyro_hive.getPitch() - abs(self.PitchOffset)) < threshold) and (
                abs(self.Gyro_hive.getRoll() - abs(self.RollOffset)) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within gyro threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            print("Gyro:", self.Gyro_hive.getGyro())
            self.InitialTime = time.perf_counter()

    def SendToArduino(self, whattosend):
        self.serial.write(whattosend.encode('utf-8'))

    def TradeWithArduino(self):
        self.UpdateThrusters()
        outdata = ""
        if self.secondSetTrade:
            outdata += str(self.ThrusterBL.name)
            outdata += ":"
            outdata += str(self.ThrusterBL.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterBR.name)
            outdata += ":"
            outdata += str(self.ThrusterBR.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterFL.name)
            outdata += ":"
            outdata += str(self.ThrusterFL.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterFR.name)
            outdata += ":"
            outdata += str(self.ThrusterFR.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        if not self.secondSetTrade:
            outdata += str(self.ThrusterLB.name)
            outdata += ":"
            outdata += str(self.ThrusterLB.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterLF.name)
            outdata += ":"
            outdata += str(self.ThrusterLF.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterRB.name)
            outdata += ":"
            outdata += str(self.ThrusterRB.GetSpeed())
            outdata += ","
            outdata += str(self.ThrusterRF.name)
            outdata += ":"
            outdata += str(self.ThrusterRF.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        self.secondSetTrade = not self.secondSetTrade
        self.UpdateGyro()

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        self.PositionRunning = True
        if (abs(self.Gyro_hive.getNorth() - self.NorthOffset) < threshold) and (
                abs(self.Gyro_hive.getEast() - self.EastOffset) < threshold) and (
                abs(self.Gyro_hive.getDown() - self.DownOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

    def CalculatePID(self):
        pass

    def UpdateThrustersB(self):
        self.ThrusterBL.SetSpeedPID(self.PowerLB)
        self.ThrusterFL.SetSpeedPID(self.PowerLF)
        self.ThrusterBR.SetSpeedPID(self.PowerRB)
        self.ThrusterFR.SetSpeedPID(self.PowerRF)

        self.ThrusterLB.SetSpeedPID(self.PowerBL)
        self.ThrusterRB.SetSpeedPID(self.PowerBR)
        self.ThrusterLF.SetSpeedPID(self.PowerFL)
        self.ThrusterRF.SetSpeedPID(self.PowerFR)

    def UpdateThrusters(self):
        self.ThrusterBL.SetSpeedPID(self.PowerLB, yawpid=self.Gyro_hive.getYawPID())
        self.ThrusterFL.SetSpeedPID(self.PowerLF, yawpid=self.Gyro_hive.getYawPID())
        self.ThrusterBR.SetSpeedPID(self.PowerRB, yawpid=-self.Gyro_hive.getYawPID())
        self.ThrusterFR.SetSpeedPID(self.PowerRF, yawpid=-self.Gyro_hive.getYawPID())

        self.ThrusterLB.SetSpeedPID(self.PowerBL,
                                    rollpid=self.Gyro_hive.getRollPID(),
                                    pitchpid=-self.Gyro_hive.getPitchPID())
        self.ThrusterRB.SetSpeedPID(self.PowerBR,
                                    rollpid=-self.Gyro_hive.getRollPID(),
                                    pitchpid=-self.Gyro_hive.getPitchPID())
        self.ThrusterLF.SetSpeedPID(self.PowerFL,
                                    rollpid=-self.Gyro_hive.getRollPID(),
                                    pitchpid=-self.Gyro_hive.getPitchPID())
        self.ThrusterRF.SetSpeedPID(self.PowerFR,
                                    rollpid=self.Gyro_hive.getRollPID(),
                                    pitchpid=-self.Gyro_hive.getPitchPID())

    def UpdateGyro(self):
        self.Gyro_hive.UpdateGyro()
        # print(self.Gyro.getGyro())
        # self.Gyro.UpdatePosition()
        # print(self.Gyro.getPosition())
        self.Gyro_hive.CalculateError(self.YawOffset,
                                      self.PitchOffset,
                                      self.RollOffset,
                                      self.NorthOffset,
                                      self.EastOffset,
                                      self.DownOffset)
        self.Gyro_hive.PID()

    def BrakeAllThrusters(self):
        # horizontal
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vert
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0

        self.UpdateThrusters()

    # searches for target if cannot find it
    # def SearchForTarget(self, target, repositioning=False, distancethreshold=300):

    # ending vehicle connection and AI processing after mission completion or a major fucky wucky
    def Terminate(self):
        self.ThrusterLB.SetSpeed(0)
        self.ThrusterLF.SetSpeed(0)
        self.ThrusterRB.SetSpeed(0)
        self.ThrusterRF.SetSpeed(0)
        self.ThrusterBL.SetSpeed(0)
        self.ThrusterBR.SetSpeed(0)
        self.ThrusterFR.SetSpeed(0)
        self.ThrusterFL.SetSpeed(0)
        # self.UpdateThrusters()
        self.SendToArduino("STOP")
        time.sleep(1)
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.VisionAI.terminate()
        print("Killing board. Wait 1...")
        time.sleep(1)


# dedicated class to driving a specific thruster
# has own PID, thruster, speed
class ThrusterDriver:
    def __init__(self, name):
        self.name = name
        self.speed = 0

    def SetSpeed(self, speed):  # speed is a value between -100 and 100
        if speed > MAX_THROTTLE:
            speed = MAX_THROTTLE
        elif speed < -MAX_THROTTLE:
            speed = -MAX_THROTTLE
        self.speed = MapToPWM(speed)

    #  sets speed of thruster and incorporates the addition of pwm variables
    def SetSpeedPID(self, speed, rollpid=0.0, pitchpid=0.0, yawpid=0.0):
        self.speed = float(float(speed) + float(rollpid) + float(pitchpid) + float(yawpid))
        if self.speed > MAX_THROTTLE:
            self.speed = MAX_THROTTLE
        elif self.speed < -MAX_THROTTLE:
            self.speed = -MAX_THROTTLE
        self.speed = MapToPWM(self.speed)

    # returns speed
    def GetSpeed(self):
        return self.speed


def MapToPWM(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 1100
    out_max = 1900
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
