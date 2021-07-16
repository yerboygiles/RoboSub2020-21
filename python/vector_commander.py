#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 7/16/21
# Description:
# This node manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
# import random
import math
import serial
import bno055_data
import phidget9dof_data
import gyro_data_merger
import vision_v1
import remote_control

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
        print("Waiting 8 for Arduino...")
        time.sleep(8)
        print("Communicating with Arduino and it's peripherals...")
        self.UsingGyro = usinggyro
        self.serial = serial.Serial('/dev/ttyAMA0', 115200)
        if self.UsingGyro:
            print("Sending IMU")
            self.SendToArduino("IMU")
            self.Gyro_drone1 = bno055_data.BN055(self.serial)
            self.Gyro_queen = phidget9dof_data.Phidget9dof()
            self.Gyro_hive = gyro_data_merger.GyroMerger(self.Gyro_queen, self.Gyro_drone1)
        else:
            print("Sending NOIMU")
            self.SendToArduino("NOIMU")

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
            # import Theos_Really_Good_Detection_Script as obj_det
            # self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            # print("MovementCommander is using Vision AI...")
            self.Vision = vision_v1.vision()
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingSim:
            from advancedtelemetry import Telemetry
            self.TelemetrySim = Telemetry()
            print("MovementCommander is using Telemetry...")
        else:
            print("MovementCommander is not using Telemetry...")
        # thruster hardpoint classes
        # 'ventral' are the central, vertically oriented thrusters
        # for roll/pitch and ascent/descent
        # 'lateral' are the outer, 45 deg. oriented thrusters for
        # yaw/turning and strafe movement
        self.VentralThrusterLB = ThrusterDriver("LB")  # left back
        self.VentralThrusterLF = ThrusterDriver("LF")  # left front
        self.VentralThrusterRB = ThrusterDriver("RB")  # right back
        self.VentralThrusterRF = ThrusterDriver("RF")  # right front
        self.LateralThrusterLB = ThrusterDriver("BL")  # back left
        self.LateralThrusterRB = ThrusterDriver("BR")  # back right
        self.LateralThrusterLF = ThrusterDriver("FL")  # front left !
        self.LateralThrusterRF = ThrusterDriver("FR")  # front right !
        # power values to set to the thruster hardpoints
        # horizontally oriented
        self.LateralPowerLB = 0
        self.LateralPowerLF = 0
        self.LateralPowerRB = 0
        self.LateralPowerRF = 0
        # vertically oriented
        self.VentralPowerLB = 0
        self.VentralPowerRB = 0
        self.VentralPowerRF = 0
        self.VentralPowerLF = 0
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
            "STRAFE LEFT",
            "BACKWARDS",
            "STRAFE RIGHT",
            "TURN LEFT",
            "TURN RIGHT",
            "ASCEND",
            "DESCEND",
            "PAUSE",
        ]
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "GYRO TO",
            "POSITION TO"
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE TO",
            "RAM",
            "FIRE AT",
            "FOLLOW"
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

    def BasicDriverControl(self):
        DrivingWithControl = True
        print("Driver Control!!")
        while DrivingWithControl:
            DriveCommand = remote_control.get_wasdqerv_directional()
            self.BasicDirectionPower(DriveCommand)
            DrivingWithControl = DriveCommand is not -2
            self.TradeWithArduino()

    def BasicWithTime(self):
        DrivingWithTime = True
        while DrivingWithTime:
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < int(self.SuppCommand)
            self.BasicDirectionPower(self.CommandIndex)

    def BasicLinear(self):
        while time.perf_counter() - self.InitialTime < int(self.SuppCommand):
            self.BasicDirectionPower(self.CommandIndex)

    def BasicVectoring(self):
        Vectoring = True
        i = 0
        print("Supplemental: ", self.SuppCommand)
        for SuppParse in str(self.SuppCommand).split(':'):
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

    def TargetMovement(self):
        print("Scanning for target...")
        while self.SearchAndLockTarget(self.SuppCommand):
            pass
        engaging = True
        ramtime = 0
        while engaging:
            self.Vision.StereoTarget(False)
            # 0- "MOVE TO",
            if self.CommandIndex == 0:
                if self.Vision.getDistance() < int(self.SuppCommand):
                    engaging = False
            # 1- "RAM",
            elif self.CommandIndex == 1:
                if self.Vision.getDistance() < int(self.SuppCommand):
                    self.BasicDirectionPower(1)
                    if time.perf_counter() - ramtime > (int(self.SuppCommand) / 4):
                        engaging = False
                else:
                    ramtime = time.perf_counter()
            # 2- "FIRE AT",
            # 3- "FOLLOW"
            self.UpdateThrustersVisionPID(self.Vision.getOffset())

    def SearchAndLockTarget(self, target):
        scanstate = False
        state1_timer = 0
        state2_timer = 0
        confidence_timer = 0
        self.TargetLocked = False
        if not self.Vision.sees(target):
            self.MovingToConfidence = False
            if scanstate:  # pause and look
                state1_timer = time.perf_counter()
                self.BasicDirectionPower(-2)
                if state1_timer - state2_timer > 5:
                    scanstate = False
            else:  # increment and look, 6 = right, 5 = left
                state2_timer = time.perf_counter()
                self.BasicDirectionPower(6)
                if state2_timer - state1_timer > 3:
                    scanstate = True
        else:
            if not self.MovingToConfidence:
                confidence_timer = time.perf_counter()
                self.MovingToConfidence = True
            else:
                if time.perf_counter() - confidence_timer > 5:
                    return False
                self.BasicDirectionPower(-2)
        self.UpdateThrusters()
        return True

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
        self.SendToArduino("STOP")
        print("Stopping arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("START")
        print("Starting arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("MAXPOWER:20")
        print("Sending settings... Wait 3.")
        time.sleep(3)
        try:
            for command in commandlist:
                print("VectorCommander running: ", command)
                self.MainCommand = ""
                self.SuppCommand = ""
                j = 0
                for commandParsed in str(command).split(','):
                    commandParsed.strip()
                    if j == 0:
                        self.MainCommand = commandParsed
                    if j == 1:
                        self.SuppCommand = commandParsed
                    j = j + 1
                print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
                if self.MainCommand == "REMOTE":
                    print("Driver Control With:")
                    self.BasicDriverControl()
                    if self.SuppCommand == "KEYBOARD":
                        print("Keyboard!")
                    else:
                        pass
                else:
                    for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == basiccommand:
                            self.InitialTime = time.perf_counter()
                            if self.UsingGyro:
                                self.BasicLinear(self.SuppCommand)
                            else:
                                self.BasicWithTime(self.SuppCommand)
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
                    for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == advancedcommand:
                            self.InitialTime = time.perf_counter()
                            self.BasicVectoring(self.SuppCommand)
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
                    for targetcommand in self.TARGET_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == targetcommand:
                            self.InitialTime = time.perf_counter()
                            self.TargetMovement(self.SuppCommand)
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
        except:
            self.Terminate()

    def CheckIfGyroDone(self, threshold=15, timethreshold=5):
        # if(self.Gyro.getYaw() < 0):
        self.GyroRunning = True
        integer = 0
        self.UpdateGyro()
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
            outdata += str(self.LateralThrusterLB.name)
            outdata += ":"
            outdata += str(self.LateralThrusterLB.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterRB.name)
            outdata += ":"
            outdata += str(self.LateralThrusterRB.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterLF.name)
            outdata += ":"
            outdata += str(self.LateralThrusterLF.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterRF.name)
            outdata += ":"
            outdata += str(self.LateralThrusterRF.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        if not self.secondSetTrade:
            outdata += str(self.VentralThrusterLB.name)
            outdata += ":"
            outdata += str(self.VentralThrusterLB.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterLF.name)
            outdata += ":"
            outdata += str(self.VentralThrusterLF.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterRB.name)
            outdata += ":"
            outdata += str(self.VentralThrusterRB.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterRF.name)
            outdata += ":"
            outdata += str(self.VentralThrusterRF.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        self.secondSetTrade = not self.secondSetTrade

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

    def BasicDirectionPower(self, index, power=15):
        if index != 0:
            if index == 1:
                print("MOVING FORWARDS")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = power
                self.LateralPowerRF = power
            elif index == 2:
                print("STRAFING LEFT")
                self.LateralPowerLB = power
                self.LateralPowerLF = -power
                self.LateralPowerRB = -power
                self.LateralPowerRF = power
            elif index == 3:
                print("REVERSING")
                self.LateralPowerLB = -power
                self.LateralPowerLF = -power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 4:
                print("STRAFING RIGHT")
                self.LateralPowerLB = -power
                self.LateralPowerLF = power
                self.LateralPowerRB = power
                self.LateralPowerRF = -power
            elif index == 5:
                print("TURNING LEFT")
                self.LateralPowerLB = -power
                self.LateralPowerLF = -power
                self.LateralPowerRB = power
                self.LateralPowerRF = power
            elif index == 6:
                print("TURNING RIGHT")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 7:
                print("ASCENDING")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 8:
                print("DESCENDING")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == -1:
                print("PAUSING")
                self.LateralPowerLB = 0
                self.LateralPowerLF = 0
                self.LateralPowerRB = 0
                self.LateralPowerRF = 0
            elif index == -2:
                print("STOPPING")
                self.LateralPowerLB = 0
                self.LateralPowerLF = 0
                self.LateralPowerRB = 0
                self.LateralPowerRF = 0

    def UpdateThrusters(self):
        self.LateralThrusterLB.SetSpeed(self.LateralPowerLB)
        self.LateralThrusterLF.SetSpeed(self.LateralPowerLF)
        self.LateralThrusterRB.SetSpeed(self.LateralPowerRB)
        self.LateralThrusterRF.SetSpeed(self.LateralPowerRF)

        self.VentralThrusterLB.SetSpeed(self.VentralPowerLB)
        self.VentralThrusterRB.SetSpeed(self.VentralPowerRB)
        self.VentralThrusterLF.SetSpeed(self.VentralPowerLF)
        self.VentralThrusterRF.SetSpeed(self.VentralPowerRF)

    def UpdateThrustersPID(self):
        self.LateralThrusterLB.SetSpeedPID(self.LateralPowerLB, xpid=self.Gyro_hive.getYawPID())
        self.LateralThrusterLF.SetSpeedPID(self.LateralPowerLF, xpid=self.Gyro_hive.getYawPID())
        self.LateralThrusterRB.SetSpeedPID(self.LateralPowerRB, xpid=-self.Gyro_hive.getYawPID())
        self.LateralThrusterRF.SetSpeedPID(self.LateralPowerRF, xpid=-self.Gyro_hive.getYawPID())

        self.VentralThrusterLB.SetSpeedPID(self.VentralPowerLB,
                                           zpid=self.Gyro_hive.getRollPID(),
                                           ypid=-self.Gyro_hive.getPitchPID())
        self.VentralThrusterRB.SetSpeedPID(self.VentralPowerRB,
                                           zpid=-self.Gyro_hive.getRollPID(),
                                           ypid=-self.Gyro_hive.getPitchPID())
        self.VentralThrusterLF.SetSpeedPID(self.VentralPowerLF,
                                           zpid=-self.Gyro_hive.getRollPID(),
                                           ypid=-self.Gyro_hive.getPitchPID())
        self.VentralThrusterRF.SetSpeedPID(self.VentralPowerRF,
                                           zpid=self.Gyro_hive.getRollPID(),
                                           ypid=-self.Gyro_hive.getPitchPID())

    def UpdateThrustersVisionPID(self, targoffset):

        self.LateralThrusterLB.SetSpeedPID(self.LateralPowerLB, xpid=self.Vision.getXPID())
        self.LateralThrusterLF.SetSpeedPID(self.LateralPowerLF, xpid=self.Vision.getXPID())
        self.LateralThrusterRB.SetSpeedPID(self.LateralPowerRB, xpid=-self.Vision.getXPID())
        self.LateralThrusterRF.SetSpeedPID(self.LateralPowerRF, xpid=-self.Vision.getXPID())

        self.VentralThrusterLB.SetSpeedPID(self.VentralPowerLB,
                                           ypid=-self.Vision.getYPID())
        self.VentralThrusterRB.SetSpeedPID(self.VentralPowerRB,
                                           ypid=-self.Vision.getYPID())
        self.VentralThrusterLF.SetSpeedPID(self.VentralPowerLF,
                                           ypid=-self.Vision.getYPID())
        self.VentralThrusterRF.SetSpeedPID(self.VentralPowerRF,
                                           ypid=-self.Vision.getYPID())

    def UpdateGyro(self):
        if self.UsingGyro:
            self.Gyro_hive.UpdateGyro()
            # print(self.Gyro.getGyro())
            self.Gyro_hive.CalculateError(self.YawOffset,
                                          self.PitchOffset,
                                          self.RollOffset,
                                          self.NorthOffset,
                                          self.EastOffset,
                                          self.DownOffset)
            self.Gyro_hive.PID()

    def BrakeAllThrusters(self):
        # horizontal
        self.LateralPowerLB = 0
        self.LateralPowerLF = 0
        self.LateralPowerRB = 0
        self.LateralPowerRF = 0
        # vert
        self.VentralPowerLB = 0
        self.VentralPowerRB = 0
        self.VentralPowerRF = 0
        self.VentralPowerLF = 0

        self.UpdateThrusters()

    # searches for target if cannot find it
    # def SearchForTarget(self, target, repositioning=False, distancethreshold=300):

    # ending vehicle connection and AI processing after mission completion or a major fucky wucky
    def Terminate(self):
        self.VentralThrusterLB.SetSpeed(0)
        self.VentralThrusterLF.SetSpeed(0)
        self.VentralThrusterRB.SetSpeed(0)
        self.VentralThrusterRF.SetSpeed(0)
        self.LateralThrusterLB.SetSpeed(0)
        self.LateralThrusterRB.SetSpeed(0)
        self.LateralThrusterRF.SetSpeed(0)
        self.LateralThrusterLF.SetSpeed(0)
        # self.UpdateThrusters()
        self.SendToArduino("STOP")
        time.sleep(1)
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.Vision.Terminate()
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
    def SetSpeedPID(self, speed, zpid=0.0, ypid=0.0, xpid=0.0):
        self.speed = float(float(speed) + float(zpid) + float(ypid) + float(xpid))
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
