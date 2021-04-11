#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 11/22/20
# Description:
# This program manages the commands/movement/physical control of the RoboSub V1
#
import time
import random
import math
import serial
import bno055_data
import gyro_data_merger
from threading import Thread

# ROBOSUB
A_TARGET = 1
A_POSITION = 2
A_GYRO = 3

MAX_THROTTLE = 35

GENERAL_THROTTLE = 17.5


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision = False, usinggyro = False, usingsim = False):
        # setting up board serial port
        print("Communicating with Arduino...")
        self.ardserial = serial.Serial('/dev/ttyACM0', 9600)
        self.ardserial.flushInput()
        self.UsingGyro = usinggyro
        if self.UsingGyro:
            self.Gyro = bno055_data.Sensor9Axis(self.ardserial)
            self.GyroMerger = gyro_data_merger.GyroMerger(self.Gyro)
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

        # initialize thruster values to brake (self.PowerXX set to 0^)
        self.UpdateThrusters()

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.
                # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "GYRO TO",
            "POSITION TO"
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE TO TARGET",
            "RAM TARGET",
            "FIRE AT TARGET",
            "FOLLOW TARGET"
        ]
        # currently only for firing torpedoes, maybe a claw action later on?
        self.SUPPLEMENTARY_COMMANDS = [
            "FIRE TORPEDO"
        ]
        # name of object to target sent to TF/openCV AI
        self.TO_TARGET = ""

        # possible targets, matches up with labelmap.txt to make easier
        self.POSSIBLE_TARGETS = [
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate"]
        self.TargetList = []
        print("MovementCommander initialized...")

    def BasicVectoring(self):
        pass
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
        for command in commandlist:
            for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                i = 0
                if command == advancedcommand:
                    self.AdvancedMove()
                i += 1
            self.CommandIndex+=1
    def CheckIfGyroDone(self, threshold=3, timethreshold=5):
        self.PowerBR = -10
        self.PowerBL = -10
        self.PitchOffset = 0
        if (abs(self.Gyro.getYaw() - self.YawOffset) < threshold) and (
                abs(self.Gyro.getPitch() - self.PitchOffset) < threshold) and (
                abs(self.Gyro.getRoll() - self.RollOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within gyro threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.GyroRunning

    def TurnOffArduino(self):
        self.ardserial.write("END\n".encode('ascii'))

    def TurnOnArduino(self):
        self.ardserial.write("START\n".encode('ascii'))

    def SendToArduino(self):
        outdata = ""
        outdata += str(self.ThrusterLB.GetSpeed())
        outdata += str(self.ThrusterLF.GetSpeed())
        outdata += str(self.ThrusterRB.GetSpeed())
        outdata += str(self.ThrusterRF.GetSpeed())
        outdata += str(self.ThrusterBL.GetSpeed())
        outdata += str(self.ThrusterBR.GetSpeed())
        outdata += str(self.ThrusterFL.GetSpeed())
        outdata += str(self.ThrusterFR.GetSpeed())
        outdata += "\n"
        self.ardserial.write(outdata.encode('ascii'))

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        if (abs(self.Gyro.getNorth() - self.NorthOffset) < threshold) and (
                abs(self.Gyro.getEast() - self.EastOffset) < threshold) and (
                abs(self.Gyro.getDown() - self.DownOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

    def UpdateThrusters(self):
        self.Gyro.UpdateGyro()
        self.Gyro.CalculateError(self.YawOffset,
                                    self.PitchOffset,
                                    self.RollOffset,
                                    self.NorthOffset,
                                    self.EastOffset,
                                    self.DownOffset)
        self.Gyro.PID()
        self.ThrusterLB.SetSpeedPID(self.PowerLB, yawpid=self.Gyro.getYawPID())
        self.ThrusterLF.SetSpeedPID(self.PowerLF, yawpid=self.Gyro.getYawPID())
        self.ThrusterRB.SetSpeedPID(self.PowerRB, yawpid=-self.Gyro.getYawPID())
        self.ThrusterRF.SetSpeedPID(self.PowerRF, yawpid=-self.Gyro.getYawPID())

        self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                    rollpid=self.Gyro.getRollPID(),
                                    pitchpid=-self.Gyro.getPitchPID())
        self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                    rollpid=-self.Gyro.getRollPID(),
                                    pitchpid=-self.Gyro.getPitchPID())
        self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                    rollpid=-self.Gyro.getRollPID(),
                                    pitchpid=-self.Gyro.getPitchPID())
        self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                    rollpid=self.Gyro.getRollPID(),
                                    pitchpid=-self.Gyro.getPitchPID())

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

    # self.ADVANCED_MOVEMENT_COMMANDS = [
    #     "LOG START POINT",
    #     "RETURN TO START",
    # ]

    # self.TARGET_MOVEMENT_COMMANDS = [
    #     "MOVE TO TARGET",
    #     "RAM TARGET",
    #     "FIRE AT TARGET"
    # ]

    def VectorMovement(self, designatedcoords):
        #deltaVals = [designatedcoords[dimension] - self.Gyro[dimension] for dimension in range(len(point1))]
        self.Gyro.UpdateGyro()
        self.Gyro.CalculateError(self.YawOffset,
                                    self.PitchOffset,
                                    self.RollOffset,
                                    self.NorthOffset,
                                    self.EastOffset,
                                    self.DownOffset)



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
