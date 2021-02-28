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
from threading import Thread

# ROBOSUB

MAX_THROTTLE = 35

GENERAL_THROTTLE = 17.5


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision, usingpixhawk, usingsim):
        # setting up board serial port
        print("Communicating with Arduino...")
        self.ardserial = serial.Serial('/dev/ttyACM0', 9600)
        self.ardserial.flushInput()

        self.YawOffset = 0
        self.PitchOffset = 0
        self.RollOffset = 0

        self.EastOffset = 0
        self.NorthOffset = 0
        self.DownOffset = 0

        self.UsingVision = usingvision
        self.UsingPixHawk = usingpixhawk
        self.UsingSim = usingsim
        if self.UsingVision:
            import Theos_Really_Good_Detection_Script as obj_det
            self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            print("MovementCommander is using Vision AI...")
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingPixHawk:
            from pixhawk_data import PixHawk
            self.PixHawk = PixHawk()
            print("MovementCommander is using PixHawk...")
        else:
            print("MovementCommander is not using PixHawk...")
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
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFL.SetSpeed(self.PowerFL)
        self.ThrusterFR.SetSpeed(self.PowerFR)

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.

        # basic: these commands are just normal up, down, turn, etc.
        self.BASIC_MOVEMENT_COMMANDS = [
            "FORWARD",
            "REVERSE",
            "LEFT",
            "RIGHT",
            "CLOCKWISE TURN",
            "COUNTERCLOCKWISE TURN",
            "DIVE",
            "SURFACE",
            "IDLE"]

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

    # handles list of commands
    def receiveCommands(self, commandlist):
        # going through commands in parsed list
        self.CommandIndex = 0
        for command in commandlist:
            for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                i = 0
                if command == basiccommand:
                    self.BasicCommand(GENERAL_THROTTLE,i)
                i += 1

            for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                i = 0
                if command == advancedcommand:
                    self.AdvancedCommand(GENERAL_THROTTLE,i)
                i += 1
            self.CommandIndex+=1
    def rotateTo(self, wantedyaw, wantedpitch, wantedroll):

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
        if (abs(self.PixHawk.getNorth() - self.NorthOffset) < threshold) and (
                abs(self.PixHawk.getEast() - self.EastOffset) < threshold) and (
                abs(self.PixHawk.getDown() - self.DownOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

    def UpdateThrustersPID(self):
        self.PixHawk.UpdateGyro()
        self.PixHawk.CalculateError(self.YawOffset,
                                    self.PitchOffset,
                                    self.RollOffset,
                                    self.NorthOffset,
                                    self.EastOffset,
                                    self.DownOffset)
        self.PixHawk.PID()
        self.ThrusterLB.SetSpeedPID(self.PowerLB, yawpid=self.PixHawk.getYawPID())
        self.ThrusterLF.SetSpeedPID(self.PowerLF, yawpid=self.PixHawk.getYawPID())
        self.ThrusterRB.SetSpeedPID(self.PowerRB, yawpid=-self.PixHawk.getYawPID())
        self.ThrusterRF.SetSpeedPID(self.PowerRF, yawpid=-self.PixHawk.getYawPID())

        self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                    rollpid=self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                    rollpid=-self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                    rollpid=-self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                    rollpid=self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())

    def UpdateThrusters(self):
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFR.SetSpeed(self.PowerFR)
        self.ThrusterFL.SetSpeed(self.PowerFL)

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

    # self.BASIC_MOVEMENT_COMMANDS = [
    #     "FORWARD",
    #     "REVERSE",
    #     "LEFT",
    #     "RIGHT",
    #     "CLOCKWISE TURN",
    #     "COUNTERCLOCKWISE TURN",
    #     "DIVE",
    #     "SURFACE",
    #     "IDLE"]

    def BasicCommand(self, speed=GENERAL_THROTTLE, commandnum):
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError(self.YawOffset,
                                        self.PitchOffset,
                                        self.RollOffset,
                                        self.NorthOffset,
                                        self.EastOffset,
                                        self.DownOffset)
            self.PixHawk.PID()
        DownConst = -5.0
        # 0 = FORWARD
        if commandnum == 0:
            # horizontal
            self.PowerLB = speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 1 = REVERSE
        if commandnum == 1:
            # horizontal
            self.PowerLB = -speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = -speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 2 = LEFT
        if commandnum == 2:
            # horizontal
            self.PowerLB = speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 3 = RIGHT
        if commandnum == 3:
            # horizontal
            self.PowerLB = -speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = -speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 4 = CLOCKWISE
        if commandnum == 4:
            self.PowerLB = speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = speed

            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 5 = COUNTERCLOCKWISE
        if commandnum == 5:
            self.PowerLB = -speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = -speed
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 6 = DIVE
        if commandnum == 6:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = speed
            self.PowerBR = speed
            self.PowerFR = -speed
            self.PowerFL = -speed

        # 7 = SURFACE
        if commandnum == 7:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -speed
            self.PowerBR = -speed
            self.PowerFR = speed
            self.PowerFL = speed

        # 8 = IDLE
        if commandnum == 8:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -DownConst
            self.PowerBR = -DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst
        if self.UsingPixHawk:
            self.ThrusterLB.SetSpeedPID(self.PowerLB)
            self.ThrusterLF.SetSpeedPID(self.PowerLF)
            self.ThrusterRB.SetSpeedPID(self.PowerRB)
            self.ThrusterRF.SetSpeedPID(self.PowerRF)

            self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
        else:
            self.UpdateThrusters()

    # self.ADVANCED_MOVEMENT_COMMANDS = [
    #     "LOG START POINT",
    #     "RETURN TO START",
    # ]

    def AdvancedCommand(self, commandnum, supplementarycmd=None):
        if self.UsingPixHawk:
            pass
        if commandnum == 0:
            pass
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError(self.YawOffset,
                                        self.PitchOffset,
                                        self.RollOffset,
                                        self.NorthOffset,
                                        self.EastOffset,
                                        self.DownOffset)
            self.PixHawk.PID()
            if commandnum == 5:
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
        else:
            print("Command requires the PixHawk")
            self.Running = False

    # self.TARGET_MOVEMENT_COMMANDS = [
    #     "MOVE TO TARGET",
    #     "RAM TARGET",
    #     "FIRE AT TARGET"
    # ]

    def TargetCommand(self, commandnum, target):
        self.Running = True
        targettaskdone = self.SearchForTarget(target)
        targettaskdone = not targettaskdone
        if targettaskdone:
            print("Target found. Wait 1...")
            time.sleep(1)
        else:
            print("No target found. Wait 1...")
            time.sleep(1)
        while targettaskdone:
            self.VisionAI.process_image(target)
            self.PitchOffset = self.PixHawk.getPitch()
            self.YawOffset = self.PixHawk.getYaw()
            self.PitchOffset += self.VisionAI.getObjectVector()[0]
            self.YawOffset += self.VisionAI.getObjectVector()[1]
            self.DistanceOffset = self.VisionAI.getLatDistanceMM()
            speed = self.DistanceOffset / 100
            if commandnum == 0:
                if self.DistanceOffset < 100:
                    targettaskdone = True
            elif commandnum == 1:
                if self.DistanceOffset < 100:
                    speed = 80
                elif self.DistanceOffset < 30:
                    targettaskdone = True
                elif (self.DistanceOffset < 30) or not FoundTarget:
                    time.sleep(3)
                    targettaskdone = True
            elif commandnum == 2:
                if self.DistanceOffset < 200:
                    # firer torperdor
                    pass
            if speed >= 0:
                self.PowerLB = speed + 15
                self.PowerLF = speed + 15
                self.PowerRB = speed + 15
                self.PowerRF = speed + 15
            else:
                self.PowerLB = 0
                self.PowerLF = 0
                self.PowerRB = 0
                self.PowerRF = 0

            # self.TargetVector

    # searches for target if cannot find it
    def SearchForTarget(self, target, repositioning=False, distancethreshold=300):
        FoundTarget = False
        if repositioning:
            for i in range(5):
                self.EastOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.NorthOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.DownOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.PositionRunning = True
                while self.PositionRunning:
                    self.CheckIfPositionDone(threshold=6, timethreshold=3)
                    self.UpdateThrustersPID()
                    self.SendToArduino()
                for j in range(7):
                    self.YawOffset = j * 55 - 165
                    for k in range(5):
                        self.PitchOffset = k * 35 - 70
                        self.GyroRunning = True
                        print("Scanning at rotation matrix: ", "Yaw: ", self.YawOffset, ", Pitch: ", self.PitchOffset)
                        while self.GyroRunning:
                            LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
                            FoundTarget = self.VisionAI.process_image(target)
                            self.CheckIfGyroDone(threshold=5, timethreshold=2)
                            self.UpdateThrustersPID()
                            self.SendToArduino()
                        if FoundTarget:
                            break
                    if FoundTarget:
                        break
                if FoundTarget:
                    break
        else:
            FoundTarget = False
            for i in range(7):
                self.YawOffset = i * 55 - 165
                for j in range(5):
                    self.PitchOffset = j * 35 - 70
                    self.GyroRunning = True
                    print("Scanning at rotation matrix: ")
                    while self.GyroRunning and not FoundTarget:
                        LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
                        FoundTarget = self.VisionAI.process_image(target)
                        self.CheckIfGyroDone(threshold=5, timethreshold=2)
                        self.UpdateThrustersPID()
                        self.SendToArduino()
                    if FoundTarget:
                        break
                if FoundTarget:
                    break
        return FoundTarget

    # ending vehicle connection and AI processing after mission completion/fatal error
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
        if self.UsingPixHawk:
            print("Killing Pixhawk. Wait 1...")
            time.sleep(1)
            self.PixHawk.Terminate()
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.VisionAI.terminate()
        print("Killing board. Wait 1...")
        time.sleep(1)


def MapToPWM(x) -> float:
    in_min = -100.0
    in_max = 100.0
    out_min = 1100
    out_max = 1900
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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
