
import pathlib
import pandas as pd
import numpy as np
import time
#import torch
#import torch.nn as nn
#from EmbeddedSystems.Gantry.envs.GantrySimulation import GantrySimulation
# from EmbeddedSystems.Gantry.controller.SNS_layer import SNS_layer, SENSORY_LAYER_1_INPUT_SIZE, SENSORY_LAYER_1_SIZE, \
#     SENSORY_LAYER_2_INPUT_SIZE, SENSORY_LAYER_2_SIZE, THETA_MAX, THETA_MIN, F_MAX, F_MIN, sensory_layer_1, \
#     sensory_layer_2, R, perceptor, controller
import serial
import re
from datetime import datetime
from enum import Enum
from EmbeddedSystems.Support.Structures import Point,Velocity, Acceleration


class GantryActions(Enum):
    STAY = 0 #dont move
    MOVE = 1 #Move jaws closer together

class Gantry:

    def __init__(self,comport='COM12',serialRate=115200,timeout=2,initPos=Point(220,220,200),MaxBufferSize = 3,MoveSpeed_mm_p_min = 50*60, homeSystem = True):
        self.ser = None
        self.initPos = initPos #position in [x,y,z] in mm. x is movement of the gantry head, y is the movement of the base plate and z is the movement up and down.

        self.maxPos_mm = Point(450, 480, 300) #max position from the endstops in mm for x,y and z, respectively
        self.BufferLength = 0
        self.MaxBufferSize = MaxBufferSize #maximum number of commands to keep in the buffer
        self.MoveSpeed = MoveSpeed_mm_p_min #mm/min
        self.MaxSpeedRate_mmps = Velocity(300, 300 , 35) #mm/s.  Max feedrate for the x, y and z axes.
        self.MaxAcceleration_mmps2 = Acceleration(500, 500, 100) #mm/s^2.  Max acceleration for the x, y and z axes
        self.PositionArray = {"time":[0],"x":[0],"y":[0],"z":[0]} #indicates current position
        self.goalPos = Point(0, 0, 0) #triad indicating goal position
        self.inMotion = False #set to true if the gantry is executing a linear move, false otherwise
        self.atGoal = False #set to true if the gantry head is at the goal, false otherwise
        self.readyToInject = True #set to true if it is ready to take another command, false otherwise. it is ready to take a command when it is at goal and not in motion, or in motion and >75% of the distance from the starting pos to the goal position there.
        self.GoalTolerance_mm = 0.5 #distance away from the goal in mm to be considered "at goal"
        self.startMotionPos = Point(0, 0, 0) #triad indicating where the current motion was started from

        #Run Initialization routine:
        self.SetupGantry(comport = comport, serialRate = serialRate,timeout=timeout,initPos = initPos, homeSystem=homeSystem)

    def SetupGantry(self,comport='COM12',serialRate=115200,timeout=2,initPos=Point(220,220,315), homeSystem = True):
        self.ser = serial.Serial(comport, serialRate, timeout=timeout)
        time.sleep(20)

        if homeSystem == True:
            self.HomeGantry(initPos)

    def HomeGantry(self,initPos:Point):
        self.sendCommand("G28 X0 Y0 Z0\r\n")
        # sendCommand(ser,"G0 F15000 X0\r\n")
        self.sendCommand("M400\r\n")
        time.sleep(2)

        self.sendCommand("G90\r\n")
        print("Finished Sending G90")
        time.sleep(1)
        self.sendCommand("G0 F15000\r\n")
        time.sleep(1)
        self.sendCommand("G0 F5000 " + 'X{0} Y{1} Z{2}'.format(*initPos) + "\r\n")  # move to offset position
        self.sendCommand("M400\r\n")  # waits until all motions in the planning queue are completed
        time.sleep(1)
        self.sendCommand("M280 P0 S95\r\n")  # make sure that the gripper is closed
        self.sendCommand("M400\r\n")


    def sendCommand(self,commandstr,wait_for_ok = True):
        self.ser.write(commandstr.encode('utf-8'))
        # readval = ser.read(20)

        # print(readval)
        line_hold=""
        while wait_for_ok:
            line = self.ser.readline()
            print(line)
            line_hold = line_hold + str(line, 'utf-8') +"\n"

            if line == b'ok\n':
                break

        return(line_hold)

    def getPosition(self):
        try:
            tic = time.perf_counter()
            responseM114 = self.sendCommand("M114 B\r\n")
            M114_restr = "X:(?P<Xcom>\d*\.\d*).*Y:(?P<Ycom>\d*\.\d*).*Z:(?P<Zcom>\d*\.\d*).*E:(?P<Ecom>\d*\.\d*).*B:(?P<BufferCountHead>\d*)\s*(?P<BufferCountTail>\d*).*Count.*X:(?P<Xsteps>\d*).*Y:(?P<Ysteps>\d*).*Z:(?P<Zsteps>\d*)"
            matchM114 = re.search(M114_restr, responseM114)
            matchM114_dict = matchM114.groupdict()
            Xcom, Ycom, Zcom, Ecom = float(matchM114_dict['Xcom']), float(matchM114_dict['Ycom']), float(
                matchM114_dict['Zcom']), float(
                matchM114_dict['Ecom'])  # not real time positons, but what was commanded last
            Xsteps, Ysteps, Zsteps = int(matchM114_dict['Xsteps']), int(matchM114_dict['Ysteps']), int(
                matchM114_dict['Zsteps'])  # position in steps
            BufferCountHead = int(matchM114_dict['BufferCountHead'])  # buffer size is 16 (0 indexed, 0:15)
            BufferCountTail = int(matchM114_dict['BufferCountTail'])
            self.BufferLength = (
                BufferCountHead - BufferCountTail if BufferCountHead >= BufferCountTail else BufferCountHead - BufferCountTail + 16) #update the buffer length

            x_off = self.initPos[0]
            y_off = self.initPos[1]
            z_off = self.initPos[2]

            x, y, z = (Xsteps / 80 - x_off) / 1000, (Ysteps / 80 - y_off) / 1000, (
                    Zsteps / 400 - z_off) / 1000  # convert from steps to m

            #update the time and the positions
            positionArray =[x,y,z]
            self.PositionArray["time"].append(datetime.now())
            self.PositionArray["x"].append(x*1000) #multiply by 1000 to convert from m to mm
            self.PositionArray["y"].append(y*1000)
            self.PositionArray["z"].append(z*1000)

            #Check if still in motion
            commandedPos = Point(Xcom*1000,Ycom*1000,Zcom*1000)
            inMotionBool = self.inMotion(currentPosition=Point(*positionArray),commandedPosition = commandedPos)

            #Check if at goal
            atGoalBool = self.atGoalCheck(currentPosition= Point(*positionArray), goalPosition = self.goalPos)

            #Update Inject status based on size of the buffer
            injectBool = self.updateInjectStatus()

            toc = time.perf_counter()
            print("M114 Time " + str(toc - tic))

            return(Point(*positionArray))

        except Exception as e:
            print(responseM114)
            return([np.Inf,np.Inf,np.Inf]) #return Inf for all three axes

    def CheckPositionToReference(self, position1: Point, position2: Point, tolerance: float):  # true if points are within tolerance of each other , false if outside tolerance
        distanceval = np.linalg.norm([np.abs(x - position2[i])  for (i, x) in
                      enumerate(position1)])
        truthValue = distanceval <= tolerance
        return (truthValue,distanceval)

    def InMotionCheck(self,currentPosition: Point, commandedPosition: Point): #true if still in motion, false if not moving
        MotionBool, distanceval = not(self.CheckPositionToReference(currentPosition,commandedPosition,self.GoalTolerance_mm)) #negate result because if still in motion, it is outside the commanded
        self.inMotion = MotionBool
        return(MotionBool)

        # truthArray = [np.abs(x-commandedPosition[i])<=self.GoalTolerance_mm for (i,x) in enumerate(currentPosition)]
        # if np.all(truthArray) == True:
        #     self.inMotion = False
        #     return (False)
        #
        # else:
        #     self.inMotion = True
        #     return (True)


    def atGoalCheck(self, currentPosition: Point, goalPosition: Point): #true if at goal within tolerance, false otherwise.  2nd argument returned is fraction of distance covered
        GoalBool, distanceval = (self.CheckPositionToReference(currentPosition, goalPosition, self.GoalTolerance_mm))

        distanceGoalToStart = np.linalg.norm([x-self.startMotionPos[i] for (i,x) in enumerate(goalPosition)]) #get distance from goal to start

        fractionCovered = distanceval/distanceGoalToStart


        self.atGoal = GoalBool
        return (GoalBool,fractionCovered)

    def updateInjectStatus(self):#fraction distance covered is fraction of the starting distance covered.  #threshold is the percentage covered
        if self.BufferLength < self.MaxBufferSize:
            self.readyToInject = True
        else:
            self.readyToInject = False

        return(self.readyToInject)


    def setXYZ_Position(self,x_mm,y_mm,z_mm,MoveSpeed=None):

        x_off = self.initPos[0]
        y_off = self.initPos[1]
        z_off = self.initPos[2]

        if MoveSpeed is None:
            MoveSpeed = self.MoveSpeed

        posvec = [MoveSpeed, x_mm + x_off, y_mm + y_off,
                  z_mm + z_off]  # first element is the speed in mm/min

        commandStr = "G0 F{0:.2f} X{1:.2f} Y{2:.2f} Z{3:.2f} \r\n".format(*posvec)
        if self.BufferLength < self.MaxBufferSize:  # only send commands when the buffer is small
            # print("Buffer "+str(BufferLength))
            # print(commandStr)

            self.sendCommand(commandStr)





    def incrementalMove(self,move_x_mm = 0, move_y_mm = 0, move_z_mm = 0, moveSpeed_mmps =None): #x, y and z are increments to move, not absolute position

        curPos = self.getPosition() #get current position
        posVec = [move_x_mm, move_y_mm, move_z_mm]

        if self.readyToInject == True & np.any([x !=0 for x in posVec]): #only update startMotionPos if you are are ready to inject and one or more of the commanded axes increments is non zero
            self.startMotionPos = curPos # note the starting position of the move so that later we can check how far into the move we are.
            self.goalPos = [max(min(x+self.initPos[i]+self.goalPos[i], self.maxPos_mm[i]), 0)-self.initPos[i] for (i, x) in enumerate(posVec)] #Update the goal position. limit the move to between 0 and the maximum, but first need to convert incremental move to absolute move, and then afterwards subtract the offset


        #newPos = [max(min(x+self.initPos[i]+curPos[i], self.maxPos_mm[i]), 0)-self.initPos[i] for (i, x) in enumerate(posVec)] #limit the move to between 0 and the maximum, but first need to convert incremental move to absolute move, and then afterwards subtract the offset


        self.setXYZ_Position(*self.goalPos, moveSpeed_mmps*60) #send commands to move to the new position




    def pick_and_place(log_name=""):
        gS = GantrySimulation()  # gantryURDFfile = "URDF//GrasperAndGantry//urdf//GrasperAndGantry.urdf"
        # add object to the simulation at the center of the plate
        gS.addObjectsToSim("PickupCube", startPos=[0, 0, (0.063 + 0.02)], mass_kg=1, sizeScaling=0.6,
                           sourceFile=str(
                               pathlib.Path.cwd() / "Gantry\\envs\\URDF\\PickUpObject_URDF\\urdf\\PickUpObject_URDF.urdf"))

        positionset = []
        targetpositionset = []
        forceset = []
        neuronset = []
        pressureset = []


        GUI_control = True

        x, y, z, left_angle, right_angle = 0, 0, 0, 0, 0  # initialize_to_zero
        x_off, y_off, z_off, left_angle_off, right_angle_off = 220, 220, 315, 0, 0  # offsets for each of the axes
        force = 0

        maxFeedRate = 150 * 60  # mm/min
        periodT = 25  # seconds.  Done to violate maximum speed of 150 mm/sec
        accel = 300.48  # mm/s for x axis
        d_total = 400  # mm

        ts = 0  # time step of the simulation in seconds

        logArray = []

        BeginSNS_T = 0
        delT = 0
        JawRadialPos=0


        while (not gS.CheckStopSim()):  # check to see if the button was pressed to close the sim
            startt_loop = time.time()

            GUIcontrolTarget = gS.bulletClient.readUserDebugParameter(
                gS.GUIcontrols["GUIcontrolId"])
            if GUIcontrolTarget % 2 == 0 and GUI_control is True:
                GUI_control = False
                gS.simCounter = 0
                object_position_array= [0, 0, -0.305]
                object_position = torch.Tensor(object_position_array).unsqueeze(dim=0)
                target_position_array = [0.15, 0.15, -0.310]
                target_position = torch.Tensor(target_position_array).unsqueeze(dim=0)


                ## Setup Logging
                BeginSNS_T = time.time()

                # for log file append
                logArray.append("Current Date: " + time.strftime("%a %d %b %Y %H:%M:%S", time.localtime()))
                logArray.append("Object Position (x,y,z)(m): " + ",".join([str(x) for x in object_position_array]))
                logArray.append("Target Position (x,y,z)(m): " + ",".join([str(x) for x in target_position_array]))

                HeaderStr = ",".join(
                    ["Time", "x (m)", "y (m)", "z (m)", "JawRadialPos (m)", "force", "Buffer Length", "Move to Pre Grasp",
                     "Move to grasp"
                        , "grasp", "lift after grasp", "move to pre release", "move to release", "release",
                     "lift after release","P0","P1","P2","P3","P4","P5","P6","P7","P8","P9","P10","P11"])
                logArray.append(HeaderStr)

            ## get current x,y and z positions via M114 command
            try:
                tic = time.perf_counter()
                responseM114 = sendCommand(ser, "M114 B\r\n")
                M114_restr = "X:(?P<Xcom>\d*\.\d*).*Y:(?P<Ycom>\d*\.\d*).*Z:(?P<Zcom>\d*\.\d*).*E:(?P<Ecom>\d*\.\d*).*B:(?P<BufferCountHead>\d*)\s*(?P<BufferCountTail>\d*).*Count.*X:(?P<Xsteps>\d*).*Y:(?P<Ysteps>\d*).*Z:(?P<Zsteps>\d*)"
                matchM114 = re.search(M114_restr, responseM114)
                matchM114_dict = matchM114.groupdict()
                Xcom, Ycom, Zcom, Ecom = float(matchM114_dict['Xcom']), float(matchM114_dict['Ycom']), float(
                    matchM114_dict['Zcom']), float(
                    matchM114_dict['Ecom'])  # not real time positons, but what was commanded last
                Xsteps, Ysteps, Zsteps = int(matchM114_dict['Xsteps']), int(matchM114_dict['Ysteps']), int(
                    matchM114_dict['Zsteps'])  # position in steps
                BufferCountHead = int(matchM114_dict['BufferCountHead'])  # buffer size is 16 (0 indexed, 0:15)
                BufferCountTail = int(matchM114_dict['BufferCountTail'])
                BufferLength = (
                    BufferCountHead - BufferCountTail if BufferCountHead >= BufferCountTail else BufferCountHead - BufferCountTail + 16)
                x, y, z = (Xsteps / 80 - x_off) / 1000, (Ysteps / 80 - y_off) / 1000, (
                            Zsteps / 400 - z_off) / 1000  # convert from steps to mm
                toc = time.perf_counter()
                print("M114 Time " + str(toc - tic))
            except Exception as e:
                print(responseM114)

            gripper_position = torch.Tensor([x, y, z]).unsqueeze(dim=0)
            #force = torch.Tensor([force]).unsqueeze(dim=0)
            force_vec=[0 ,0 ,0]
            if GUI_control is False:
                tic = time.perf_counter()

                ## Control Grasper
                PVal = SG.GetPressureFromPosition(JawRadialPos * 1000)
                print("Command Pressure Value: "+str(PVal))
                SG.SendPressureCommand(PVal)
                SG.ReadPressureVals()
                ChP = SG.getJawChangePressureVals()
                print("Contact Pressure Change (psi): "+str(ChP))
                force_vec=[x*5 if x>0.2 else 0 for x in ChP]
                force_1 = force_vec[0]
                force_2 = force_vec[1]
                force_3 = force_vec[2]

                print("force_1:", str(force_vec[0]))
                print("force_2:", str(force_vec[1]))
                print("force_3:", str(force_vec[2]))
                force = torch.Tensor(force_vec).unsqueeze(dim=0)


                commands = perceptor.forward(
                    gripper_position, object_position, target_position, force)
                [move_to_pre_grasp, move_to_grasp, grasp, lift_after_grasp, move_to_pre_release,
                 move_to_release, release, lift_after_release] = commands.squeeze(dim=0).numpy()
                [x_d, y_d, z_d, JawRadialPos] = controller.forward(
                    object_position, target_position, commands).numpy()
                if lift_after_release > 10:
                    object_position = torch.Tensor([0, 0, 0]).unsqueeze(dim=0)

                toc = time.perf_counter()
                print("SNS Time " + str(toc - tic))

                ## send commands

                tic = time.perf_counter()
                posvec = [50 * 60, x_d * 1000 + x_off, y_d * 1000 + y_off,
                          z_d * 1000 + z_off]  # first element is the speed in mm/min

                commandStr = "G0 F{0:.2f} X{1:.2f} Y{2:.2f} Z{3:.2f} \r\n".format(*posvec)
                if BufferLength < 10:  # only send commands when the buffer is small
                    # print("Buffer "+str(BufferLength))
                    # print(commandStr)
                    sendCommand(ser, commandStr)
                toc = time.perf_counter()
                print("G0 movement " + str(toc - tic))





                ## get time
                delT = time.time() - BeginSNS_T
                graspAngle=0
                try:
                    PressVals=[x[-1] for x in SG.PressureArray]
                except Exception as e:
                    PressVals =[0 for x in range(0,12)]
                logStr = ",".join([str(x) for x in
                                   [delT, x, y, z, JawRadialPos, force_vec, BufferLength, move_to_pre_grasp, move_to_grasp, grasp,
                                    lift_after_grasp, move_to_pre_release,
                                    move_to_release, release, lift_after_release, PressVals]])
                logArray.append(logStr)

                ## Update arrays of positions, neuron states and froces
                positionset.append([x, y, z])
                targetpositionset.append([x_d, y_d, z_d])
                forceset.append([force_1, force_2, force_3])
                neuronset.append(
                    [move_to_pre_grasp, move_to_grasp, grasp, lift_after_grasp, move_to_pre_release, move_to_release,
                     release, lift_after_release])
                #pressureset.append(PressVals)

            else:
                [x_d, y_d, z_d, JawRadialPos] = [0, 0, 0, 0]

            endt_loop = time.time()
            print('LoopTime: ' + str(startt_loop - endt_loop))  # print the time it takes in the loop


        if log_name == "":
            log_name = str(pathlib.PurePath.joinpath(pathlib.Path.cwd(), "Data",
                                                     time.strftime("%d_%b_%Y_%H_%M_%S", time.localtime()) + ".txt"))
        with open(log_name, "w") as logFile:
            logFile.write("\n".join(logArray))
        return positionset, targetpositionset, forceset, neuronset


