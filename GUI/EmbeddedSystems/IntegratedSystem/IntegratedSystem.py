import time
import asyncio
import aioconsole
import numpy as np

import logging
from datetime import datetime
import sys

from pathlib import Path

from enum import Enum

from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import PortActions
from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper
from GUI.EmbeddedSystems.Gantry.GantryController import Gantry as GantryController
import GUI.EmbeddedSystems.JoyCon.JoyCon as JC
from GUI.EmbeddedSystems.SNS.SNScontroller import SNScontroller,controller
from GUI.EmbeddedSystems.Support.Structures import GrasperContactForce,Point
from copy import deepcopy

import aioconsole


class GrasperType(Enum):
    SoftGrasper = 0 #Soft Grasper
    RigidGrasper = 1 #Rigid Grasper


### For items for user study experiments ####
class itemStatus(str,Enum):
    notstarted = 'notstarted'
    inprogress = 'inprogress'
    fail = 'fail'
    success = 'success'
    broken = 'broken'

class itemClass(Enum):
    fixed= 'fixed'
    breakable = 'breakable'
    graspable = 'graspable'
    notspecified = 'notspecified'

class itemTrack:
    def __init__(self,objectID, status = itemStatus.notstarted,itemClass=itemClass.graspable):
        self.objectID = objectID
        self.status = status #Not started, inprogress, fail, broken, success
        self.itemClass = itemClass #fixed, breakable, graspable

### End items for user study experiments ####

class IntegratedSystem:
    def __init__(self):
        self.SG = None # Soft Grasper or rigid grasper
        self.GC = None # Gantry Controller
        self.SNSc = None # SNS
        self.jcSG = None #Joycon+Soft Grasper controller

        self.grasperType = None

        #calibration parameters
        self.calibrationParams = {"Calibration Distance (mm)":1, "Grasp Pressure Threshold (psi)":[0.005,0.005,0.005], "Grasp Lift Height (mm)":35}


        #object for user experiments
        # object 1 is top left, object 9 is bottom right.
        self.ObjectVal = {"1": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "2": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "3": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "4": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "5": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "6": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "7": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "8": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
                     "9": itemTrack(1, itemStatus.notstarted, itemClass.notspecified)
                     }

        self.fragileThreshold = 0.4 #scale of 0 to 1.  Forces above this are considered broken for fragile objects
        self.ObjectPressureThreshold = [0.010, 0.010, 0.010]  # threshold above which to trigger the rumble, psi
        self.ObjectPressureScaling = 1  # 100% pressure value for rumble.  In psi. formula is (max change in pressure - pressurethreshold)/PressureScaling

        #logging
        self.logger = None
        self.setupLogger()

        #Events for determining
        self.MoveGrasperEvent = asyncio.Event()
        self.MoveGantryEvent = asyncio.Event()
        self.FreshDataEvent = asyncio.Event()
        self.GrasperReadAverage = {"Average Event":asyncio.Event(),"Number of Loops":20,"Time Delay (s)":0.005}

        #For storing position and pressure variables
        self.jawPressure = None
        self.ClosurePressure = None
        self.curPos = None #will be a Point (x,y,z) in meters

        #For SNS
        self.max_z_height = -0.184
        self.SNS_target_pos_m = [0,-0.25,-0.184]
        self.SNS_object_pos_m = [-0.00931,0.01145,-0.18326] #[0,0,-0.184]
        self.ContactThreshold = {"Pressure Threshold (psi)":[0.05,0.05,0.05], "Pressure Scaling":[100,100,100]}
        self.maxJawChangeInRadius_mm = 15 #20 mm max jaw change in radius
        self.SNS_BypassForceFeedback = True




    def setupLogger(self):
        ##### Set up logging ####
        logger_sys = logging.getLogger(__name__)

        fname = Path(__file__).parents[3].joinpath('datalogs', str(__name__) + datetime.now().strftime(
            "_%d_%m_%Y_%H_%M_%S") + ".txt")

        fh = logging.FileHandler(fname)  # file handler
        fh.setLevel(logging.DEBUG)

        ch = logging.StreamHandler(sys.stdout)  # stream handler
        ch.setLevel(logging.ERROR)

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        logger_sys.setLevel(logging.DEBUG)
        # add the handlers to the logger_soft
        logger_sys.addHandler(fh)
        logger_sys.addHandler(ch)
        self.logger = logger_sys

    async def HardwareInitialize(self, grasper_type = GrasperType.SoftGrasper):

        self.grasperType = grasper_type

        self.GC = GantryController(comport="COM4")#,homeSystem = False, initPos=[0,0,0])#, homeSystem = False,initPos=[0,0,0]  #initialize gantry controller

        if self.grasperType == GrasperType.SoftGrasper:
            self.SG = SoftGrasper(COM_Port='COM7', BaudRate=460800, timeout=1,
                             controllerProfile="New")  # initialize soft grasper

            self.jcSG = JC.Joy_SoftGrasper(SGa=self.SG,
                                      GantryS=self.GC)  # initialize joystick control of soft grasper and gantry controller

        else:  # rigid grasper
            pass

        self.SNSc = SNScontroller(ModulateSNS=False)
        #controller._inter_layer_1._params["tau"].data[2] = 0.75
        #self.logger.info("SNS z height time constant set to %f" % 0.75)


    async def Read_Move_Hardware(self):
        self.SG.ReadGrasperData() #To fix: need to do this at least once so that there is data on the serial line, otherwise "ReadSensorValues" will fail
        await asyncio.sleep(0.5)

        while True:
            if self.jcSG.ControlMode != JC.JoyConState.NORMAL: #only do constant update of position when not in joystick control mode
                curPos = self.GC.getPosition()
                await asyncio.sleep(0.001)

                if self.GrasperReadAverage["Average Event"].is_set() == True: #for the case where you want to average
                    jawPressure, ClosurePressure = await self.SG.ReadSensorValues(number_avg=self.GrasperReadAverage["Number of Loops"],
                                                                                  loop_delay=self.GrasperReadAverage["Time Delay (s)"])
                    self.GrasperReadAverage["Average Event"].clear()

                else: #default
                    jawPressure, ClosurePressure = await self.SG.ReadSensorValues(number_avg=1, loop_delay=0)


                self.SG.ChangeInPressure = jawPressure
                self.jawPressure = jawPressure
                self.ClosurePressure = ClosurePressure
                self.curPos = curPos
                await asyncio.sleep(0.001)  # nominal 50 Hz

                # set the flag indicating that fresh data has been received, then sleep, then change the flag
                self.FreshDataEvent.set()
                await asyncio.sleep(0.010)

            #Move gantry only when event is set to true
            if self.MoveGantryEvent.is_set():
                self.GC.setXYZ_Position(*self.GC.goalPos, self.GC.feedrate_mmps * 60)  # absolute move of the gantry
                self.MoveGantryEvent.clear() #reset the event

            #Actuate grasper only when event is set to true
            if self.MoveGrasperEvent.is_set():
                self.SG.MoveGrasper()
                self.MoveGrasperEvent.clear() #reset the event


            # Set the rumble
            rumbleValue = self.calculateRumble(self.ObjectPressureThreshold, self.ObjectPressureScaling)
            self.jcSG.rumbleFeedback(rumbleValue, rumbleValue, 1000)


            # change the flags
            self.FreshDataEvent.clear()
            await asyncio.sleep(0.001)
            # now that all the actions have been awaited, set the fresh data flag to false


    async def Normal_Mode(self): #meant to run as a long running co-routine
        while (True):
            #print ("Normal mode")
            [buttonVal, AxesPos] = self.jcSG.eventLoop()  # run event loop to determine what values and axes to execute
            self.jcSG.ExecuteButtonFunctions(buttonVal, AxesPos)  # execute Button functions defined by the button values and axes positions

            if self.jcSG.ControlMode == JC.JoyConState.NORMAL:
                posInc, feedrate_mmps = self.jcSG.calcPositionIncrement(AxesPos[0], AxesPos[
                    1])  # get the joystick position for x, y and z, corrected for the flip on the x axis
                self.GC.calculateIncrementalMove(*posInc)  # will set the GC.goalPos with the appropriate increments

                self.MoveGantryEvent.set() #Move the gantry

                self.MoveGrasperEvent.set() #Move the grasper



            #Update the hardware measurements of grasper position and soft grasper state, respecting the lock
            # TODO: to be populated



            await asyncio.sleep(0.001)  # allow other tasks to run



    async def SNS_Mode(self): #meant to run as a long running co-routine
        # TODO: to be populated
        # Finish SNS
        # Add quick close grasper step
        # Fix logging
        # Take video of it closing and record the change in radius 3 times
        #
        while (True):
            if self.jcSG.ControlMode == JC.JoyConState.USE_SNS:
                self.logger.info('Inside SNS control')

                await self.FreshDataEvent.wait()  # wait for fresh data

                curPos = self.curPos
                curPos_orig = curPos
                jawPressure = self.jawPressure

                if self.SNSc.first_attempt == True:
                    # z_offset = curPos.z
                    z_offset = 0
                    # will adjust height relative to object position

                    self.logger.info("Z offset %f" % z_offset)
                    object_position_list = [0, 0, self.SNS_object_pos_m[
                        2] - z_offset]  # [0, 0, -0.185] #calculate move relative to the object position.
                    # [-0.15, -0.15, -0.190]

                    target_position_list = [self.SNS_target_pos_m[0] - self.SNS_object_pos_m[0],
                                            self.SNS_target_pos_m[1] - self.SNS_object_pos_m[1],
                                            self.SNS_target_pos_m[2] - z_offset]

                    self.SNSc.first_attempt = False


                curPos = Point(-self.SNS_object_pos_m[0] + curPos.x, -self.SNS_object_pos_m[1] + curPos.y,
                               -z_offset + curPos.z)  # get where the current position is relative to where the object is

                grasperPosition = Point(curPos.x, curPos.y, curPos.z)  # convert to meters
                self.logger.info('Target position in m: %f %f %f' % (*target_position_list,))
                self.logger.info('GrasperPos in m:%f %f %f' % (*grasperPosition,))
                self.logger.info('ObjectPos in m:%f %f %f' % (*object_position_list,))

                grasperThreshold = self.ContactThreshold["Pressure Threshold (psi)"]
                pressureScaling = self.ContactThreshold["Pressure Scaling"]

                grasperContact = [(x - grasperThreshold[i]) * pressureScaling[i] if x >= grasperThreshold[i] else 0 for (i, x) in
                                  enumerate(jawPressure)]

                grasperContact = GrasperContactForce(*grasperContact)

                object_grasped_phase = (self.SNSc.neuronset['grasp'] >= 20 or self.SNSc.neuronset['lift_after_grasp'] >= 20
                        or self.SNSc.neuronset['move_to_pre_release'] >= 20 or self.SNSc.neuronset['move_to_release'] >= 20) #these should all only trigger once. Only move to pre grasp, move to grasp, grasp are triggered twice, once in the beginning and once when it returns home.



                if (self.SNS_BypassForceFeedback == True and object_grasped_phase == True): #need to fix this so it releases the object as well
                    # for grasping set to 20 psi. For releasing, use real pressure
                    grasperContact = GrasperContactForce(*[0, 0, 0]) if self.SG.commandedPosition[
                                                                            "ClosureChangeInRadius_mm"] < self.maxJawChangeInRadius_mm else GrasperContactForce(
                        *[20, 20, 20])  # set contact threshold based on the position #maybe need to change this to see some change in pressure at the jaws before lifting up, or adding some delay time during inflation. Need to do the same during deflation

                #when commanded position is sufficient and you just transition to lift after grasp, then sleep for 5 seconds in order to allow grasper time to pressurize.
                

                commandPosition_m, JawRadialPos_m = self.SNSc.SNS_forward(grasperPos_m=grasperPosition,
                                                                     grasperContact=grasperContact,
                                                                     objectPos_m=Point(*object_position_list),
                                                                     targetPos_m=Point(*target_position_list),
                                                                     useRealGantry=False) #update SNS


                # if (self.SNS_BypassForceFeedback == True and object_grasped_phase == True):
                #     JawRadialPos_m = self.maxJawChangeInRadius_mm/1000
                #     if self.SNSc.num_grasp_attempts==1 and prev_num_grasp_attempts ==0: #only do this wait one time to allow the grasper to close fully before moving
                #         await asyncio.sleep(5)


                print(self.SNSc.neuronset)
                self.logger.info('Jaw radial pos in m:%f' % (JawRadialPos_m))
                self.logger.info('Command Position: %f %f %f' % (*list(commandPosition_m),))
                # command position is absolute move in m relative to the offset.
                commandPosition_m = Point(self.SNS_object_pos_m[0] + commandPosition_m.x,
                                          self.SNS_object_pos_m[1] + commandPosition_m.y,
                                          z_offset + commandPosition_m.z)  # calculate the move relative to the object position

                self.GC.goalPos = [x * 1000 for x in list(commandPosition_m)] #set goal position

                self.logger.info('SNS commanded goal position (mm):%f,%f,%f' % (*self.GC.goalPos,))
                self.logger.debug('SNS commanded change in radius (mm):%f' % (JawRadialPos_m * 1000))
                self.logger.debug(','.join([k + ":" + str(v) for k, v in self.SNSc.neuronset.items()]))

                self.SG.commandedPosition["ClosureChangeInRadius_mm"] = min(JawRadialPos_m * 1000,
                                                                       self.maxJawChangeInRadius_mm)  # limit the radial position change to prevent overinflation


                self.logger.info('Number of grasp attempts %i' % self.SNSc.num_grasp_attempts)

                motion_complete = self.SNSc.lift_after_release_done == True  and self.SNSc.neuronset["grasp"]>=20

                if self.SNSc.num_grasp_attempts >= 1 or self.SNSc.lift_after_release_done == True:

                    # if (SNSc.num_grasp_attempts>=1
                    #         and
                    #         (SNSc.neuronset["move_to_grasp"]>=20 or SNSc.neuronset["move_to_pre_grasp"]>=20)): #if about to attempt a regrasp
                    #     GC.goalPos = [curPos_orig.x*1000, curPos_orig.y*1000, curPos_orig.z*1000+70] #move object up
                    #     loggerR.info('Failed grasp, exceeded number of attempts')
                    #     SNSc.lift_after_grasp_done = True #set to true to trigger the next statement
                    #     SG.commandedPosition["ClosureChangeInRadius_mm"] = 0
                    #

                    if self.SNSc.num_grasp_attempts >= 1 and motion_complete == True:
                        self.jcSG.SNS_control = False  # reset to false to give control back to the user
                        self.jcSG.ControlMode = JC.JoyConState.NORMAL
                        self.logger.info('Reset the SNS controller after motion complete complete')
                        self.SG.commandedPosition["ClosureChangeInRadius_mm"] = 0 #so it doesn't re-pressurize





                self.MoveGrasperEvent.set()
                self.MoveGantryEvent.set()







            else:
                self.SNSc = SNScontroller() #reinitialize SNS




            await asyncio.sleep(0.002)






    async def Calibration(self):
        while True:
            if self.jcSG.ControlMode == JC.JoyConState.CALIBRATION:
                self.GrasperReadAverage["Average Event"].set()
                print("Calibration started. Hit SL+ to stop calibration")

                #get initial position and state of grasper

                await self.FreshDataEvent.wait() #wait for fresh data


                print ("Gantry position at start of calibration (mm): %f %f %f"%tuple([x*1000 for x in self.curPos]))
                print ("Grasper closure muscle pressure (psi): %f"%self.ClosurePressure) #need function to go from pressure to mm
                print ("Grasper contact pressure (psi) at start of calibration: %f, %f, %f" % tuple(self.jawPressure))

                #close grasper
                self.SG.IncrementalMove(closureIncrement_mm=self.calibrationParams["Calibration Distance (mm)"],
                                        jawIncrement_psi=[0, 0, 0])  # setup the variables

                self.MoveGrasperEvent.set()
                self.MoveGantryEvent.set()

                #allow time to settle
                await asyncio.sleep(5) #sleep for 5 seconds.  Allow other tasks to run

                #get contact pressure prior to lift
                await self.FreshDataEvent.wait()  # wait for fresh data
                print("Contact Pressure prior to lift: %f %f %f"%tuple(self.jawPressure))


                #lift grasper
                self.GC.calculateIncrementalMove(0,0,self.calibrationParams["Grasp Lift Height (mm)"]) #move up by 20 mm
                self.MoveGantryEvent.set()
                await asyncio.sleep(15)


                #check contact
                await self.FreshDataEvent.wait()  # wait for fresh data
                grasperContact = np.all([x if x >= self.calibrationParams["Grasp Pressure Threshold (psi)"][i] else 0 for (i, x) in
                                  enumerate(self.jawPressure)]) == True #sufficient contact if any of the thresholds greater than the threshold

                print("Contact Pressure: %f %f %f" % tuple(self.jawPressure))

                #if object not picked up, relax grasper by 1 mm, return to home then contract by 1mm to set up next loop
                if grasperContact == False:
                    #open grasper slightly
                    self.SG.IncrementalMove(closureIncrement_mm=-2,
                                            jawIncrement_psi=[0, 0, 0])  # setup the variables
                    self.MoveGrasperEvent.set()
                    await asyncio.sleep(5)

                    #lower grasper
                    self.GC.calculateIncrementalMove(0, 0, -self.calibrationParams["Grasp Lift Height (mm)"])
                    self.MoveGantryEvent.set()
                    await asyncio.sleep(15)
                    print("After Sleep")


                    #close grasper slightly
                    self.SG.IncrementalMove(closureIncrement_mm=2,
                                            jawIncrement_psi=[0, 0, 0])  # setup the variables
                    self.MoveGrasperEvent.set()
                    await asyncio.sleep(5)

                #if object picked up, return to position, release object, set ControlMode to normal and report the final values.
                elif grasperContact == True:
                    print("Calibration successful")
                    print("Gantry position at end of calibration (mm): %f %f %f" % tuple([x * 1000 for x in self.curPos]))
                    print(
                        "Grasper closure muscle pressure at end of calibration (psi): %f" % self.ClosurePressure)  # need function to go from pressure to mm
                    print("Grasper closure radius at end of calibration (mm): %f" % self.SG.commandedPosition[
                        "ClosureChangeInRadius_mm"])
                    print("Grasper contact pressure (psi) at end of calibration: %f, %f, %f" % tuple(self.jawPressure))

                    #return to home
                    print("Returning object to base ...")
                    self.GC.calculateIncrementalMove(0, 0, -self.calibrationParams["Grasp Lift Height (mm)"])
                    self.MoveGantryEvent.set()
                    await asyncio.sleep(15)

                    #open grasper
                    self.SG.commandedPosition["ClosureChangeInRadius_mm"] = 0
                    self.MoveGrasperEvent.set()

                    await asyncio.sleep(5)

                    #set variable
                    self.jcSG.ControlMode = JC.JoyConState.NORMAL #return to normal mode

                    self.GrasperReadAverage["Average Event"].clear() #go back to only reading values 1 time

                await asyncio.sleep(0.05)  # allow other tasks to run
            #print("End of Calibration loop")
            #print(self.jcSG.ControlMode)
            await asyncio.sleep(0.001)  # allow other tasks to run
        await asyncio.sleep(0.001)

    def calculateRumble(self,pressureThreshold=[0.2, 0.2, 0.2], pressureScaling=1):

        # pressureThreshold:  change in pressure threshold in psi above which to register changes in pressure
        rumbleValue_arr = [min((x - pressureThreshold[i]) / pressureScaling, 1) if x >= pressureThreshold[i] else 0 for
                           (i, x) in
                           enumerate(self.SG.changeInPressure)]  # was divide by 1.75, changed to 1

        rumbleValue = max(rumbleValue_arr)
        # if the current attempt is on a breakable item, check to see if the rumble value is greater than the threshold

        idx = [k for k, x in self.ObjectVal.items() if x.status == itemStatus.inprogress]  # find the key which in progress

        try:
            if len(idx) > 0:
                idx = idx[0]
                if self.ObjectVal[idx].itemClass == itemClass.breakable and rumbleValue > self.fragileThreshold:
                    rumbleValue = 0
                    self.logger.warning('Force exceeded on item %s.  Threshold %f, rumble value %f' %
                                    (idx, fragileThreshold, rumbleValue))


        except Exception as e:
            self.logger.error(e)
            self.logger.error('Error during fragile object check')
            rumbleValue = 0

        return (rumbleValue)

    async def ReadCommandLine(self):
        print_string = "Enter C for calibration. \n" \
                       "Enter S for SNS. \n" \
                       "Enter Z to return to joystick control\n" \
                       "Enter H to move grasper home \n"
        print(print_string)
        while (True):
            s = await aioconsole.ainput()
            print(s)


            match s.upper():

                # Calibration
                case "C":
                    self.jcSG.ControlMode = JC.JoyConState.CALIBRATION

                # SNS
                case "S":
                    self.jcSG.ControlMode = JC.JoyConState.PREP_SNS
                    await self.Get_SNS_Input() #prompts to setup SNS
                    self.jcSG.ControlMode = JC.JoyConState.USE_SNS

                # Return to Joystick Mode
                case "Z":
                    self.jcSG.ControlMode = JC.JoyConState.NORMAL

                case "H":
                    await(self.returnHome())


                # Default
                case _:
                    print(print_string)



            await asyncio.sleep(0.01)

    async def returnHome(self, defaultMode = None):
        priorMode = self.jcSG.ControlMode
        self.jcSG.ControlMode = JC.JoyConState.GO_HOME
        self.GC.goalPos = [0,0,0]
        self.MoveGantryEvent.set()
        await asyncio.sleep(20) #sleep for 20 seconds while returning home
        if defaultMode is None:
            self.jcSG.ControlMode = priorMode
        else:
            self.jcSG.ControlMode = defaultMode

        print ("Return home completed")




    async def Get_SNS_Input(self):
        useXYZcalibration = await aioconsole.ainput(
            "Do you want to use the current position as the object location?\n"
            "Enter 'Y' for yes, 'N' to use default position, 'T' to adjust the calibration xyz value, or 'X' to enter a new value. \n")

        match useXYZcalibration.upper():

            case "Y":
                await self.FreshDataEvent.wait()
                self.SNS_object_pos_m =[self.curPos.x,self.curPos.y,self.curPos.z]



            case "N":
                print("Using default values")

            case "T":
                vals = await aioconsole.ainput(
                    "Expecting 3 values for x,y,z offset separated by comma and in mm \n")
                vals = [float(x) for x in vals.split(',')]
                self.SNS_object_pos_m[0] = self.SNS_object_pos_m[0] + vals[0]/1000
                self.SNS_object_pos_m[1] = self.SNS_object_pos_m[1] + vals[1]/1000
                self.SNS_object_pos_m[2] = self.SNS_object_pos_m[2] + vals[2]/1000

            case "X":
                vals = await aioconsole.ainput(
                    "Expecting 3 values for new x,y,z separated by comma and in mm \n")
                vals = [float(x) for x in vals.split(',')]
                self.SNS_object_pos_m[0] = vals[0] / 1000
                self.SNS_object_pos_m[1] = vals[1] / 1000
                self.SNS_object_pos_m[2] = vals[2] / 1000

            case "Z":
                "Exiting SNS setup ... \n"
                return

            case _:
                pass


        print("x,y,z used is (mm): %f, %f, %f" % (
            self.SNS_object_pos_m[0]*1000, self.SNS_object_pos_m[1]*1000, self.SNS_object_pos_m[2]*1000))

        # Ask it they want to adjust or enter new value for the grasper max closure calibration
        useGrasperCalibration = await aioconsole.ainput(
            "Do you want to use the grasper maximum change in radial position from the current state?\n "
            "Enter 'Y' for yes, 'N' to use default position, 'T' to adjust the calibration grasper value, or 'X' to enter a new value.\n")

        match useGrasperCalibration.upper():

            case "Y":
                await self.FreshDataEvent.wait()  # wait for fresh data
                self.maxJawChangeInRadius_mm = self.SG.commandedPosition["ClosureChangeInRadius_mm"]


            case "N":
                print("Using default values")

            case "T":
                vals = await aioconsole.ainput(
                    "Expecting value in mm, this will be an offset")
                self.maxJawChangeInRadius_mm = self.maxJawChangeInRadius_mm + float(vals)

            case "X":
                vals = await aioconsole.ainput(
                    "Expecting value in mm. This is the new value for maximum change in radial position.")
                self.maxJawChangeInRadius_mm = float(vals)

            case "Z":
                "Exiting SNS setup ... \n"
                return

            case _:
                pass

        print("Max closure (mm): %f"%self.maxJawChangeInRadius_mm)

        returnHome = await aioconsole.ainput(
            "Enter any button to return home")

        await self.returnHome()

        print("Beginning SNS ...")



if __name__ == '__main__':

    async def run_Program():
        IS = IntegratedSystem()
        await IS.HardwareInitialize()

        print("Finished Calibration")



        # https://stackoverflow.com/questions/53465862/python-aiohttp-into-existing-event-loop
        L = await asyncio.gather(
            IS.Normal_Mode(),
            IS.Calibration(),
            IS.SNS_Mode(),
            IS.ReadCommandLine(),
            IS.Read_Move_Hardware()
        )
        #runp = asyncio.create_task(HandleProgram())
        #await runp

        print('Before forever sleep')
        while True:
            await asyncio.sleep(1)



    try:
        #web.run_app(app)
        asyncio.run(run_Program())
    except KeyboardInterrupt:
        "Exiting Program.  Thank you."
