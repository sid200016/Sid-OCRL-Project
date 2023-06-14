import pygame
import time
import numpy as np
# from .. Gantry import GantryController
# from .. SoftGrasper import SoftGrasper
# from .. RigidGrasper import RigidGrasper

class JoyCon:

    def __init__(self):

        self.joysticks = []
        self.JoyStick_JoyConID = None
        self.buttonMapping = {"A": 1, "B": 3, "X": 0, "Y": 2, "SL": 9, "SR": 10, "+": 6, "Stick In": 7,
                                     "Home": 5, "R": 16, "ZR": 18} #For Joy-Con Right


        self.functionMapping = {"A": self.buttonA, "B": self.buttonB, "X": self.buttonX, "Y": self.buttonY,
                                "SL": self.buttonSL, "SR": self.buttonSR, "+": self.buttonPlus,
                                "Stick In": self.buttonJoystickIn(),"Home": self.buttonHome,
                                "R": self.buttonR, "ZR": self.buttonZR}
        self.initializeJoystick()


    def initializeJoystick(self):
        pygame.init()

        # initialise the joystick module
        pygame.joystick.init()

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                joy = pygame.joystick.Joystick(event.device_index)
                self.joysticks.append(joy)
                if joy.get_name() == "Nintendo Switch Joy-Con (R)":  # only look for Joy-Con (right)
                    self.JoyStick_JoyConID = len(self.joysticks)-1
            # quit program
            if event.type == pygame.QUIT:
                run = False

    def eventLoop(self):

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                run = False

        for joystick in self.joysticks:

            if joystick.get_name() == "Nintendo Switch Joy-Con (R)": #only look for Joy-Con (right)

                buttonValues = {k: None for (k, v) in self.buttonMapping.items()}
                for k, v in self.buttonMapping.items():
                    buttonValues[k] = joystick.get_button(v)

                # player movement with analogue sticks
                vert_move = joystick.get_axis(0)
                horiz_move = joystick.get_axis(1)
                #ax3 = joystick.get_axis(4)
                #ax4 = joystick.get_axis(5)


                print(",".join(buttonValues.keys()))
                print(",".join([str(k) for k in buttonValues.values()]))
                print("Analog Stick: horizontal, vertical")
                print(str(horiz_move) + "," + str(vert_move))
                #print(str(ax3) + "," + str(ax4))

        return(buttonValues,[horiz_move,vert_move]) #return the button press values and the analog stick values

    def rumbleFeedback(self,lowIntensity,highIntensity,duration_ms):
        self.joysticks[self.JoyStick_JoyConID].rumble(lowIntensity,highIntensity,duration_ms)



    #Button Functions
    def buttonA(self):
        pass

    def buttonB(self):
        pass

    def buttonX(self):
        pass

    def buttonY(self):
        pass

    def buttonHome(self):
        pass

    def buttonPlus(self):
        pass

    def buttonJoystickIn(self):
        pass

    def buttonSL(self):
        pass

    def buttonSR(self):
        pass

    def buttonR(self):
        pass

    def buttonZR(self):
        pass


class Joy_RigidGrasper(JoyCon):

    def __init__(self,RG=None):
        super().__init__()
        self.grasper = RG
        self.grasperIncrement = 100


    def buttonA(self): #close right claw, assume position control
        print("A")
        CurrentPosition,dxl_comm_result,dxl_error = self.grasper.ReadCurrentPosition()
        CurrentPosition=[CurrentPosition["1"],CurrentPosition["2"]]
        CurrentPosition[0] = max(CurrentPosition[0]-self.grasperIncrement,self.grasper.GoalPosition["1"][0])
        self.grasper.SetGoalPosition(CurrentPosition[0],CurrentPosition[1])

    def buttonB(self): #open right claw, assume position control
        CurrentPosition,dxl_comm_result,dxl_error = self.grasper.ReadCurrentPosition()
        CurrentPosition = [CurrentPosition["1"], CurrentPosition["2"]]
        CurrentPosition[0] = min(CurrentPosition[0]+self.grasperIncrement,self.grasper.GoalPosition["1"][1])
        self.grasper.SetGoalPosition(CurrentPosition[0],CurrentPosition[1])

    def buttonY(self): #close right claw, assume position control
        CurrentPosition,dxl_comm_result,dxl_error = self.grasper.ReadCurrentPosition()
        CurrentPosition=[CurrentPosition["1"],CurrentPosition["2"]]
        CurrentPosition[1] = max(CurrentPosition[1]-self.grasperIncrement,self.grasper.GoalPosition["2"][1])
        self.grasper.SetGoalPosition(CurrentPosition[0],CurrentPosition[1])

    def buttonX(self): #open right claw, assume position control
        CurrentPosition,dxl_comm_result,dxl_error = self.grasper.ReadCurrentPosition()
        CurrentPosition = [CurrentPosition["1"], CurrentPosition["2"]]
        CurrentPosition[1] = min(CurrentPosition[1]+self.grasperIncrement,self.grasper.GoalPosition["2"][0])
        self.grasper.SetGoalPosition(CurrentPosition[0],CurrentPosition[1])









# jc = JoyCon()
# jc.rumbleFeedback(0,1,5000)
# while(True):
#     [buttonVal,AxesPos] = jc.eventLoop()
#     time.sleep(0.01)
