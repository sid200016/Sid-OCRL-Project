import pygame
import time
import numpy as np
from EmbeddedSystems.Gantry import GantryController as GC
from EmbeddedSystems.SoftGrasper import SoftGrasper as SG
from EmbeddedSystems.RigidGrasper import RigidGrasper as RG
from enum import Enum
from copy import deepcopy

class Button:

    def __init__(self,buttonNumber:int,buttonName:str, excludeButtons:tuple, priority:int, fcn:None):
        self.buttonNumber = buttonNumber
        self.buttonName = buttonName
        self.exclude = excludeButtons
        self.priority = priority #higher number means higher priority
        self.fcn = fcn




class JoyCon:

    def __init__(self):

        self.joysticks = []
        self.JoyStick_JoyConID = None
        self.buttonMapping = {"A": Button(1,"A",(),1,self.buttonA), #arguments: Button Number, Button Name, Exclusion buttons, priority, button press function
                              "B": Button(3,"B",(),1,self.buttonB),
                              "X": Button(0,"X",(),1,self.buttonX),
                              "Y": Button(2,"Y",(),1,self.buttonY),
                              "SL": Button(9,"SL",(),1,self.buttonSL),
                              "SR": Button(10,"SR",(),1,self.buttonSR),
                              "+": Button(6,"+",(),1,self.buttonPlus),
                              "Stick In": Button(7,"Stick In",(),1,self.buttonJoystickIn),
                              "Home": Button(5,"Home",(),1,self.buttonHome),
                              "R": Button(16,"R",(),1,self.buttonR),
                              "ZR": Button(18,"ZR",(),1,self.buttonZR)} #For Joy-Con Right


        # self.functionMapping = {"A": self.buttonA, "B": self.buttonB, "X": self.buttonX, "Y": self.buttonY,
        #                         "SL": self.buttonSL, "SR": self.buttonSR, "+": self.buttonPlus,
        #                         "Stick In": self.buttonJoystickIn(),"Home": self.buttonHome,
        #                         "R": self.buttonR, "ZR": self.buttonZR}
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

                buttonValues = {k: None for (k, v) in self.buttonMapping.items() if not isinstance(k,tuple)} #only get button presses, not multiple maps.
                for k, v in buttonValues.items():
                    buttonValues[k] = joystick.get_button(self.buttonMapping[k].buttonNumber)

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



    def sortButtonMapping(self):
        self.buttonMapping = dict(sorted(self.buttonMapping.items(),key=lambda x:x[1].priority,reverse = True))

    def ExecuteButtonFunctions(self,buttonValues,AnalogAxis):

        self.sortButtonMapping()  # should be in order from highest priority to lowest priority.

        #single buttons:
        singleButtons = {k: v for (k,v) in self.buttonMapping.items() if not isinstance(k, tuple)}

        #Process tuples first as these indicate simultaneous key presses
        multipleButtons = {k: v for (k, v) in self.buttonMapping.items() if
                           isinstance(k, tuple)}  # only get multiple button presses



        for (k,v) in multipleButtons.items(): #k is a key like "A" or "X", v  is a button class object
            buttonPressBool = np.all([buttonValues[x] for x in v.buttonName]) #Check if all the required buttons have been pressed. v.buttonNumber is a tuple with the buttons corresponding to the multi-button press.
            if buttonPressBool == True: #if the buttons corresponding to the multiple button command were pressed, then execute the function and remove the buttons from the single execution list

                func = [v.fcn for (k2,v) in multipleButtons.items() if frozenset(k2)==frozenset(k)][0] #get the function
                func() #call the function

                #remove  single buttons from the dictionary
                for k3 in v.exclude:
                    del singleButtons[k3] #remove buttons

        for (k, v) in singleButtons.items():
            if buttonValues[k] == 1:
                v.fcn()






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

    def __init__(self,RGa:RG.RigidGrasper):
        super().__init__()
        self.grasper = RGa
        self.grasperIncrement = 100

        self.buttonMapping[("A", "X")] = Button((self.buttonMapping["A"].buttonNumber,self.buttonMapping["X"].buttonNumber),
                                                ("A","X"),
                                                ("A","X"),2,self.buttonAX) #situation where two buttons are pressed at once to close the grasper
        self.buttonMapping[("B", "Y")] = Button((self.buttonMapping["B"].buttonNumber,self.buttonMapping["Y"].buttonNumber),
                                                ("B","Y"),
                                                ("B","Y"),2,self.buttonBY) #situation where two buttons are pressed at once to open the grasper

        self.sortButtonMapping() #should be in order from highest priority to lowest priority.


    def buttonA(self): #close right claw, assume position control
        self.grasper.IncrementalMove(moveIncrement1= self.grasperIncrement, moveIncrement2 = self.grasperIncrement,
                                     action1=RG.GrasperActions.CLOSE,action2=RG.GrasperActions.STAY)

    def buttonB(self): #open right claw, assume position control
        self.grasper.IncrementalMove(moveIncrement1=self.grasperIncrement, moveIncrement2=self.grasperIncrement,
                                     action1=RG.GrasperActions.OPEN, action2=RG.GrasperActions.STAY)

    def buttonX(self):  # close left claw, assume position control
        self.grasper.IncrementalMove(moveIncrement1=self.grasperIncrement, moveIncrement2=self.grasperIncrement,
                                     action1=RG.GrasperActions.STAY, action2=RG.GrasperActions.CLOSE)

    def buttonY(self): #open left claw, assume position control
        self.grasper.IncrementalMove(moveIncrement1=self.grasperIncrement, moveIncrement2=self.grasperIncrement,
                                     action1=RG.GrasperActions.STAY, action2=RG.GrasperActions.OPEN)

    def buttonAX(self):
        self.grasper.IncrementalMove(moveIncrement1= self.grasperIncrement, moveIncrement2 = self.grasperIncrement,
                                     action1=RG.GrasperActions.CLOSE,action2=RG.GrasperActions.CLOSE)


    def buttonBY(self):
        self.grasper.IncrementalMove(moveIncrement1=self.grasperIncrement, moveIncrement2=self.grasperIncrement,
                                 action1=RG.GrasperActions.OPEN, action2=RG.GrasperActions.OPEN)











# jc = JoyCon()
# jc.rumbleFeedback(0,1,5000)
# while(True):
#     [buttonVal,AxesPos] = jc.eventLoop()
#     time.sleep(0.01)
