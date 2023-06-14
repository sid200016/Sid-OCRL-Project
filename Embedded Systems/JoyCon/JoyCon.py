import pygame
import time
import numpy as np

class JoyCon:

    def __init__(self):

        self.joysticks = []
        self.JoyStick_JoyConID = None
        self.buttonMapping = {"A": 1, "B": 3, "X": 0, "Y": 2, "SL": 9, "SR": 10, "+": 6, "Stick In": 7,
                                     "Home": 5, "R": 16, "ZR": 18} #For Joy-Con Right

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
                return(buttonValues,[horiz_move,vert_move]) #return the button press values and the

    def rumbleFeedback(self,lowIntensity,highIntensity,duration_ms):
        self.joysticks[self.JoyStick_JoyConID].rumble(lowIntensity,highIntensity,duration_ms)



jc = JoyCon()
jc.rumbleFeedback(0,1,5000)
while(True):
    [buttonVal,AxesPos] = jc.eventLoop()
    time.sleep(0.01)
