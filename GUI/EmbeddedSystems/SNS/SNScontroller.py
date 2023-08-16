import pathlib
import pandas as pd
import numpy as np
import time
import torch

from .controller.SNS_layer import perceptor, controller

from ..Support.Structures import Point,Velocity,Acceleration,GrasperContactForce
import logging
from pathlib import Path
from datetime import datetime
import sys

class SNScontroller:

    def __init__(self,objectPos_m = Point(0,0,0), targetPos_m = Point(0,0,0)):
        self.neuronset = {
            "move_to_pre_grasp":0,
            "move_to_grasp":0,
            "grasp":0,
            "lift_after_grasp":0,
            "move_to_pre_release":0,
            "move_to_release":0,
            "release":0,
            "lift_after_release":0
        }

        #self.logger = None
        #self.setupLogger()

        self.first_attempt = True

        self.lift_after_release_done = False #set to true when lifted after release neuron exceeds 10.  Necessary to set the object pos to 0,0,0 to trigger the final phases of motion
        self.lift_after_grasp_done = False

        self.object_position = torch.Tensor(list(objectPos_m)).unsqueeze(dim=0)
        self.target_position = torch.Tensor(list(targetPos_m)).unsqueeze(dim=0)

        self.cmd_position_m = Point(0,0,0)
        self.JawRadialPos_m = 0
        self.z_offset = 0

        self.num_grasp_attempts = 0


    def setupLogger(self):
        ##### Set up logging ####
        logger_soft = logging.getLogger(__name__)

        fname = Path(__file__).parents[3].joinpath('datalogs', str(__name__) + datetime.now().strftime(
            "_%d_%m_%Y_%H_%M_%S") + ".txt")

        fh = logging.FileHandler(fname)  # file handler
        fh.setLevel(logging.DEBUG)

        ch = logging.StreamHandler(sys.stdout)  # stream handler
        ch.setLevel(logging.INFO)

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        logger_soft.setLevel(logging.DEBUG)
        # add the handlers to the logger_soft
        logger_soft.addHandler(fh)
        logger_soft.addHandler(ch)
        self.logger = logger_soft

    def SNS_forward(self,grasperPos_m:Point, grasperContact:GrasperContactForce,objectPos_m:Point = None,
                    targetPos_m:Point = None, useRealGantry = False):
        """
        Class to forward SNS

        Parameters
        ----------
        grasperPos_m    :   Point representing the grasper position in x,y,z space.
                            This is in meters, and is referenced with center of the gantry plate

        grasperContact  :   GrasperContactForce representing the force on jaws 1, 2 and 3.

        objectPos_m    :    Point (optional)
                            Defaults to None. If a point is supplied, updates state variable for object position.

        targetPos_m  :      Point (optional)
                            Defaults to None. If a point is supplied, updates state variable for target position.

        """

        if objectPos_m is not None and self.lift_after_release_done == False:
            self.object_position = torch.Tensor(list(objectPos_m)).unsqueeze(dim=0) #update the object pos


        if self.neuronset["lift_after_release"]>10:
            self.lift_after_release_done = True

        if self.lift_after_release_done == True:
            self.object_position = torch.Tensor([0, 0, 0]).unsqueeze(dim=0)


        if targetPos_m is not None:
            self.target_position = torch.Tensor(list(targetPos_m)).unsqueeze(dim=0) #update the target pos


        force = torch.Tensor(list(grasperContact)).unsqueeze(dim=0)
        grasperPos_m = Point(grasperPos_m.x,grasperPos_m.y,grasperPos_m.z-self.z_offset)
        gripper_position = torch.Tensor(list(grasperPos_m)).unsqueeze(dim=0)

        commands = perceptor.forward(
            gripper_position, self.object_position, self.target_position, force)

        [move_to_pre_grasp, move_to_grasp, grasp, lift_after_grasp, move_to_pre_release,
         move_to_release, release, lift_after_release] = commands.squeeze(dim=0).numpy()

        [x_d, y_d, z_d, JawRadialPos_m] = controller.forward(
            self.object_position, self.target_position, commands).numpy()

        cmd_grasperPos_m = Point(x_d, y_d,
                                 z_d + self.z_offset)  # note that this is in meters, and doesn't account for the offset that is considered 0,0,0 on the gantry.  Need to correct for that afterwards




        #if using the real gantry, stop after one grasp attempt
        if useRealGantry == True:

            if self.neuronset["grasp"]<20 and grasp>=20:
                self.num_grasp_attempts = self.num_grasp_attempts+1
                print(self.num_grasp_attempts)




            if lift_after_release>=20 or move_to_pre_release>=20:
                self.z_offset = 0.07 #add offset to the z so it lifts the object higher

            if move_to_release>=20 or release >= 20:
                self.z_offset = -0.07

            if lift_after_release>=20:
                self.z_offset = 0

            if self.num_grasp_attempts<=1:

                if self.neuronset["lift_after_release"]>=20:
                    JawRadialPos_m = 0
                    cmd_grasperPos_m = self.cmd_position_m #use the last commanded position, don't return to home
                    #self.logger.debug('Lift after release triggered, stay at this position')

                if move_to_pre_release>=20:
                    self.lift_after_grasp_done = True


            else: #if more than 1 grasp attempted
                JawRadialPos_m = 0
                cmd_grasperPos_m = self.cmd_position_m #use the last commanded position, don't return to home
                #self.logger.debug('More than 1 grasp attempted')




        self.cmd_position_m = cmd_grasperPos_m
        self.JawRadialPos_m = JawRadialPos_m



        #neuron state update
        self.neuronset['move_to_pre_grasp'] = move_to_pre_grasp
        self.neuronset['move_to_grasp'] = move_to_grasp
        self.neuronset['grasp'] = grasp
        self.neuronset['lift_after_grasp'] = lift_after_grasp
        self.neuronset['move_to_pre_release'] = move_to_pre_release
        self.neuronset['move_to_release'] = move_to_release
        self.neuronset['release'] = release
        self.neuronset['lift_after_release'] = lift_after_release





        return(cmd_grasperPos_m,JawRadialPos_m)


