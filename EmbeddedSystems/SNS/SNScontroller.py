import pathlib
import pandas as pd
import numpy as np
import time
import torch

from .controller.SNS_layer import perceptor, controller

from ..Support.Structures import Point,Velocity,Acceleration,GrasperContactForce

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

        self.object_position = torch.Tensor(list(objectPos_m)).unsqueeze(dim=0)
        self.target_position = torch.Tensor(list(targetPos_m)).unsqueeze(dim=0)


    def SNS_forward(self,grasperPos_m:Point, grasperContact:GrasperContactForce,objectPos_m:Point = None,
                    targetPos_m:Point = None):
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

        if objectPos_m is not None:
            self.object_position = torch.Tensor(list(objectPos_m)).unsqueeze(dim=0) #update the object pos

        if targetPos_m is not None:
            self.target_position = torch.Tensor(list(targetPos_m)).unsqueeze(dim=0) #update the target pos


        force = torch.Tensor(list(grasperContact)).unsqueeze(dim=0)
        gripper_position = torch.Tensor(list(grasperPos_m)).unsqueeze(dim=0)

        commands = perceptor.forward(
            gripper_position, self.object_position, self.target_position, force)

        [move_to_pre_grasp, move_to_grasp, grasp, lift_after_grasp, move_to_pre_release,
         move_to_release, release, lift_after_release] = commands.squeeze(dim=0).numpy()

        [x_d, y_d, z_d, JawRadialPos_m] = controller.forward(
            object_position, target_position, commands).numpy()

        if lift_after_release > 10:
            self.object_position = torch.Tensor([0, 0, 0]).unsqueeze(dim=0)


        cmd_grasperPos_m=Point(x_d,y_d,z_d) #note that this is in meters, and doesn't account for the offset that is considered 0,0,0 on the gantry.  Need to correct for that afterwards



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


