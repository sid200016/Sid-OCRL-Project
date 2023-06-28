import pygame
import time
import numpy as np

from EmbeddedSystems.Gantry.GantryController import Gantry as GC
from EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper as SG
from EmbeddedSystems.RigidGrasper.RigidGrasper import RigidGrasper as RG
import EmbeddedSystems.JoyCon.JoyCon as JC

import cProfile


def checkJoystick():
    GCa = GC.Gantry(comport = "COM4")#, homeSystem = False,initPos=[0,0,0]  `````````````

    # Rigid Grasper Only
    #RGa = RG.RigidGrasper()
    #jcRG = JC.Joy_RigidGrasper(RGa, GantryS = GCa)

    # Gantry Only
    #jcRG = JC.Joy_Gantry(GantryS = GCa)

    # Soft Grasper Only
    jcRG = JC.Joy_SoftGrasper(SGa=SGa, GantryS=GCa)

    for i in range(0,1000000):
        [buttonVal,AxesPos] = jcRG.eventLoop()
        jcRG.ExecuteButtonFunctions(buttonVal, AxesPos)
        jcRG.MoveGantry(AxesPos[0], AxesPos[1])
        # for i, (k,v) in enumerate(buttonVal.items()):
        #     if v==1:
        #         jcRG.buttonMapping[k].fcn()
        time.sleep(0.01)



cProfile.run('checkJoystick()')