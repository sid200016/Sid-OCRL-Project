import pygame
import time
import numpy as np
import SoftGrasper.SoftGrasper
import RigidGrasper.RigidGrasper as RG
import JoyCon.JoyCon as JC
import Gantry.GantryController as GC

GCa = GC.Gantry()
RGa = RG.RigidGrasper(GantryS = GCa)
jcRG = JC.Joy_RigidGrasper(RGa)

while(True):
    [buttonVal,AxesPos] = jcRG.eventLoop()
    jcRG.ExecuteButtonFunctions(buttonVal, AxesPos)
    jcRG.MoveGantry(AxesPos[0], AxesPos[1])
    # for i, (k,v) in enumerate(buttonVal.items()):
    #     if v==1:
    #         jcRG.buttonMapping[k].fcn()