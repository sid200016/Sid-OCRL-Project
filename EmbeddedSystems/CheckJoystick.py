import pygame
import time
import numpy as np
import SoftGrasper.SoftGrasper
import RigidGrasper.RigidGrasper as RG
import JoyCon.JoyCon as JC
import Gantry.GantryController as GC

GCa = GC.Gantry(comport = "COM4", homeSystem = False,initPos=[0,0,0])
#RGa = RG.RigidGrasper()
#jcRG = JC.Joy_RigidGrasper(RGa, GantryS = GCa)
jcRG = JC.Joy_Gantry(GantryS = GCa)


while(True):
    [buttonVal,AxesPos] = jcRG.eventLoop()
    jcRG.ExecuteButtonFunctions(buttonVal, AxesPos)
    jcRG.MoveGantry(AxesPos[0], AxesPos[1])
    # for i, (k,v) in enumerate(buttonVal.items()):
    #     if v==1:
    #         jcRG.buttonMapping[k].fcn()