import pygame
import time
import numpy as np
import SoftGrasper.SoftGrasper
import RigidGrasper.RigidGrasper as RG
import JoyCon.JoyCon as JC

RGa = RG.RigidGrasper()
jcRG = JC.Joy_RigidGrasper(RGa)

while(True):
    [buttonVal,AxesPos] = jcRG.eventLoop()
    jcRG.ExecuteButtonFunctions(buttonVal, AxesPos)
    # for i, (k,v) in enumerate(buttonVal.items()):
    #     if v==1:
    #         jcRG.buttonMapping[k].fcn()