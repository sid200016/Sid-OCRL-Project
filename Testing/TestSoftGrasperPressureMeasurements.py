from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper
from datetime import datetime
import time
import numpy as np

SG = SoftGrasper(COM_Port='COM5', BaudRate=460800, timeout=1, controllerProfile="New")  # initialize soft grasper

t0 = time.time()
while True:
    deltaT = time.time() - t0
    #SG.readSerialData()
    SG.MoveGrasper()

    SGPressurevals = [str(SG.PressureArray[i][-1]) if len(SG.PressureArray[i]) > 0 else str(np.NaN) for i in range(0, SG.numPorts)]

    print(','.join(SGPressurevals))
    time.sleep(0.010)
