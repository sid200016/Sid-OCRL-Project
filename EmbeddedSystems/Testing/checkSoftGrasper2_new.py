from SoftGrasper.SoftGrasper import SoftGrasper
from SoftGrasper.SoftGrasper import PortActions
import time

SG  = SoftGrasper(COM_Port = 'COM4',BaudRate=460800,timeout=1,controllerProfile="New")

while (True):
    SG.readSerialData()
    #print("BigLoop:"+str(vv))
    time.sleep(0.001)