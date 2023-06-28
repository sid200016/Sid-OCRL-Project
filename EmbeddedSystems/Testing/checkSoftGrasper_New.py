from EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper
from EmbeddedSystems.SoftGrasper.SoftGrasper import PortActions
import time

SG  = SoftGrasper(COM_Port = 'COM4',BaudRate=460800,timeout=1,controllerProfile="New")
while (True):
    for i,(k,v) in enumerate(SG.PressurePorts.items()):
        SG.PressurePorts[k].commandedPressure=i
        SG.PressurePorts[k].PortStatus = PortActions.INFLATE_AND_STOP


    byteList = SG.ConstructPortCommand()
    SG.sendCommunicationArray(byteList=byteList)
    vv=SG.readSerialData()
    print("BigLoop:"+str(vv))
    time.sleep(0.001)
