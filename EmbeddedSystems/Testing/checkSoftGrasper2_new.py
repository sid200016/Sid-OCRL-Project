import time

from SoftGrasper.SoftGrasper import PortActions
from SoftGrasper.SoftGrasper import SoftGrasper

SG  = SoftGrasper(COM_Port = 'COM4',BaudRate=460800,timeout=1,controllerProfile="New")

while (True):

    # construct port commands and send to Teensy
    SG.PressurePorts[1].portStatus = PortActions.HOLD
    SG.PressurePorts[0].portStatus = PortActions.INFLATE_AND_MODULATE
    SG.PressurePorts[0].commandedPressure = 10.3

    byteList = SG.ConstructPortCommand()
    numBytes = SG.sendCommunicationArray(byteList=byteList)
    print("Bytes sent:%i" % (numBytes))

    # read serial data
    SG.readSerialData()
    print('New')

    #print("BigLoop:"+str(vv))
    time.sleep(0.001)