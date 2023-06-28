from SoftGrasper import SoftGrasper,PortActions


SG  = SoftGrasper(COM_Port = 'COM4',BaudRate=460800,timeout=1)
while (True):
    for i,(k,v) in enumerate(SG.PressurePorts.items()):
        SG.PressurePorts[k].commandedPressure=i
        SG.PressurePorts[k].PortStatus = PortActions.INFLATE_AND_STOP


    byteList = SG.ConstructPortCommand()
    SG.sendCommunicationArray(byteList=byteList)
    vv=SG.readSerialData()
    print("BigLoop:"+str(vv))
