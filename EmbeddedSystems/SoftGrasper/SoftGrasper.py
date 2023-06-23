
import numpy as np
import serial
import struct


#########################################################
class SoftGrasper:

    def __init__(self,COM_Port = 'COM7',BaudRate=115200,timeout=1):
        self.ser = serial.Serial(COM_Port, BaudRate, timeout=timeout)
        self.PressureArray =[[] for x in range(0,12)] #to store pressure values
        self.WaitForJawsToInflate()
        self.PrevJawPress = None #list to hold the original Jaw Press
        self.JawPos=[8,9,11]  #position of pressure values that the jaws are at

    def GetPressureFromPosition(self,position_mm,coeffs=[3.034e-6,-1.251e-4,9.3731e-4,0.0217,-0.3659,2.0752,0]):
        b = coeffs
        x = position_mm
        x=min(x,19)  #limit to maximum 19 mm contraction
        pressV = b[0]*(x**6) + b[1]*(x**5) + b[2]*(x**4) + b[3]*(x**3) + b[4]*(x**2) + b[5]*(x) + b[6]*1
        pressV=min(pressV,12)  #limit pressure
        return(pressV)
    def SendPressureCommand(self,PressureVal):
        byteFval = bytearray(struct.pack("f", PressureVal))
        self.ser.write(byteFval)
    def WaitForJawsToInflate(self):
        WaitForJawsToInflate = True
        self.ser.write("Ready\n".encode('utf-8'))

        while (WaitForJawsToInflate):
            line = self.ser.readline()
            if line.decode().rstrip() == 'Jaws finished!':
                WaitForJawsToInflate = False
            print(line)

    def ReadPressureVals(self):
        line = self.ser.readline().decode()
        try:
            pVal = [float(x) for x in line.split(",")[1:]]
            if len(pVal)!=12:
                print("Not enough pressure values:")
                print(line)
            else:
                for count,val in enumerate(pVal):
                    self.PressureArray[count].append(pVal[count])
        except Exception as e:
            print("Error during read")

    def getJawChangePressureVals(self):
        if (len(self.PressureArray[self.JawPos[0]])<=2):
            return([0,0,0])
        else:

            CurJawPress = [self.PressureArray[x][-1] for x in self.JawPos]
            #PrevJawPress= [self.PressureArray[x][-2] for x in self.JawPos]
            if self.PrevJawPress is None:
                self.PrevJawPress=CurJawPress

            ChangeInPressure = (np.array(CurJawPress)-np.array(self.PrevJawPress)).tolist()
            #return(ChangeInPressure)
            return(ChangeInPressure)