import numpy as np
import serial
import struct
import sys
from enum import Enum
import re


#########################################################
class PortActions(Enum):
    HOLD = 0 #Don't open any valves, maintain current pressure
    INFLATE = 1 #Open inflation valve and close vacuum valve. This will increase pressure in the Artificial Muscle.
    VACUUM = 2 #Open vacuum valve and close inflation valve.  This will decrease pressure in the Artificial Muscle.
    START = 3 #Class has just been initialized.
    OPEN = 4 #Open both the inflation and vacuum valves.
    INFLATE_AND_STOP = 5 #When you first reach the set point pressure, stop
    INFLATE_AND_MODULATE = 6 #Continue to modulate pressure actively even after you first reach the set point pressure
    IGNORE = 7 #keep executing previous command
class PressurePort:

    def __init__(self,portNumber=0,portStatus=PortActions.HOLD, commandedPressure = 0.0, readPressure = 0.0, maxPressure = 15):
        self.portNumber = portNumber
        self.portStatus = portStatus
        self.commandedPressure = commandedPressure
        self.readPressure = readPressure
        self.maxPressure = maxPressure


class SoftGrasper:

    def __init__(self,COM_Port = 'COM7',BaudRate=115200,timeout=1,controllerProfile = "Legacy"):
        self.ser = serial.Serial(COM_Port, BaudRate, timeout=timeout)

        self.numPorts = 12 #number of ports available
        self.PressurePorts = {0 : PressurePort(0),
                              1 : PressurePort(1),
                              2 : PressurePort(2),
                              3 : PressurePort(3),
                              4 : PressurePort(4),
                              5 : PressurePort(5),
                              6 : PressurePort(6),
                              7 : PressurePort(7),
                              8 : PressurePort(8),
                              9 : PressurePort(9),
                              10 : PressurePort(10),
                              11 : PressurePort(11)
                              } #to hold the pressure values and the status of the ports

        #Variables for legacy protocol
        self.PressureArray = [[] for x in range(0, 12)]  # to store pressure values
        self.PrevJawPress = None  # list to hold the original Jaw Press
        self.JawPos = [8, 9, 11]  # position of pressure values that the jaws are at

        #Tx-Rx Information for New Protocol
        self.startChar = ">>" #indicates start of comm
        self.endChar = "<<" #indicates end of comm
        self.messageStarted = False #true if you have received the startChar
        self.ProtocolSizeStarted = False #true if you have received the protocol type and size
        self.PayloadStarted = False # true if you have received the payload
        self.protocol_type = None #type of message received
        self.payload_size = 0 #size of the payload
        self.txData = None #data read

        #Tx-Rx Information for the modified protocol:
        self.prevBuffer = bytearray() #empty byte array that unprocessed data at the end of the buffer will be processed with

        #What form of the controller  to use: Legacy or New
        self.controllerProfile = controllerProfile

        #Variables for keeping track of commanded position
        self.commandedPosition = {"ClosureChangeInRadius_mm":0, "Jaw1_psi":0, "Jaw2_psi":0, "Jaw3_psi":0}

        #For GUI real-time control
        self.isActive = False

        # #------ Initialization -----#
        # if self.controllerProfile == "Legacy":
        #     self.WaitForJawsToInflate()
        # else:
        #     self.WaitForTeensyInitialization()

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

    def WaitForTeensyInitialization(self):
        print("Waiting for Arduino to initialize")

        FinishedInitialization = False
        while (not FinishedInitialization):
            msg = self.ser.readline()
            msg = msg.decode().rstrip()
            if msg == 'Teensy Ready!':
                print("Soft Grasper Teensy Initialized!")
                FinishedInitialization = True

        return (FinishedInitialization)

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

    def IncrementalMove(self, closureIncrement_mm = 0, jawIncrement_psi = [0,0,0]):
        self.commandedPosition["ClosureChangeInRadius_mm"] = min(max(0,self.commandedPosition["ClosureChangeInRadius_mm"] + closureIncrement_mm),23)
        self.commandedPosition["Jaw1_psi"] = min(max(0,self.commandedPosition["Jaw1_psi"] + jawIncrement_psi[0]),2)
        self.commandedPosition["Jaw2_psi"] = min(max(0,self.commandedPosition["Jaw2_psi"] + jawIncrement_psi[1]),2)
        self.commandedPosition["Jaw3_psi"] = min(max(0,self.commandedPosition["Jaw3_psi"] + jawIncrement_psi[2]),2)

    def MoveGrasper(self):
        PVal = self.GetPressureFromPosition(self.commandedPosition["ClosureChangeInRadius_mm"])
        print(PVal)
        self.SendPressureCommand(PVal)
        self.ReadPressureVals()

        if len(self.PressureArray[0]) > 0:
            print("Pressure val: " + str(self.PressureArray[0][-1]))


        #ChP = SG.getJawChangePressureVals()


    def sendCommunicationArray(self,startDelim = None,byteList = None, endDelim = None): #send list of bytearrays over serial with start and stop delim
        if startDelim is None:
            startDelim = self.startChar

        if endDelim is None:
            endDelim = self.endChar

        numBytesSent = 0 #keep track of number of bytes sent
        byteArr = bytearray()

        val = startDelim.encode('utf-8')
        #other possibility: byteArr.append(bytes.fromhex("ff"))
        byteArr.extend(val)
        numBytesSent += len(val)

        byteArr.extend(byteList)
        numBytesSent += len(byteList)

        val = endDelim.encode('utf-8')
        byteArr.extend(val)
        numBytesSent += len(val)

        self.ser.write(byteArr)

        #format(int.from_bytes(byteArr,sys.byteorder),"b")

        return(numBytesSent)

    def ConstructPortCommand_DEPRECATED(self): #ByteStructure: [<hold_and_actuate_array: ----0001 00010001><INFLATE_AND_STOP, INFLATE_AND_MODULATE, OPEN for ports 11, 4 and 0, respectively: 0x0, 0x5 (port 11), 0x6 (port 4), 0x4 (port 0)>
                # <float pressure for port 11 (4 bytes)><float pressure for port 4 (4 bytes)><float pressure for port 0 (4 bytes)>]
        hold_and_actuate_array = bytearray() #two bytes of : <x,x,x,x,11,10,9,8><7,6,5,4,3,2,1,0> where if value is 0, then it sends a hold, if 1 then it will execute an actuation command
        numBytes = np.ceil(self.numPorts * 4 / 8)
        type_of_action_array = bytearray(int(numBytes)) #get the number of bytes to hold a 4 bit value for each of ports which will be actuated
        floatPressure = bytearray() #empty array to fill in with float values for the ports later

        numActuate = 0

        for i,(k,v) in enumerate(self.PressurePorts.items()):
            if v.portStatus.value == PortActions.HOLD.value:
                tempHold = int.from_bytes(hold_and_actuate_array,sys.byteorder) | 0 << v.portNumber #setup whether to actuate or not
                hold_and_actuate_array = tempHold.to_bytes(tempHold,byteorder = sys.byteorder)
            else:
                hold_and_actuate_array = hold_and_actuate_array | 1 << v.portNumber
                numActuate += 1 #increment the number of ports that will be actuated
                tempAct = int.from_bytes(type_of_action_array,sys.byteorder) | v.portStatus.value << numActuate*4
                type_of_action_array = tempAct.to_bytes(numBytes,byteorder = sys.byteorder) #windows is little endian , so x[0] is the least significant byte
                floatPressure =floatPressure.extend(struct.pack("f",v.commandedPressure)) #send in the pressure value

        BytesToSend = bytearray()
        BytesToSend.extend(hold_and_actuate_array)
        BytesToSend.extend(type_of_action_array)
        BytesToSend.extend(floatPressure)
        return(BytesToSend)


    def ConstructPortCommand(self):

        hold_and_actuate_array = bytearray()  # six bytes of  : <aaaabbbb><ccccdddd>...<kkkkllll> where aaaa corresponds to a value between 0 and 15 in the PortActions class above for port 0, bbbb corresponds to port 1, kkkk corresponds to port 10 and llll corresponds to port 11
        numBytes = int(np.ceil(self.numPorts * 4 / 8)) #should be 6

        type_of_action_array = bytearray(int(numBytes))  # get the number of bytes to hold a 4 bit value for each of the ports which will be actuated
        PressureVal = bytearray()  # empty array to fill in with values for the ports later where the value sent is between 0 and 127, and corresponds to the following psi value: max_pressure_psi/floatPressure

        numActuate = 0

        for i, (k, v) in enumerate(self.PressurePorts.items()):


            tempAct = int.from_bytes(type_of_action_array, sys.byteorder) | (v.portStatus.value << (i) * 4)
            type_of_action_array = tempAct.to_bytes(numBytes, byteorder=sys.byteorder)  # windows is little endian , so x[0] is the least significant byte

            if v.portStatus.value == PortActions.INFLATE_AND_MODULATE.value or v.portStatus.value == PortActions.INFLATE_AND_STOP.value:
                numActuate += 1  # increment the number of ports that will be actuated
                PressureVal.extend( round(v.commandedPressure*127/v.maxPressure).to_bytes(1,sys.byteorder) )  # send in the pressure value #only the first byte, i.e. PressureVal[0], contains information


        BytesToSend = bytearray()
        BytesToSend.extend(type_of_action_array)
        BytesToSend.extend(PressureVal)
        return (BytesToSend)


    def readSerialData(self): #inspired by: https://be189.github.io/lessons/14/asynchronous_streaming.html
        # protocol is: > PROTOCOL_BYTE SIZE_BYTE |PAYLOAD of SIZE BYTES| < #https://forum.arduino.cc/t/sending-raw-binary-data/694724
        if self.ser.inWaiting()>0: #if there is data to read
            numBytes = self.ser.inWaiting()
            xd = self.ser.read(numBytes) #read the entire buffer

            totalBuffer = bytearray()
            totalBuffer.extend(self.prevBuffer)
            totalBuffer.extend(xd) #total buffer is the unprocessed data from last round with the new data from this round

            #check to see if the byte(s) representing the start of the communication is present and if the byte(s) representing the end of the communication is present
            rePayload = re.compile(b'.*?>>(?P<Payload>.*?)<<.*?') #replace >> and << with the start and end indicator pos
            payload = rePayload.finditer(totalBuffer)
            index_StartStop = {"start":[],"stop":[]}

            for m in payload: #iterate through matches representing the payload
                payload_m = m.group('Payload')
                index_StartStop["start"].append(m.start('Payload'))
                index_StartStop["stop"].append(m.end('Payload')) #store index of the start and stop of the match.  Will use later to see if there are unprocessed data at the end of the string

                #To DO: sanity check to determine if the number of bytes is correct
                payloadRE = re.compile(b'(?P<ProtocolByte>\D)(?P<PayloadSize>\D)(?P<Payload>.+)')
                payloadRes = payloadRE.search(payload_m)

                protocolType = None
                numBytes = None
                payload = None
                
                if payloadRes is not None: #if the structure of the payload is in the expected form, then extract the number of bytes etc.
                    protocolType = int.from_bytes(payloadRes.group('ProtocolByte'),sys.byteorder)
                    numBytes = int.from_bytes(payloadRes.group('PayloadSize'),sys.byteorder)
                    payload = payloadRes.group('Payload') #need to check this
                    
                if numBytes == len(payload):
                    print('Payload matches the expected number of bytes')

                else:
                    print('Warning: Payload size does not match the expected number of bytes')


                self.processData(protocolType,numBytes,payload)

                #To DO: if stop index is shorter than the length of the total payload, then need to store that for processing in the next time step

                #To DO: If the last stop index is exactly at the end of the total payload, reset self.prevBuffer to be empty.


                
            


            # startpos = totalBuffer.find(self.startChar.encode('utf-8'))
            # endpos = totalBuffer.find(self.endChar.encode('utf-8'))
            #
            # if startpos ~= -1 and endpos ~= -1:
            #     #If both the start and end indicators are present, then process the data.
            #     self.processData(xd)

            #If the start is present but not the end, then add that data to the buffer

    
    def processData(self,protocolType=None, numBytes=None, payload=None):

        if protocolType is not None:
            match protocolType:
                case 0:
                    len_payload = math.floor((len(payload))/4)
                    data=[struct.unpack('f', newByte2[x:x + 4]) for x in (range(0, len_payload) % 4)] #for floats representing pressure values of each port

                case 1:
                    #for strings
                    print(payload.decode())


                case _:
                    #action - default
                    pass



        
        

    def readBuffer(self):
        numBytes = self.ser.inWaiting()
        xd = self.ser.read(numBytes)
        return(xd,numBytes)
    def readSerialData_option2(self): #inspired by: https://forum.arduino.cc/t/pc-arduino-comms-using-python-updated/574496
        #protocol is: > PROTOCOL_BYTE SIZE_BYTE |PAYLOAD of SIZE BYTES| < #https://forum.arduino.cc/t/sending-raw-binary-data/694724
        print(self.ser.inWaiting())
        dataStr = None
        if self.ser.inWaiting() > 0 and self.messageStarted == False: #if there is data to read
            xd = self.ser.read().decode("utf-8")

            if xd == self.startChar:
                self.messageStarted = True

            else:
                print(xd)



        # if two or more bytes in the buffer, read the protocol type and payload size
        if self.ser.inWaiting()>=2 and self.messageStarted == True and self.ProtocolSizeStarted == False:
            self.protocol_type = self.ser.read()
            self.payload_size = int.from_bytes(self.ser.read(), sys.byteorder)
            self.ProtocolSizeStarted = True

        # if payload_size or more in buffer, read the payload
        if self.ser.inWaiting()>=self.payload_size and self.messageStarted == True and self.ProtocolSizeStarted == True and self.PayloadStarted == False:

            self.txData =self.ser.read(self.payload_size)
            self.PayloadStarted = True

            #Perform Analysis of data
            print(self.txData)
            print('Got Payload')

        # if 1 or more bytes in buffer, check if message is complete
        if self.ser.inWaiting()>=1 and self.messageStarted == True and self.ProtocolSizeStarted == True and self.PayloadStarted == True:

            xd = self.ser.read().decode("utf-8")

            if xd == self.endChar:
                self.messageStarted = False
                self.ProtocolSizeStarted = False
                self.PayloadStarted = False
                self.protocol_type = None
                self.payload_size = 0
                dataStr = self.txData
                self.txData = None

        return dataStr





























