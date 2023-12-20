#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Borrowed from:
# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
# To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os
from enum import Enum

import numpy as np
import re
import struct
import sys


import serial
import math

import logging
from datetime import datetime

from pathlib import Path

from dynamixel_sdk import *

from time import time



if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class GrasperActions(Enum):
    STAY = 0 #dont move
    CLOSE = 1 #Move jaws closer together
    OPEN = 2 #Move jaws apart

class ForceSensor(Enum):
    IGNORE = 0 #don't do anything
    READ_FORCE = 1 #read force value

class RigidGrasper:

    def __init__(self,BAUDRATE = 57600, DEVICEPORT = "COM3", GoalPosition1=[1489,2000], GoalPosition2 = [2125,1620], useForceSensor = False, COM_Port_Force = 'COM7',BaudRate_Force=115200,timeout=1):

        # setup Logger
        self.logger = None
        self.setupLogger()

        # Control table address
        self.ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
        self.ADDR_PRO_GOAL_POSITION = 116
        self.ADDR_PRO_PRESENT_CURRENT = 126 #current in units of 1.34 mA
        self.ADDR_PRO_PRESENT_POSITION = 132

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID = {"1":1,"2":2} #Dynamixel IDs
        self.BAUDRATE = BAUDRATE  # Dynamixel default baudrate : 57600
        self.DEVICENAME = DEVICEPORT  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE = 2000  # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE = 1500  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 100  # Dynamixel moving status threshold

        self.OPERATING_MODE = 11
        self.CONTROL_MODE = 3 #position control

        self.POSITION_PROP_BYTE = 84
        self.POSITION_PROP_GAIN = 100

        self.VELOCITY_PROP_BYTE = 78
        self.VELOCITY_PROP_GAIN = 0

        self.VELOCITY_I_BYTE = 76
        self.VELOCITY_I_GAIN = 0

        # Port and packet handlers for communicating with the USB2DYNAMIXEL
        self.portHandler = None
        self.packetHandler = None

        # Goal Positions for the Grasper:
        self.NumMotors = 2 #one Dynamixel each for left and right
        self.GoalPosition_Limits = {"1":GoalPosition1, "2":GoalPosition2} #limits of motion

        # Current position of the grasper:
        self.CurrentPosition ={"1":[], "2":[]}

        # Commanded Position for the grasper
        self.commandedPosition_mm = 0

        # Present Current in mA
        self.PresentCurrent = {"1": [], "2": []}

        # Communication result and errors
        self.dxl_comm_result = None
        self.dxl_error = None

        # SetupMotors
        self.setupComms_and_Motors()

        #Force Sensor
        self.useForceSensor = useForceSensor
        if self.useForceSensor == True:
            self.numPorts = 1
            self.ForceArray = [[] for x in range(0, self.numPorts)]
            self.PrevJawForce = None #list to hold the original Jaw force
            self.changeInForce = 0 #change in force relative to baseline jaw force.

            # Tx-Rx Information for New Protocol
            self.startChar = ">!"  # indicates start of comm
            self.endChar = "!<"  # indicates end of comm
            self.messageStarted = False  # true if you have received the startChar
            self.ProtocolSizeStarted = False  # true if you have received the protocol type and size
            self.PayloadStarted = False  # true if you have received the payload
            self.protocol_type = None  # type of message received
            self.payload_size = 0  # size of the payload
            self.txData = None  # data read

            # Tx-Rx Information for the modified protocol:
            self.prevBuffer = bytearray()  # empty byte array that unprocessed data at the end of the buffer will be processed with

            #Serial port for force sensor
            self.ser = serial.Serial(COM_Port_Force, BaudRate_Force, timeout=timeout)

    def setupLogger(self):
        ##### Set up logging ####
        logger_soft = logging.getLogger(__name__)

        fname = Path(__file__).parents[3].joinpath('datalogs', str(__name__) + datetime.now().strftime(
            "_%d_%m_%Y_%H_%M_%S") + ".txt")

        fh = logging.FileHandler(fname)  # file handler
        fh.setLevel(logging.DEBUG)

        ch = logging.StreamHandler(sys.stdout)  # stream handler
        ch.setLevel(logging.DEBUG)

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        logger_soft.setLevel(logging.DEBUG)
        # add the handlers to the logger_soft
        logger_soft.addHandler(fh)
        logger_soft.addHandler(ch)
        self.logger = logger_soft
    def setupComms_and_Motors(self):

        # Setup PacketHandlers and Port Handlers
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Port opening succeeded")
        else:
            print("Port opening failed")

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Baudrate change succeeded")
        else:
            print("Baudrate change failed")


        #Setup Motors

        for i, (k,DXL_ID) in enumerate(self.DXL_ID.items()):
            # set operating mode
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.OPERATING_MODE,
                                                                      self.CONTROL_MODE)

            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.ADDR_PRO_TORQUE_ENABLE,
                                                                      self.TORQUE_ENABLE)

            # set gain
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, self.POSITION_PROP_BYTE,
                                                                      self.POSITION_PROP_GAIN)

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, self.VELOCITY_PROP_BYTE,
                                                                      self.VELOCITY_PROP_GAIN)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, self.VELOCITY_I_BYTE,
                                                                      self.VELOCITY_I_GAIN)
            # get position
            dxl_present_position, self.dxl_comm_result, self.dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID,
                                                                                            self.ADDR_PRO_PRESENT_POSITION)

            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif self.dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel has been successfully connected")

            self.CurrentPosition[k] = dxl_present_position


    def writeByte(self,numBytes,DXL_ID,REG_ADDR,Value):
        match numBytes:
            case 1:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, REG_ADDR,Value)

            case 2:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, REG_ADDR,Value)

            case 4:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, REG_ADDR,
                                                                               Value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s\n" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s\n" % self.packetHandler.getRxPacketError(dxl_error))

        self.dxl_comm_result = dxl_comm_result
        self.dxl_error = dxl_error
        return(dxl_comm_result,dxl_error)

    def readByte(self,numBytes,DXL_ID,REG_ADDR):
        match numBytes:
            case 1:
                value,dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, REG_ADDR)

            case 2:
                value,dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, REG_ADDR)

            case 4:
                value,dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, REG_ADDR)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s\n" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s\n" % self.packetHandler.getRxPacketError(dxl_error))

        self.dxl_comm_result = dxl_comm_result
        self.dxl_error = dxl_error
        return (value,dxl_comm_result, dxl_error)

    def SetGoalPosition(self,goal_position1=None,goal_position2=None):
        goal_position={"1":goal_position1,"2":goal_position2}

        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):
            if goal_position[k] is not None: #only send byte if it is not none
                dxl_comm_result,dxl_error = self.writeByte(4, DXL_ID, self.ADDR_PRO_GOAL_POSITION,goal_position[k])

        return(dxl_comm_result,dxl_error)


    def ReadCurrentPosition(self):
        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):
            dxl_present_position, dxl_comm_result, dxl_error = self.readByte(4,DXL_ID,self.ADDR_PRO_PRESENT_POSITION)
            self.CurrentPosition[k] = dxl_present_position

        return (self.CurrentPosition,dxl_comm_result,dxl_error)

    def ReadCurrent(self):
        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):
            dxl_present_current, dxl_comm_result, dxl_error = self.readByte(2,DXL_ID,self.ADDR_PRO_PRESENT_CURRENT)
            self.PresentCurrent[k] = dxl_present_current
            print(dxl_present_current)
            print("Current in mA for actuator %s: %i"%(k,self.PresentCurrent[k]))

        return (self.PresentCurrent,dxl_comm_result,dxl_error)


    def ClosePort(self):\

        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):

            # Disable Dynamixel Torque for the motors
            self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.ADDR_PRO_TORQUE_ENABLE,
                                                                      self.TORQUE_DISABLE)
            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s\n" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif self.dxl_error != 0:
                print("%s\n" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()


    def GetCountFromGripperWidth(self,gripperWidth_mm,coeffs_M1=[-0.0032,-6.8731,801.91],coeffs_M2=[0.000007,-0.002,0.2001,-13.864,793.44]):
        gW = np.clip(gripperWidth_mm*2,16,111) #SNS produces a radius value. Multiply by 2 to get distance between jaws. Limit between 16 and 111 mm.

        M1_init_count = 2173 #offset corresponding to maximum gripper width
        M2_init_count = 1463 #offset corresponding to maximum gripper width
        M1_count = M1_init_count - (coeffs_M1[0]*(gW**2) + coeffs_M1[1]*(gW**1) + coeffs_M1[2]*(gW**0))
        M2_count = M2_init_count + (coeffs_M2[0]*(gW**4) + coeffs_M2[1]*(gW**3) +coeffs_M2[2]*(gW**2) + coeffs_M2[3]*(gW**1) + coeffs_M2[4]*(gW**0))

        M1_count = np.clip(M1_count,self.GoalPosition_Limits["1"][0],self.GoalPosition_Limits["1"][1])
        M2_count = np.clip(M2_count, self.GoalPosition_Limits["2"][0], self.GoalPosition_Limits["2"][1])
        return(M1_count,M2_count)


    def IncrementalMove(self,moveIncrement1 = 100,moveIncrement2 = 100, action1 = GrasperActions.STAY,action2 = GrasperActions.STAY): #close claws, assume position control

        CurrentPosition,dxl_comm_result,dxl_error = self.ReadCurrentPosition() #get current position and update member variable with the same.
        CurrentPosition=[CurrentPosition["1"],CurrentPosition["2"]]

        #Claw 1
        if action1.value == GrasperActions.CLOSE.value:
            CurrentPosition[0] = max(CurrentPosition[0]-moveIncrement1,self.GoalPosition_Limits["1"][0])
        elif action1.value == GrasperActions.OPEN.value:
            CurrentPosition[0] = min(CurrentPosition[0] + moveIncrement1, self.GoalPosition_Limits["1"][1])

        self.SetGoalPosition(goal_position1=CurrentPosition[0])  # move towards goal position for claw 1 only

        #Claw 2
        if action2.value == GrasperActions.CLOSE.value:
            CurrentPosition[1] = min(CurrentPosition[1] + moveIncrement2, self.GoalPosition_Limits["2"][0])
        elif action2.value == GrasperActions.OPEN.value:
            CurrentPosition[1] = max(CurrentPosition[1] - moveIncrement2, self.GoalPosition_Limits["2"][1])
        self.SetGoalPosition(goal_position2=CurrentPosition[1])  # move towards goal position for claw 2 only


        #Read pressure sensor:
        if self.useForceSensor == True:
            byteList = self.ConstructPortCommand()
            numBytes = self.sendCommunicationArray(byteList=byteList)
            self.logger.debug("Bytes sent:%i" % (numBytes))

            # read serial data
            self.readSerialData()

            #ChP = self.getJawChangePressureVals()
            #self.changeInPressure = ChP
            #self.logger.debug("Change in pressure: " + ','.join([str(x) for x in ChP]))

    def MoveGrasper(self): #close claws, assume position control

        M1_count,M2_count = self.GetCountFromGripperWidth(self.commandedPosition_mm)
        self.SetGoalPosition(goal_position1=M1_count)  # move towards goal position for claw 1 only
        self.SetGoalPosition(goal_position2=M2_count)  # move towards goal position for claw 2 only


        #Read pressure sensor:
        if self.useForceSensor == True:
            byteList = self.ConstructPortCommand()
            numBytes = self.sendCommunicationArray(byteList=byteList)
            self.logger.debug("Bytes sent:%i" % (numBytes))

            # read serial data
            self.readSerialData()

            ChF = self.getJawChangeForce()
            self.changeInForce = ChF
            self.logger.debug("Change in Force: " + ','.join([str(x) for x in ChF]))

    def sendCommunicationArray(self, startDelim=None, byteList=None,
                               endDelim=None):  # send list of bytearrays over serial with start and stop delim
        if startDelim is None:
            startDelim = self.startChar

        if endDelim is None:
            endDelim = self.endChar

        numBytesSent = 0  # keep track of number of bytes sent
        byteArr = bytearray()

        val = startDelim.encode('utf-8')
        # other possibility: byteArr.append(bytes.fromhex("ff"))
        byteArr.extend(val)
        numBytesSent += len(val)

        byteArr.extend(byteList)
        numBytesSent += len(byteList)

        val = endDelim.encode('utf-8')
        byteArr.extend(val)
        numBytesSent += len(val)

        self.ser.write(byteArr)

        # format(int.from_bytes(byteArr,sys.byteorder),"b")

        return (numBytesSent)

    def ConstructPortCommand(self):

        hold_and_actuate_array = bytearray()  # Should just be one integer
        numBytes = int(np.ceil(self.numPorts * 8 / 8))  # should be 1

        type_of_action_array = bytearray(
            int(numBytes))  # get the number of bytes to hold a 4 bit value for each of the ports which will be actuated

        tempAct = int(ForceSensor.READ_FORCE.value) #int.from_bytes(type_of_action_array, sys.byteorder) | ForceSensor.READ_FORCE
        type_of_action_array = tempAct.to_bytes(numBytes, byteorder=sys.byteorder)  # windows is little endian , so x[0] is the least significant byte

        BytesToSend = bytearray()
        BytesToSend.extend(type_of_action_array)
        return (BytesToSend)

    def readSerialData(self):  # inspired by: https://be189.github.io/lessons/14/asynchronous_streaming.html
        # protocol is: > PROTOCOL_BYTE SIZE_BYTE |PAYLOAD of SIZE BYTES| < #https://forum.arduino.cc/t/sending-raw-binary-data/694724
        if self.ser.inWaiting() > 0:  # if there is data to read
            numBytes = self.ser.inWaiting()
            xd = self.ser.read(numBytes)  # read the entire buffer

            totalBuffer = bytearray()
            totalBuffer.extend(self.prevBuffer)
            totalBuffer.extend(
                xd)  # total buffer is the unprocessed data from last round with the new data from this round

            # check to see if the byte(s) representing the start of the communication is present and if the byte(s) representing the end of the communication is present
            rePayload = re.compile(
                b'.*?>!(?P<Payload>.*?)!<.*?')  # replace >> and << with the start and end indicator pos
            payload = rePayload.finditer(totalBuffer)
            index_StartStop = {"start": [], "stop": []}

            for m in payload:  # iterate through matches representing the payload
                payload_m = m.group('Payload')
                index_StartStop["start"].append(m.start('Payload'))
                index_StartStop["stop"].append(m.end(
                    'Payload'))  # store index of the start and stop of the match.  Will use later to see if there are unprocessed data at the end of the string

                # To DO: sanity check to determine if the number of bytes is correct
                payloadRE = re.compile(b'(?P<ProtocolByte>.)(?P<PayloadSize>.)(?P<Payload>.+)')
                payloadRes = payloadRE.search(payload_m)

                protocolType = None
                numBytes = None
                payload = None

                if payloadRes is not None:  # if the structure of the payload is in the expected form, then extract the number of bytes etc.
                    protocolType = int.from_bytes(payloadRes.group('ProtocolByte'), sys.byteorder)
                    numBytes = int.from_bytes(payloadRes.group('PayloadSize'), sys.byteorder)
                    payload = payloadRes.group('Payload')

                if payload is not None:
                    if numBytes == len(payload):
                        self.logger.info('Payload matches the expected number of bytes')
                        self.processData(protocolType, numBytes, payload)

                    else:
                        self.logger.error(
                            'Warning: Payload size does not match the expected number of bytes\n Protocol Type %s, Numbytes expected %s, Length of payload %s ' % (
                            str(protocolType), str(numBytes), str(len(payload))))
                        self.logger.debug(payload)

                # To DO: if stop index is shorter than the length of the total payload, then need to store that for processing in the next time step

                # To DO: If the last stop index is exactly at the end of the total payload, reset self.prevBuffer to be empty.

            # startpos = totalBuffer.find(self.startChar.encode('utf-8'))
            # endpos = totalBuffer.find(self.endChar.encode('utf-8'))
            #
            # if startpos ~= -1 and endpos ~= -1:
            #     #If both the start and end indicators are present, then process the data.
            #     self.processData(xd)

            # If the start is present but not the end, then add that data to the buffer

    def calcForceFromSensor(self,reading,coeffs = [4.6e-15, 0.03704, 0.117, 0.003891]):
        reading = np.clip(reading, 0, 1000)

        Force_N = coeffs[0]*np.exp(coeffs[1]*reading) + coeffs[2]*np.exp(coeffs[3]*reading)
        Force_N = clip(Force_N, 0, 40)
        return(Force_N)
    def processData(self, protocolType=None, numBytes=None, payload=None):

        if protocolType is not None:
            match protocolType:
                case 0:  # pressure values
                    len_payload = math.floor((len(payload)))
                    data = [struct.unpack('f', payload[x:x + 4])[0] for x in
                            range(0, len_payload, 4)]  # for floats representing pressure values of each port

                    for count, val in enumerate(data):
                        ForceN = self.calcForceFromSensor(data[count])
                        self.ForceArray[count].append(ForceN)

                    self.logger.debug("Data for Case 0 (Force values): " + ','.join([str(x) for x in data]))

                case 1:  # strings
                    # for strings
                    self.logger.debug("Payload Case 1 (strings): " + payload.decode())

                case _:
                    # action - default
                    pass

    def getJawChangeForce(self):
        if (len(self.PressureArray[self.JawPos[0]]) <= 0):
            return ([0])
        else:

            CurJawForce = [self.ForceArray[0][-1]]
            # PrevJawPress= [self.PressureArray[x][-2] for x in self.JawPos]
            if self.PrevJawForce is None:
                self.PrevJawForce = CurJawForce
                self.logger.info('baseline Jaw force is ' + str(self.PrevJawForce))

            ChangeInForce = (np.array(CurJawForce) - np.array(self.PrevJawForce)).tolist()
            # return(ChangeInPressure)
            return (ChangeInForce)


def CyclicTestGrasper(self):

        index = 0
        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position for Motor 1
            self.SetGoalPosition(self.GoalPosition_Limits["1"][index],self.GoalPosition_Limits["2"][index])


            MotorFin = {"1":False,"2":False}

            while 1:

                # Read present position
                CurrentPosition, dxl_comm_result, dxl_error = self.ReadCurrentPosition()
                print(CurrentPosition)

                for i,(k,currPos) in enumerate(CurrentPosition.items()):

                    if abs(self.GoalPosition_Limits[k][index] - currPos) > self.DXL_MOVING_STATUS_THRESHOLD:
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s\n" % packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s\n" % packetHandler.getRxPacketError(dxl_error))

                        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % (self.DXL_ID[k],
                                                                          self.GoalPosition_Limits[k][index], currPos))

                    else:
                        MotorFin[k] =True

                if np.all(np.array(list(MotorFin.values()))==True):
                    break

            # Change goal position
            if index == 0:
                index = 1
            else:
                index = 0

if __name__ == '__main__':
    RG = RigidGrasper(DEVICEPORT = "COM4",useForceSensor = False, COM_Port_Force = 'COM3',BaudRate_Force=460800)
    CurrentPosition, dxl_comm_result, dxl_error = RG.ReadCurrentPosition()
    print("%i,%i. In Deg: %f, %f:" % (
    CurrentPosition["1"], CurrentPosition["2"], CurrentPosition["1"] * 360 / 4096, CurrentPosition["2"] * 360 / 4096))

    while(input("Press a key to increment by 100")):
        RG.IncrementalMove(moveIncrement1 = 50,moveIncrement2 = 50, action1 = GrasperActions.CLOSE,action2 = GrasperActions.CLOSE)
        time.sleep(3)
        #RG.ReadCurrent()
        CurrentPosition,dxl_comm_result,dxl_error = RG.ReadCurrentPosition()
        print("%i,%i. In Deg: %f, %f:"%(CurrentPosition["1"],CurrentPosition["2"],CurrentPosition["1"]*360/4096,CurrentPosition["2"]*360/4096))



# RG = RigidGrasper()
# RG.CyclicTestGrasper()




