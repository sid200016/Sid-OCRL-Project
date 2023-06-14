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
import numpy as np

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

class RigidGrasper:

    def __init__(self,BAUDRATE = 57600, DEVICEPORT = "COM3", GoalPosition1=[1500,2000], GoalPosition2 = [2120,1620]):

        # Control table address
        self.ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
        self.ADDR_PRO_GOAL_POSITION = 116
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
        self.GoalPosition = {"1":[1500,2000], "2":[2120,1620]}

        # Current position of the grasper:
        self.CurrentPosition ={"1":[], "2":[]}

        # Communication result and errors
        self.dxl_comm_result = None
        self.dxl_error = None

        # SetupMotors
        self.setupComms_and_Motors()



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
            print("%s\n" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s\n" % packetHandler.getRxPacketError(dxl_error))

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
            print("%s\n" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s\n" % packetHandler.getRxPacketError(dxl_error))

        self.dxl_comm_result = dxl_comm_result
        self.dxl_error = dxl_error
        return (value,dxl_comm_result, dxl_error)

    def SetGoalPosition(self,goal_position1,goal_position2):
        goal_position={"1":goal_position1,"2":goal_position2}

        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):
            dxl_comm_result,dxl_error = self.writeByte(4, DXL_ID, self.ADDR_PRO_GOAL_POSITION,goal_position[k])

        return(dxl_comm_result,dxl_error)


    def ReadCurrentPosition(self):
        for i,(k,DXL_ID) in enumerate(self.DXL_ID.items()):
            dxl_present_position, dxl_comm_result, dxl_error = self.readByte(4,DXL_ID,self.ADDR_PRO_PRESENT_POSITION)
            self.CurrentPosition[k] = dxl_present_position

        return (self.CurrentPosition,dxl_comm_result,dxl_error)

    def ClosePort(self):

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

    def CyclicTestGrasper(self):

        index = 0
        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            # Write goal position for Motor 1
            self.SetGoalPosition(self.GoalPosition["1"][index],self.GoalPosition["2"][index])


            MotorFin = {"1":False,"2":False}

            while 1:

                # Read present position
                CurrentPosition, dxl_comm_result, dxl_error = self.ReadCurrentPosition()
                print(CurrentPosition)

                for i,(k,currPos) in enumerate(CurrentPosition.items()):

                    if abs(self.GoalPosition[k][index] - currPos) > self.DXL_MOVING_STATUS_THRESHOLD:
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s\n" % packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s\n" % packetHandler.getRxPacketError(dxl_error))

                        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % (self.DXL_ID[k],
                                                                          self.GoalPosition[k][index], currPos))

                    else:
                        MotorFin[k] =True

                if np.all(np.array(list(MotorFin.values()))==True):
                    break

            # Change goal position
            if index == 0:
                index = 1
            else:
                index = 0



RG = RigidGrasper()
RG.CyclicTestGrasper()




