import asyncio
import socketio
from aiohttp import web
import aioconsole #https://stackoverflow.com/questions/58454190/python-async-waiting-for-stdin-input-while-doing-other-stuff

import time
from enum import Enum

from datetime import datetime
import sys
from pathlib import Path

### Run concurrently with "Hardware_SNS_Deformable.py"


sio = socketio.AsyncServer(async_mode='aiohttp')
app = web.Application()
sio.attach(app)

statusDict = {"Hardware Initialized":False, "Loop Status":True, "Started Calibration":False, "Finished Calibration":False}
robotCalibration_Vals = {} #to store the robot calibration

@sio.on('start event')
def handle_start_event(v,string):
    print('received string: ' + str(string))


@sio.on('Initialize-Start')
def handle_hardware_initialize(v,data):
    statusDict["Hardware Initialized"] = True


@sio.on('Robot-Status')
def handle_RobotStatus(v,data):
    global robotCalibration_Vals, statusDict
    if statusDict["Started Calibration"] == True and statusDict["Finished Calibration"] == False:
        #print(data) #print the data to the screen
        pass
    if statusDict["Finished Calibration"] == True:
        robotCalibration_Vals = data #will be a dict of x,y,z, grasper position, pressure in closure muscle, pressure in jaw 1, pressure in jaw 2, pressure in jaw 3

@sio.on('SNS-start')
def handle_SNS_start(v,string):
    print(string)

async def HandleProgram():
    global statusDict, robotCalibration_Vals



    # Boot System
    while (True):
        if statusDict["Hardware Initialized"] == True:
            # Say system has booted, do you want to enter calibration routine (Y/N)
            s = await aioconsole.ainput("System initialization finished.\n Do you want to enter calibration routine? (Y/N) ")

            # If N, then proceed to main script.  If Y, then proceed to calibration routine.

            if s.upper() == 'Y':
                # Give prompt to enter "F" to finish calibration routine.

                print("Use joystick to move and position grasper. Enter 'F' when finished with calibration.\n Data will start streaming in 5 seconds")
                await asyncio.sleep(5)
                statusDict["Started Calibration"] = True
                fincal = await aioconsole.ainput()
                match fincal.upper():
                    case "F":
                        statusDict["Finished Calibration"] = True
                        await asyncio.sleep(2)

                        print("Robot calibration values:\n x:%f, y:%f, z:%f, closure:%f, closure pressure:%f, jaw pressure 1: %f, jaw pressure 2: %f, jaw pressure 3: %f \n"%tuple([v for k,v in robotCalibration_Vals.items()]))


                        useXYZcalibration = await aioconsole.ainput(
                            "Do you want to use the XYZ position from the calibration for the target position? \n Enter 'Y' for yes, 'N' to use default position, 'T' to adjust the calibration xyz value, or 'X' to enter a new value. \n")

                        match useXYZcalibration.upper():

                            case "Y":
                                statusDict["sendCalibrationXYZ"] = True

                            case "N":
                                statusDict["sendCalibrationXYZ"] = False

                            case "T":
                                vals = await aioconsole.ainput("Expecting 3 values for x,y,z offset separated by comma and in mm \n")
                                vals = [float(x) for x in vals.split(',')]
                                robotCalibration_Vals["x_mm"] = robotCalibration_Vals["x_mm"] + vals[0]
                                robotCalibration_Vals["y_mm"] = robotCalibration_Vals["y_mm"] + vals[1]
                                robotCalibration_Vals["z_mm"] = robotCalibration_Vals["z_mm"] + vals[2]
                                statusDict["sendCalibrationXYZ"] = True

                            case "X":
                                vals = await aioconsole.ainput("Expecting 3 values for new x,y,z separated by comma and in mm \n")
                                vals = [float(x) for x in vals.split(',')]
                                robotCalibration_Vals["x_mm"] = vals[0]
                                robotCalibration_Vals["y_mm"] = vals[1]
                                robotCalibration_Vals["z_mm"] = vals[2]
                                statusDict["sendCalibrationXYZ"] = True

                            case _:
                                print("Please enter 'Y, N or T' when finished \n")
                                statusDict["Finished Calibration"] = False
                                statusDict["sendCalibrationXYZ"] = False

                        if statusDict["sendCalibrationXYZ"] == True:
                            print("x,y,z used is (mm): %f, %f, %f" % (robotCalibration_Vals["x_mm"],robotCalibration_Vals["y_mm"],robotCalibration_Vals["z_mm"]))
                            await sio.emit("object-location",{"x_m":robotCalibration_Vals["x_mm"]/1000,"y_m":robotCalibration_Vals["y_mm"]/1000,"z_m":robotCalibration_Vals["z_mm"]/1000})


                        # Ask it they want to adjust or enter new value for the grasper max closure calibration
                        useGrasperCalibration = await aioconsole.ainput(
                            "Do you want to use the grasper maximum change in radial position from the calibration for the target position?\n Enter 'Y' for yes, 'N' to use default position, 'T' to adjust the calibration grasper value, or 'X' to enter a new value.\n")

                        match useGrasperCalibration.upper():

                            case "Y":
                                statusDict["sendCalibrationGrasper"] = True

                            case "N":
                                statusDict["sendCalibrationGrasper"] = False

                            case "T":
                                vals = await aioconsole.ainput(
                                    "Expecting value in mm, this will be an offset")
                                robotCalibration_Vals["grasper_width_mm"] = robotCalibration_Vals["grasper_width_mm"] + float(vals)
                                statusDict["sendCalibrationGrasper"] = True

                            case "X":
                                vals = await aioconsole.ainput(
                                    "Expecting value in mm. This is the new value for maximum change in radial position.")
                                robotCalibration_Vals["grasper_width_mm"] = float(vals)
                                statusDict["sendCalibrationGrasper"] = True

                            case _:
                                print("Please enter 'Y, N or T' when finished")
                                statusDict["Finished Calibration"] = False
                                statusDict["sendCalibrationGrasper"] = False




                        if statusDict["sendCalibrationGrasper"] == True:
                            await sio.emit("grasper-max",robotCalibration_Vals["grasper_width_mm"])
                            print("Grasper width used in mm: %f" % robotCalibration_Vals["grasper_width_mm"])
                            '''To be done:
                                add emit for hardware initialize ("Initialize-Start") -> done
                                add emit for robot status xyz,grasper pos, pressures ("Robot-Status") -> done
                                edit status_dict for Finished Calibration, sendCalibrationGrasper etc. 
                                add handlers on hardware side for: object-location and grasper-max -> done
                                Edit hardware program to only set force threshold 
                                modify run_Program
                                test and see the units of the position 
                                Do trial run   
                            '''

                        # Prompt to press home button on Joystick to return to home position
                        waitHome = await aioconsole.ainput('Press any button to return grasper to Home. \n')
                        await sio.emit("Go-Home", "home")


                        startTest = await aioconsole.ainput('Press the SR button on the joystick to start the test. \n')



            else:
                "Please enter Y or N"


        await asyncio.sleep(0.001)





'''
Experimental Protocol:
- System boots up
- Asks if you want to enter calibration routine
- Use joystick to move grasper and record when grasper touches the object by pressing a keyboard button
- save offset value
- ask it want to start experiment
- If yes, ask for additional offset
*grasper will not close beyond this value. 
'''








async def run_Program():

    # https://stackoverflow.com/questions/53465862/python-aiohttp-into-existing-event-loop

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, 'localhost', 8080)
    await site.start()

    await asyncio.sleep(1)
    print("Server started ...")
    runp = asyncio.create_task(HandleProgram())
    await runp

    print('Before forever sleep')
    while True:
        await sio.sleep(1)

        #await HandleProgram()


if __name__ == '__main__':
    try:
        #web.run_app(app)
        asyncio.run(run_Program())
    except KeyboardInterrupt:
        "Exiting Program.  Thank you."