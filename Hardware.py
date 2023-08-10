import asyncio
import socketio

import time

from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import PortActions
from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper
from GUI.EmbeddedSystems.Gantry.GantryController import Gantry as GantryController
import GUI.EmbeddedSystems.JoyCon.JoyCon as JC

import logging
from datetime import datetime
import sys


##### Set up logging ####
logger = logging.getLogger("Hardware")
fname = "Hardware"+datetime.now().strftime("_%d_%m_%Y_%H_%M_%S")

fh = logging.FileHandler(fname) #file handler
fh.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout) #stream handler
ch.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

fh.setFormatter(formatter)
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)

#### Set up logging data logger
datalogger = logging.getLogger("HardwareDatalog")
fname = "HardwareDatalog"+datetime.now().strftime("_%d_%m_%Y_%H_%M_%S")

fh2 = logging.FileHandler(fname) #file handler
fh2.setLevel(logging.DEBUG)

ch2 = logging.StreamHandler(sys.stdout) #stream handler
ch2.setLevel(logging.ERROR)

formatter2 = logging.Formatter('%(asctime)s,%(name)s,%(levelname)s,%(message)s')

fh2.setFormatter(formatter2)
ch2.setFormatter(formatter2)
# add the handlers to the logger
datalogger.addHandler(fh)
datalogger.addHandler(ch)


logger.setLevel(logging.INFO)


SG  = None #soft grasper
GC = None #gantry controller
jcSG = None #joystick

ObjectVal = {"1":"Not Started",
             "2":"Not Started",
             "3":"Not Started",
             "4": "Not Started",
             "5": "Not Started",
             "6": "Not Started",
             "7": "Not Started",
             "8": "Not Started",
             "9": "Not Started"
             }

localHostName = 'http://localhost:8000'

updatedSoftGrasper = False


#loop = asyncio.get_event_loop()
sio = socketio.AsyncClient()
start_timer = None




async def send_ping():
    global start_timer, logger
    logger.debug('Sending ping')

    start_timer = time.time()
    await sio.emit('my event','New Data')

async def send_Start():
    global start_timer, logger
    logger.debug('Sending Start')

    start_timer = time.time()
    await sio.emit('start event','Ready to Start')

@sio.event
async def connect():
    global logger
    logger.debug('connected to socket')
    await send_Start()

@sio.on('gantry position commands')
async def gantryPosition(data):

    global GC, logger
    logger.info('Received gantry position command')
    logger.debug(data)
    GC.goalPos = [data['x'], data['y'], GC.goalPos[2]] #x and y positions come from screen, z position is kept from the joystick
    logger.debug("GoalPos changed in Gantry Position: " + ','.join([str(x) for x in GC.goalPos]))

@sio.on('soft grasper commands')
async def softGrasperCommands(data):

    global SG, updatedSoftGrasper, logger
    logger.info('Received soft grasper power command')

    closure_mm = data['grasper_l']*20/100
    logger.debug("Change in radius in mm: "+str(closure_mm))
    SG.commandedPosition["ClosureChangeInRadius_mm"] = closure_mm
    SG.commandedPosition["Jaw1_psi"] = 0
    SG.commandedPosition["Jaw2_psi"] = 0
    SG.commandedPosition["Jaw3_psi"] = 0

    updatedSoftGrasper = True

    #SG.AbsoluteMove(closureIncrement_mm=closureIncrement_mm, jawIncrement_psi=[0, 0, 0]) #setup internal variables to actuate at MoveGrasper
    #SG.MoveGrasper()  # actuate soft grasper


@sio.on('item-event')
def handle_item_event_hardware(data):

    global hardware, ObjectVal

    n = data['n']
    typ = data['typ']
    attempt = data['att']
    test = data['test']
    logger.info('Item event: item %s is %s.  Attempt %s, test %s)'%(n,typ,attempt,test) )
    ObjectVal[int(n)] = typ






async def HardwareInitialize():
    global SG, GC, jcSG, logger

    logger.info('Initializing Soft Grasper...')
    SG = SoftGrasper(COM_Port='COM5', BaudRate=460800, timeout=1, controllerProfile="New") #initialize soft grasper
    logger.info('Finished initializing Soft Grasper')

    logger.info('Initializing Gantry...')
    GC = GantryController(comport = "COM4",homeSystem = True)#, homeSystem = False,initPos=[0,0,0]  #initialize gantry controller
    logger.info('Finished initializing Gantry'!)

    logger.info('Initializing Joystick...')
    jcSG = JC.Joy_SoftGrasper(SGa=SG, GantryS=GC) #initialize joystick control of soft grasper and gantry controller
    logger.info('Finished initializing Joystick!')
    
    logger.info('Finished Initialization')

@sio.on('program_loop')
async def program_loop():
    global SG, GC, jcSG, updatedSoftGrasper, logger, datalogger
    #await sio.emit('gantry position commands', 'Gantry pos')
    #await sio.emit('soft grasper commands', 'Soft Grasper Commands')
    try:
        while(True):
            [buttonVal, AxesPos] = jcSG.eventLoop()  # run event loop to determine what values and axes to execute
            jcSG.ExecuteButtonFunctions(buttonVal,
                                        AxesPos)  # execute Button functions defined by the button values and axes positions
            posInc,feedrate_mmps = jcSG.calcPositionIncrement(AxesPos[0], AxesPos[1])  # get the joystick position for x, y and z, corrected for the flip on the x axis
            GC.calculateIncrementalMove(*posInc) #will set the GC.goalPos with the appropriate increments
            GC.setXYZ_Position(*GC.goalPos,feedrate_mmps*60)  # absolute move of the gantry

            SG.MoveGrasper()  # jcRG.executeButtonFunctions should update SGa with the the pressures, this command sends the appropriate commands to the grasper over serial


            if updatedSoftGrasper == True:
                logger.debug("UPDATED Soft Grasper has happened")
                updatedSoftGrasper = False

            # Rumble feedback based on pressure change
            pressureThreshold = [0.4, 0.4,
                                 0.4]  # change in pressure threshold in psi above which to register changes in pressure
            rumbleValue = [(x - pressureThreshold[i]) / 1.5 if x >= pressureThreshold[i] else 0 for (i, x) in
                           enumerate(SG.changeInPressure)]
            jcSG.rumbleFeedback(max(rumbleValue), max(rumbleValue), 1000)

            sio.emit('set-contact-force-soft', max(rumbleValue))

            await asyncio.sleep(0.001) #allow other tasks to run
            logger.debug('ProgramLoop')


    except KeyboardInterrupt:
        logger.debug('Exiting loop')
        return


async def datalogFcn():
    global SG, GC, jcSG. datalogger

    # should be x, y and z position of gantry, pressure in closure muscle, pressure in jaw 1, pressure in jaw 2, pressure in jaw3, button 1 status....button 11 status, axes 0 joystick, axes 1 joystick,  object 1 status .... object 9 status
    ds = ''
    ds="%f , %f , %f"%(GC.PositionArray["x"][-1], GC.PositionArray["y"][-1], GC.PositionArray["z"][-1])
    ds = ds + "%f , %f , %f , %f"%(SG.PressureArray[SG.closureMuscle_idx],
                                   SG.PressureArray[SG.JawPos[0]],
                                   SG.PressureArray[SG.JawPos[1]],
                                   SG.PressureArray[SG.JawPos[2]])
    ds = ds +"%i , %i , %i , %i , %i , %i , %i , %i , %i , %i , %i , %i , %i , %f , %f" % (*buttonVal,*AxesPos)
    ds = ds + "%s , %s , %s , %s , %s , %s , %s , %s , %s"% ( *ObjectVal )



    datalogger.info(ds)





async def start_Program():
    await HardwareInitialize()
    logger.info('Triggering connection to ' + str(localHostName))
    await sio.connect(localHostName)
    await program_loop()
    await sio.wait()





if __name__ == '__main__':
    try:
        asyncio.run(start_Program())
    except KeyboardInterrupt:
        "Exiting Program.  Thank you."

    # executor = ProcessPoolExecutor(2) #https://stackoverflow.com/questions/29269370/how-to-properly-create-and-run-concurrent-tasks-using-pythons-asyncio-module
    # loop = asyncio.new_event_loop()
    # boo = loop.run_in_executor(executor, say_boo)
    # baa = loop.run_in_executor(executor, say_baa)
    #
    # loop.run_forever()