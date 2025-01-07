import asyncio
import socketio

import time
from enum import Enum

from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import PortActions
from GUI.EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper
from GUI.EmbeddedSystems.Gantry.GantryController import Gantry as GantryController
import GUI.EmbeddedSystems.JoyCon.JoyCon as JC
from GUI.EmbeddedSystems.SNS.SNScontroller import SNScontroller
from GUI.EmbeddedSystems.Support.Structures import GrasperContactForce,Point

import logging
from datetime import datetime
import sys
from pathlib import Path

##### Set up logging ####
l_date = datetime.now().strftime("_%d_%m_%Y_%H_%M_%S")
loggerR = logging.getLogger(__name__)
fname = Path(__file__).parents[1].joinpath("datalogs","Hardware"+l_date+".txt")

fh = logging.FileHandler(fname) #file handler
fh.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout) #stream handler
ch.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

fh.setFormatter(formatter)
ch.setFormatter(formatter)
loggerR.setLevel(logging.DEBUG)

# add the handlers to the logger
loggerR.addHandler(fh)
loggerR.addHandler(ch)



#### Set up logging data logger
datalogger = logging.getLogger(__name__+"_datalogs")

fname = Path(__file__).parents[1].joinpath("datalogs","HardwareDatalog"+l_date+".csv")

fh2 = logging.FileHandler(fname) #file handler
fh2.setLevel(logging.DEBUG)

ch2 = logging.StreamHandler(sys.stdout) #stream handler
ch2.setLevel(logging.ERROR)

formatter2 = logging.Formatter('%(asctime)s,%(name)s,%(levelname)s,%(message)s')

fh2.setFormatter(formatter2)
ch2.setFormatter(formatter2)

datalogger.setLevel(logging.DEBUG)

# add the handlers to the logger
datalogger.addHandler(fh2)
datalogger.addHandler(ch2)




SG  = None #soft grasper
GC = None #gantry controller
jcSG = None #joystick
SNSc = None #SNS controller

useSG = True #set to true to use the soft grasper
useGC = True #set to true to use the gantry
useJoy = True #set to true to use the joystick
usejcSG = useJoy and useSG and useGC #set to true to use the joystick with the soft grasper and the gantry controller
useSNS = True #set to true if you want to use the SNS to control

buttonVal = None #read from joystick
AxesPos = None #joystick axes position

fragileThreshold = 0.4 #scale of 0 to 1.  Forces above this are considered broken for fragile objects

max_z_height = -0.184
SNS_object_pos_m = [0,0,max_z_height]
SNS_target_pos_m = [0,-0.25,max_z_height]
z_offset = 0

pressureThreshold = [0.05,0.05,0.05] #threshold above which to trigger the rumble, psi
pressureScaling = 1 #100% pressure value for fumble.  In psi. formula is (max change in pressure - pressurethreshold)/PressureScaling

maxJawChangeInRadius_mm = 14 #maximum jaw change in radius allowed.
SNS_BypassForceFeedback = False
class itemStatus(str,Enum):
    notstarted = 'notstarted'
    inprogress = 'inprogress'
    fail = 'fail'
    success = 'success'
    broken = 'broken'

class itemClass(Enum):
    fixed= 'fixed'
    breakable = 'breakable'
    graspable = 'graspable'
    notspecified = 'notspecified'

class itemTrack:
    def __init__(self,objectID, status = itemStatus.notstarted,itemClass=itemClass.graspable):
        self.objectID = objectID
        self.status = status #Not started, inprogress, fail, broken, success
        self.itemClass = itemClass #fixed, breakable, graspable

#object 1 is top left, object 9 is bottom right.
ObjectVal = {"1": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "2": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "3": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "4": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "5": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "6": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "7": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "8": itemTrack(1, itemStatus.notstarted, itemClass.notspecified),
             "9": itemTrack(1, itemStatus.notstarted, itemClass.notspecified)
             }

localHostName = 'http://localhost:8080'

updatedSoftGrasper = False


#loop = asyncio.get_event_loop()
sio = socketio.AsyncClient()
start_timer = None




async def send_ping():
    global start_timer, loggerR
    loggerR.debug('Sending ping')

    start_timer = time.time()
    await sio.emit('my event','New Data')

async def send_Start():
    global start_timer, loggerR
    loggerR.debug('Sending Start')

    start_timer = time.time()
    await sio.emit('start event','Ready to Start')

@sio.event
async def connect():
    global loggerR
    loggerR.debug('connected to socket')
    await send_Start()

@sio.event
def disconnect():
    global loggerR
    loggerR.info('disconnected from server')
@sio.on('gantry position commands')
async def gantryPosition(data):

    global GC, loggerR, useGC, useSNS, SNS_object_pos_m, SNS_target_pos_m, max_z_height
    loggerR.info('Received gantry position command')
    loggerR.debug(data)
    if useGC == True:
        GC.goalPos = [data['x']-20, data['y']-20, GC.goalPos[2]] #x and y positions come from screen, z position is kept from the joystick
    else:
        loggerR.debug("Gantry disabled")
    loggerR.debug("GoalPos changed in Gantry Position: " + ','.join([str(x) for x in GC.goalPos]))

    #if using the SNS control, update the object position x, y to be the goal pos.  the Z is a fixed height of -0.189 m
    if useSNS == True:
        SNS_fixed_height_z = max_z_height #height of object in meters relative to start position. z height offset is 0.2 m.
        SNS_object_pos_m = [GC.goalPos[0]/1000, GC.goalPos[1]/1000, SNS_fixed_height_z]
        SNS_target_pos_m = [0.09, 0.05, SNS_fixed_height_z]
        loggerR.info('Commanded object pos in m: %f, %f %f'%(*SNS_object_pos_m,))
        loggerR.debug("Modified the SNS object and target positions")

@sio.on('soft grasper commands')
async def softGrasperCommands(data):

    global SG, updatedSoftGrasper, loggerR, useSG
    loggerR.info('Received soft grasper power command')

    closure_mm = data['grasper_l']*20/100
    loggerR.debug("Change in radius in mm: "+str(closure_mm))

    if useSG == True:
        SG.commandedPosition["ClosureDistance_mm"] = closure_mm
        SG.commandedPosition["Jaw1_psi"] = 0
        SG.commandedPosition["Jaw2_psi"] = 0
        SG.commandedPosition["Jaw3_psi"] = 0

    else:
        loggerR.debug("Soft grasper disabled")

    updatedSoftGrasper = True

    #SG.AbsoluteMove(closureIncrement_mm=closureIncrement_mm, jawIncrement_psi=[0, 0, 0]) #setup internal variables to actuate at MoveGrasper
    #SG.MoveGrasper()  # actuate soft grasper


@sio.on('item-event_emit')
def handle_item_event_hardware(data):

    global loggerR, ObjectVal

    n = data['n']
    typ = data['typ']
    attempt = data['att']
    test = data['test']
    loggerR.info('Item event: item %s is %s.  Attempt %s, test %s)'%(n,typ,attempt,test) )
    ObjectVal[str(int(n)+1)].status = itemStatus[typ]

@sio.on('object-location')
def handle_specified_object_location(data):
    global SNS_object_pos_m, loggerR
    SNS_object_pos_m = [v for k,v in data.items()] #should already be in meters: data["x_m"],data["y_m"],data["z_m"]
    loggerR.info('Deformable Test: Received new object location (x,y,z [in meters]): %f, %f, %f' % tuple(SNS_object_pos_m))

@sio.on('grasper-max')
def handle_specified_object_location(data):
    global maxJawChangeInRadius_mm, loggerR
    maxJawChangeInRadius_mm= data #should already be in mm
    loggerR.info('Deformable Test: Received new maximum change in Radius (in mm): %f' % data)



@sio.on('TestInfo')
def handle_TestInfo(user_data, trial_data):

    global loggerR, ObjectVal
    id_code = user_data['id_code']
    age = user_data['age']
    gender = user_data['gender']
    types = user_data['item_types']

    test = trial_data['test']

    num_attempts = trial_data['num_attempts']

    loggerR.info("ID code %s, age: %s, gender: %s, types: %s, test: %s, num_attempts: %s"%(id_code, age,
                                                                                           gender, types,
                                                                                           test, num_attempts))

    for k,v in enumerate(types):
        v = v.lower()
        if v in [str(x.name) for x in itemClass]:
            ObjectVal[str(k+1)].itemClass = itemClass[v] #assign object Type

        else:
            ObjectVal[str(k + 1)].itemClass = itemClass.notspecified





async def HardwareInitialize():
    global SG, GC, jcSG, loggerR, useSG, useGC, usejcSG, SNSc, z_offset

    time.sleep(120)
    await sio.emit('Initialize-Start','Finished')
    loggerR.info('Finished Initialization')

@sio.on('program_loop')
async def program_loop():
    global SG, GC, jcSG, updatedSoftGrasper, loggerR, datalogger, buttonVal, AxesPos, useSG, useGC, usejcSG, SNSc, SNS_object_pos_m, SNS_target_pos_m, pressureScaling, pressureThreshold, max_z_height, maxJawChangeInRadius_mm, SNS_BypassForceFeedback

    #await sio.emit('gantry position commands', 'Gantry pos')
    #await sio.emit('soft grasper commands', 'Soft Grasper Commands')
    try:
        while(True):



            if SNS_BypassForceFeedback == True:

                grasperContact = GrasperContactForce(*[0,0,0]) if 12 <maxJawChangeInRadius_mm else GrasperContactForce(*[20,20,20]) #set contact threshold based on the position




            await asyncio.sleep(0.001) #allow other tasks to run
            loggerR.debug('ProgramLoop')
            await datalogFcn()


    except KeyboardInterrupt:
        loggerR.debug('Exiting loop')
        return

def calculateRumble(pressureThreshold = [0.2,0.2,0.2],pressureScaling=1):
    global SG, ObjectVal, fragileThreshold, loggerR
    #pressureThreshold:  change in pressure threshold in psi above which to register changes in pressure
    rumbleValue_arr = [min((x - pressureThreshold[i]) / pressureScaling, 1) if x >= pressureThreshold[i] else 0 for (i, x) in
                   enumerate(SG.changeInPressure)] #was divide by 1.75, changed to 1

    rumbleValue = max(rumbleValue_arr)
    # if the current attempt is on a breakable item, check to see if the rumble value is greater than the threshold

    idx = [k for k,x in ObjectVal.items() if x.status == itemStatus.inprogress] #find the key which in progress

    try:
        if len(idx) > 0:
            idx = idx[0]
            if ObjectVal[idx].itemClass == itemClass.breakable and rumbleValue > fragileThreshold:
                rumbleValue = 0
                loggerR.warning('Force exceeded on item %s.  Threshold %f, rumble value %f'%
                                (idx, fragileThreshold, rumbleValue))


    except Exception as e:
        loggerR.error(e)
        loggerR.error('Error during fragile object check')
        rumbleValue = 0

    return (rumbleValue)

async def datalogFcn():
    global SG, GC, jcSG,  datalogger, buttonVal, AxesPos, useSG, useGC, usejcSG



    # Send information to other script on current position of gantry and state of grasper
    grasper_width = 6.77
    await sio.emit('Robot-Status', {'x_mm':13.3, 'y_mm':14.7, 'z_mm':18.9, 'grasper_width_mm':grasper_width , 'closure_pressure_psi': 8.33,
                                'jaw1_psi': 0.26, 'jaw2_psi': 0.27, 'jaw3_psi': 0.28})





async def start_Program():
    loggerR.info('Triggering connection to ' + str(localHostName))
    await sio.connect(localHostName)

    await HardwareInitialize()
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