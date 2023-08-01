import asyncio
import time
import socketio

import time

from EmbeddedSystems.SoftGrasper.SoftGrasper import PortActions
from EmbeddedSystems.SoftGrasper.SoftGrasper import SoftGrasper

SG  = None
GCa = None
curr_z = 0 #current z value based on joystick
closureIncrement_mm = 0


#loop = asyncio.get_event_loop()
sio = socketio.AsyncClient()
start_timer = None


async def send_ping():
    print('Sending ping')
    global start_timer
    start_timer = time.time()
    await sio.emit('my event','New Data')

async def send_Start():
    print('Sending Start')
    global start_timer
    start_timer = time.time()
    await sio.emit('start event','Ready to Start')

@sio.event
async def connect():
    print('connected to socket')
    await send_Start()


@sio.on('SendInfo')
async def pong_from_server(data):
    global start_timer
    latency = time.time() - start_timer
    print('latency is {0:.2f} ms'.format(latency * 1000))
    print(data)
    await sio.sleep(1)
    if sio.connected:
        print('Ping')
        await send_ping()

@sio.on('gantry position commands')
async def gantryPosition(data):
    print('gantry')
    global GCa, curr_z
    latency = time.time()
    print('Time is {0:.2f} ms'.format(latency * 1000))
    print(data)

    # GCa.setXYZ_Position(data[0], data[1], curr_z) #absolute move

    # if sio.connected:
    #     print('Ping')
    #     await send_ping()

@sio.on('soft grasper commands')
async def softGrasperCommands(data):
    print('softGrasper')
    global SG

    closureIncrement_mm = data['grasper_l']*20/100
    print(closureIncrement_mm)
    #SG.AbsoluteMove(closureIncrement_mm=closureIncrement_mm, jawIncrement_psi=[0, 0, 0])
    #SG.MoveGrasper()  # actuate soft grasper



async def HardwareInitialize():
    global SG
    SG = SoftGrasper(COM_Port='COM4', BaudRate=460800, timeout=1, controllerProfile="New")

async def program_loop():

    await sio.emit('gantry position commands', 'Gantry pos')
    await sio.emit('soft grasper commands', 'Soft Grasper Commands')
    print('ProgramLoop')



async def start_Program():
    await HardwareInitialize()
    await sio.connect('http://localhost:5000')
    await program_loop()
    await sio.wait()




if __name__ == '__main__':
    asyncio.run(start_Program())

    # executor = ProcessPoolExecutor(2) #https://stackoverflow.com/questions/29269370/how-to-properly-create-and-run-concurrent-tasks-using-pythons-asyncio-module
    # loop = asyncio.new_event_loop()
    # boo = loop.run_in_executor(executor, say_boo)
    # baa = loop.run_in_executor(executor, say_baa)
    #
    # loop.run_forever()