from flask import request, redirect, url_for, session
from flask_socketio import emit, join_room

# import EmbeddedSystems.Gantry.GantryController as GC
# import EmbeddedSystems.RigidGrasper.RigidGrasper as RG
#import GUI.EmbeddedSystems.SoftGrasper.SoftGrasper as SG

from .extensions import socketio

users = {}
page = 'setup'
print('events ran')

# Gantry / Grasper setup
#SGa = SG.SoftGrasper(COM_Port = 'COM5',BaudRate=460800,timeout=1,controllerProfile="New") #soft grasper initialization
# GCa = GC.Gantry(comport = "COM4", homeSystem = True) #homeSystem = False, initPos=[0,0,0] )
# RGa = RG.RigidGrasper(BAUDRATE = 57600, DEVICEPORT = "COM6", GoalPosition1=[1500,2000], GoalPosition2 = [2120,1620])

def map(val, ilo, ihi, flo, fhi):
    return flo + ((fhi - flo) / (ihi - ilo)) * (val - ilo)

def mouse_to_mm(mouse_position):
    if mouse_position == None: return 0;

    mouse_position = float(mouse_position)
    return int(map(mouse_position, 0, 860, -255, 255))

@socketio.on("proc-join")
def handle_proc_join():
    if 'proctor' in users:
        return
    
    print("proctor joined")
    join_room('proctor')
    page = 'landing'

@socketio.on("part-join")
def handle_part_join():
    if 'participant' in users:
        return
    
    print("participant joined")
    join_room('participant')
    page = 'landing'

@socketio.on("info-done")
def handle_info_done():
    # emit("goto-action-proc", to='proctor')
    emit("goto-action", room='participant')
    page = 'action-soft'

@socketio.on("gantry-move")
def handle_gantry_move(vals):

    x = -mouse_to_mm(vals['x']) #need to check if -ve is correct
    y = mouse_to_mm(vals['y'])
    print(x, ',', y)

    socketio.emit('gantry position commands', {'x': x, 'y': y})

    # GCa.getPosition()
    # curr_z = GCa.PositionArray["z"][0]
    # GCa.setXYZ_Position(x, y, curr_z) #absolute move

@socketio.on("damping-change")
def handle_damping_change(val):
    print('damping: ' + str(val))

@socketio.on("stiffness-change")
def handle_stiffness_change(val):
    print('stiffness: ' + str(val))

@socketio.on("power-change")
def handle_power_change(val):
    print(val)
    socketio.emit('soft grasper commands', {'grasper_l': val})
    #SGa.AbsoluteMove(closureIncrement_mm = float(val)*20/100, jawIncrement_psi = [0,0,0])
    #SGa.MoveGrasper()

@socketio.on("item-event")
def handle_item_event_proctor(data):

    n = data['n']
    typ = data['typ']
    attempt = data['att']

    print(f'item {n} is {typ}')
    emit("update-events-proctor", {'n':n, 'typ': typ, 'att':attempt}, room="proctor")
    emit("update-events-participant", {'n':n, 'typ': typ, 'att':attempt}, room="participant")


# For hardware and grasper communication
@socketio.on('gantry position commands')
def sendGantryPosition(string):
    print('gantry')


@socketio.on('soft grasper commands')
def sendGantryPosition(string):
    print('softGrasper')



@socketio.on('my event')
def handle_my_custom_event(json):
    print('received json: ' + str(json))
    fsio.emit('SendInfo', 'Received')

@socketio.on('start event')
def handle_start_event(string):
    print('received string: ' + str(string))
    SID = request.sid
    socket_NAMESPACE = request.namespace
