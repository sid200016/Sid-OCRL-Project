class gantry:
    #objects: pressure/force, right claw, left claw, position?
    #flags: open/close

class target:
    #objects: name, position?, pressure sensor, type of object
    #flags: success/fail/new
    #function: update GUI? maybe gui is a class?
    #this may be split up into target position and object

class goal:
    #only one exists
    #objects: weight, position
    #function: get weight

class target_object:
    #objects: name (String), shape?

class placeholder_gui:

    # objects: targets, goal, attempts, gantry, gantry_force_bar

    # setup of system
    # add objects to gantry
    #
    def __init__(self):

    def StepSimulation(self)




#tap on screen (array of doubles representing position of tap in gantry space: [x,y,z]) triggers logic check
#check if grasper is in contact with object (call getContact function, returns double force, boolean inContact (true if in contact, false if not), boolean GrasperState (true if grasper has been "closed", false if grasper is "open") )
    #if yes, attempt started (contact force > 0, location = target, grasper not in resting state)
    #if near target, attempt not started, and no contact, attempts decrease

#if gantry has released object, check location (attempt must be started)
    #if in goal tolerance, update target with a checkmark, attempts decrease
    #if not in goal tolerance, attempts decrease
        #if in easy mode, red X does not show up
        #if in hard mode, target is updated with red X

#check the number of attempts left
    #if > 0, find a new random target
        #pick 1-9 and check if attempted or not (getState)
        #highlight new selected target
        #decrement number of targets left
    #if no attempts left or if no targets left
        #end session

