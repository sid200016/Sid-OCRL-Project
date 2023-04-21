class example:
    def __init__(self, x, y):
        self.sum = x+y
    def getSum(self):
        print(self.sum)


ex1 = example(3, 4)
ex1.getSum()
ex1.sum = 30
ex1.getSum()

v = input('give me a command')
print(v)

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