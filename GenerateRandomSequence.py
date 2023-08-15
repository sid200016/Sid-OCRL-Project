import logging
import random
from datetime import datetime
from pathlib import Path
import sys
import string

##### Set up logging ####
l_date = datetime.now().strftime("_%d_%m_%Y_%H_%M_%S")
loggerR = logging.getLogger(__name__)
fname = Path(__file__).parents[0].joinpath("datalogs","RandomSequence"+l_date+".txt")

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

#log the user's name
pName = input("Enter the id of the partipant")
loggerR.info("ID: %s"%pName)

#generate random ID
randID = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
loggerR.info("Random ID: %s",randID)
numExperiments = 6

#Generate which of the SNS or full-human control should be done in the six trials
controltype = ['SNS','full-human','SNS','full-human','full-human','SNS']
randControl = random.sample(controltype,numExperiments)
loggerR.info("Sequence of control type experiments: 1) %s, 2) %s, 3) %s, 4) %s, 5) %s, 6) %s\n\n"%(*randControl,))


#Generate which of the nine objects fixed, graspable, breakable
loggerR.info("Generating object type distribution...")
objectTypes = ['breakable','fixed','breakable','breakable','graspable','fixed','graspable','graspable','fixed']

for i in range(0,numExperiments):
    loggerR.info("Trial %i, control type %s"%(i+1,randControl[i]))

    randTypes= random.sample(objectTypes, 9)
    randstr = ','.join(randTypes)
    loggerR.info("Random sequence: %s \n As positioned on the grid: \n\n %s %s %s \n %s %s %s \n %s %s %s\n\n----------"
                 %(randstr,*objectTypes[0:3],*objectTypes[3:6],*objectTypes[6:9]))


