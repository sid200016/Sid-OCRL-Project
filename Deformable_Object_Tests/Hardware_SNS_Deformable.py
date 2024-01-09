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

# Boot System

# Say system has booted, do you want to enter calibration routine (Y/N)
s = input("")

# If N, then proceed to main script.  If Y, then proceed to calibration routine.

# Give prompt to enter "F" to finish calibration routine.

# Save offset value

# ask if want to start experiment (Y/N)

# if yes, ask for additional offset

# Prompt to press Y to return home

# After return home, prompt to hit "SR" on the joystick to start experiment.
