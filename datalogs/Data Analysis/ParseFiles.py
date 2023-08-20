from LogFileParser.LogFileParser import ParseLogFile
import pandas as pd


datalogName = "..\\HardwareDatalog_17_08_2023_10_37_29.csv"
eventLogName = "..\\Hardware_17_08_2023_10_37_29.txt"
controlType = "SNS"
groupType = "SNS_first"
ParticipantNumber = 2
ParseLogFile(False, datalogName, eventLogName, controlType, groupType, ParticipantNumber)

# usePickle = False
#
# #datalogName = "..\\HardwareDatalog_17_08_2023_10_37_29.csv"
# datalogName = "..\\HardwareDatalog_17_08_2023_11_15_43.csv"
#
#
# #eventLogName = "..\\Hardware_17_08_2023_10_37_29.txt"
# eventLogName = "..\\Hardware_17_08_2023_11_15_43.txt"
#
# #controlType = "SNS"
# controlType = "Manual"
#
# groupType = "SNS_first"
# ParticipantNumber = 2