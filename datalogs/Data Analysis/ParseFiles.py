from LogFileParser.LogFileParser import ParseLogFile
import pandas as pd

FilesToParse = [{"datalogName": "..\\HardwareDatalog_17_08_2023_08_52_12.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_08_52_12.txt" ,
                 "controlType":"SNS",
                "groupType":"SNS_first",
                "ParticipantNumber":1,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_09_21_00.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_09_21_00.txt" ,
                 "controlType":"Manual",
                "groupType":"SNS_first",
                "ParticipantNumber":1,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_10_37_29.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_10_37_29.txt" ,
                 "controlType":"SNS",
                "groupType":"SNS_first",
                "ParticipantNumber":2,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_11_15_43.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_11_15_43.txt" ,
                 "controlType":"Manual",
                "groupType":"SNS_first",
                "ParticipantNumber":2,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_12_41_45.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_12_41_45.txt" ,
                 "controlType":"Manual",
                "groupType":"Manual_first",
                "ParticipantNumber":3,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_13_32_36.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_13_32_36.txt" ,
                 "controlType":"SNS",
                "groupType":"Manual_first",
                "ParticipantNumber":3,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_14_42_23.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_14_42_23.txt" ,
                 "controlType":"Manual",
                "groupType":"Manual_first",
                "ParticipantNumber":4,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_15_42_22.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_15_42_22.txt" ,
                 "controlType":"SNS",
                "groupType":"Manual_first",
                "ParticipantNumber":4,
                "usePickle": False},
                ]


FilesToParse = [

                {"datalogName": "..\\HardwareDatalog_17_08_2023_12_41_45.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_12_41_45.txt" ,
                 "controlType":"Manual",
                "groupType":"Manual_first",
                "ParticipantNumber":3,
                "usePickle": False},

                {"datalogName": "..\\HardwareDatalog_17_08_2023_13_32_36.csv",
                 "eventLogName": "..\\Hardware_17_08_2023_13_32_36.txt" ,
                 "controlType":"SNS",
                "groupType":"Manual_first",
                "ParticipantNumber":3,
                "usePickle": False},
                ]

Dataframes = []
for i,x in enumerate(FilesToParse):
    Dataframes.append(ParseLogFile(**x))


DF = pd.concat(Dataframes)
DF.to_csv("UserExperimentsData.csv")
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