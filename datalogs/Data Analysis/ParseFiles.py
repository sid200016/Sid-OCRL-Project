from LogFileParser.LogFileParser import ParseLogFile
import pandas as pd
import shelve
from pathlib import Path


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



Dataframes = []
for i,x in enumerate(FilesToParse):

    pickleFileName = Path(__file__).with_name("%s_%s_%i_pickle.pkl" % (x["controlType"], x["groupType"], x["ParticipantNumber"]))
    if x["usePickle"] == False or (x["usePickle"] == True and Path.exists(pickleFileName.with_suffix(pickleFileName.suffix + ".dat"))==False):
        DFx = ParseLogFile(**x)
        with shelve.open(str(pickleFileName), 'n') as bk:
            bk["DFx"] = DFx

    if x["usePickle"] == True:
        bk_restore = shelve.open(str(pickleFileName))
        for k in bk_restore:
            globals()["DFx"] = bk_restore[k]
        bk_restore.close()

    Dataframes.append(DFx)





DF = pd.concat(Dataframes)
DF.to_csv("UserExperimentsData.csv")

#shelve the dataframe
with shelve.open(str("UserExperimentsData.pkl"), 'n') as bk:
    bk["DF"] = DF

#Get summary DF
def calcSuccessFunc(df):
    success = (df["Proctor Classification"]== " success").sum() /3
    return success


summaryDF = DF.groupby(["Group Type","Type of Control","Participant Number","Object Type"]).apply(calcSuccessFunc)
summaryDF.to_csv("SummarySuccess.csv")
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