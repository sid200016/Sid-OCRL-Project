import re
import pandas as pd
import numpy as np
import pickle
import mmap
import shelve
from datetime import datetime

from pathlib import Path

usePickle = False

datalogName = "..\\HardwareDatalog_17_08_2023_10_37_29.csv"
#datalogName = "..\\HardwareDatalog_17_08_2023_11_15_43.csv"
datalogHeader = ["Date Time", "Milliseconds", "Program", "Status",
                 "x", "y", "z",
                 "closure muscle pressure (psi)", "Jaw pressure 1 (psi)", "Jaw pressure 2 (psi)",
                 "Jaw pressure 3 (psi)",
                 "A", "B", "X", "Y", "SL", "SR", "+", "Stick In", "Home", "R", "ZR", "Joy x", "Joy y",
                 "Object 1", "Object 2", "Object 3", "Object 4", "Object 5", "Object 6", "Object 7", "Object 8",
                 "Object 9"]

eventLogName = "..\\Hardware_17_08_2023_10_37_29.txt"
#eventLogName = "..\\Hardware_17_08_2023_11_15_43.txt"

controlType = "SNS"
#controlType = "Manual"

#### Strings for regex for parsing the event log
programLoopString = "\s*(?P<DateTime>.*).*?- __main__.*?(?P<ProgramLoop>ProgramLoop)"
reProgramLoop = re.compile(programLoopString)

InfoString = "\s*(?P<DateTime>.*).*?- __main__.*?ID code (?P<IDcode>.*), age:(?P<Age>.*), gender:(?P<Gender>.*), types:\s*\[(?P<GraspTypes>.*)\], test:(?P<TestType>.*), num_attempts:(?P<NumAttempts>.*).*"  # https://regex101.com/r/yM9u7X/1
reInfo = re.compile(InfoString)  # replace >> and << with the start and end indicator pos
ReceivedXYcommand = "\s*(?P<DateTime>.*).*?- __main__.*?(?P<GantryStart>Received gantry position command).*"
reReceivedXY = re.compile(ReceivedXYcommand)
XYcommand = "\s*(?P<DateTime>.*).*?- __main__.*?\{.*'x'.*:(?P<XPos>.*).*,.*'y'.*:(?P<YPos>.*)}"  # https://regex101.com/r/kcb9FR/1
reXY = re.compile(XYcommand)

FailedGrasp = "\s*(?P<DateTime>.*).*?- __main__.*?(?P<FailedGrasp>Failed grasp, exceeded number of attempts).*"
reFailedGrasp = re.compile(FailedGrasp)
ResetSNS = "\s*(?P<DateTime>.*).*?- __main__.*?(?P<ResetSNS>Reset the SNS controller after lift after grasp complete).*"
reResetSNS = re.compile(ResetSNS)

BrokenItem = "\s*(?P<DateTime>.*).*?- __main__.*?Force exceeded on item(?P<Item>.*)\..*Threshold(?P<Threshold>.*),.*rumble value(?P<RumbleVal>.*)"
reBrokenItem = re.compile(BrokenItem)

ReceivedGrasper = "\s*(?P<DateTime>.*).*?- __main__.*?(?P<ReceivedSoftGrasper>Received soft grasper power command)"
reReceivedGrasper = re.compile(ReceivedGrasper)
PowerCommand = "\s*(?P<DateTime>.*).*?- __main__.*?Change in radius in mm:(?P<ChangeInRadius_mm>.*)"
rePowerCommand = re.compile(PowerCommand)

AttemptStarted = "\s*(?P<DateTime>.*).*?- __main__.*?Item event: item (?P<ItemNumber>.*).*is(?P<Status>.*)\..*Attempt.*\[\{?(?P<AttemptSequence>.*)\}?\].*,.*test(?P<testType>.*)"
reAttemptStarted = re.compile(AttemptStarted)

#### Get the object locations and goal positions in mm
goalPosition_mm = [[180, -195], [-180, -195], [-180, -255],
                   [-180, -255]]  # bottom left, bottom right, top right, top left, in mm
ObjectPositions_mm = [[165, -165],
                      [0, -165],
                      [-165, -165],
                      [165, 0],
                      [0, 0],
                      [-165, 0],
                      [165, 165],
                      [0, 165],
                      [-165, 165]]  # objects 1 to 9.  1 is top left, 9 is bottom right.  Relative to center, in mm.

ToleranceRadius_mm = 40

if usePickle == False:

    # dN = pd.read_csv(datalogName,names= datalogHeader)

    # algorithm for SNS:

    Info_Time = ""
    IDcode = ""
    Age = ""
    GraspTypes_str = ""
    GraspTypes = ""
    Gender = ""
    NumAttempts = ""

    ItemSequence = {"DateTime": [], "ItemNumber": [], "Status": [], "AttemptSequence": [], "testType": []}
    index_StartStop = {"start": [], "stop": []}
    GraspTypes = []

    XY_a = {"DateTime": [], "XPos": [], "YPos": []}

    FailedGrasp_a = {"DateTime": []}

    ResetSNS_a = {"DateTime": []}

    BrokenItem_a = {"DateTime": [], "Item": [], "Threshold": [], "RumbleVal": []}

    Power_a = {"DateTime": [], "ChangeInRadius_mm": []}

    AttemptStarted_a = {"DateTime": [], "Item Number": [], "Status": [], "AttemptSequence": [], "testType": []}

    with open(eventLogName, 'r') as f:

        for line in f.readlines():
            if re.match(reProgramLoop, line) is None:
                # look for Info string to get the types of objects in each location
                InfoR = reInfo.match(line)

                if InfoR is not None:
                    Info_Time = InfoR.group('DateTime')
                    IDcode = InfoR.group('IDcode')
                    Age = InfoR.group('Age')
                    GraspTypes_str = InfoR.group('GraspTypes')
                    GraspTypes = [x.lstrip('\'').rstrip('\'') for x in GraspTypes_str.split(', ')]
                    Gender = InfoR.group('TestType')
                    NumAttempts = InfoR.group('NumAttempts')
                    print('Found the objects')

                    continue

                # Attempts

                AttemptStarted = "\s*(?P<DateTime>.*).*?- __main__.*?Item event: item (?P<ItemNumber>.*).*is(?P<Status>.*)\..*Attempt.*\[\{(?P<AttemptSequence>.*)\}\].*,.*test(?P<testType>.*)"

                ItemR = reAttemptStarted.match(line)

                if ItemR is not None:
                    ItemSequence["DateTime"].append(ItemR.group('DateTime'))
                    ItemSequence["ItemNumber"].append(ItemR.group('ItemNumber'))
                    ItemSequence["Status"].append(ItemR.group('Status'))
                    ItemSequence["AttemptSequence"].append(ItemR.group('AttemptSequence'))
                    ItemSequence["testType"].append(ItemR.group('testType'))
                    print("Found item")

                    continue

                # XY Command

                XYR = reXY.match(line)

                if XYR is not None:
                    XY_a["DateTime"].append(XYR.group('DateTime'))
                    XY_a["XPos"].append(XYR.group('XPos'))
                    XY_a["YPos"].append(XYR.group('YPos'))

                    continue

                # Failed Grasp

                FailedGraspR = reFailedGrasp.match(line)

                if FailedGraspR is not None:
                    FailedGrasp_a["DateTime"].append(FailedGraspR.group('DateTime'))

                    continue

                # ResetSNS

                ResetSNSR = reResetSNS.match(line)

                if ResetSNSR is not None:
                    ResetSNS_a["DateTime"].append(ResetSNSR.group('DateTime'))

                    continue

                # BrokenItem

                BrokenItemR = reBrokenItem.match(line)

                BrokenItem_a = {"DateTime": [], "Item": [], "Threshold": [], "RumbleVal": []}

                if BrokenItemR is not None:
                    BrokenItem_a["DateTime"].append(BrokenItemR.group('DateTime'))
                    BrokenItem_a["Item"].append(BrokenItemR.group('Item'))
                    BrokenItem_a["Threshold"].append(BrokenItemR.group('Threshold'))
                    BrokenItem_a["RumbleVal"].append(BrokenItemR.group('RumbleVal'))
                    continue

                # PowerCommand

                PowerCommandR = rePowerCommand.match(line)

                if PowerCommandR is not None:
                    Power_a["DateTime"].append(PowerCommandR.group('DateTime'))
                    Power_a["ChangeInRadius_mm"].append(PowerCommandR.group('ChangeInRadius_mm'))
                    continue

                # AttemptStarted

                AttemptStartedR = reAttemptStarted.match(line)

                if AttemptStartedR is not None:
                    AttemptStarted_a["Item Number"].append(AttemptStartedR.group('Item Number'))
                    AttemptStarted_a["Status"].append(AttemptStartedR.group('Status'))
                    AttemptStarted_a["AttemptSequence"].append(AttemptStartedR.group('AttemptSequence'))
                    AttemptStarted_a["testType"].append(AttemptStartedR.group('testType'))
                    continue

    with shelve.open('./Data.pkl', 'n') as bk:
        for k in dir():
            try:
                bk[k] = globals()[k]
            except Exception:
                pass

if usePickle == True:
    bk_restore = shelve.open('./Data.pkl')
    for k in bk_restore:
        globals()[k] = bk_restore[k]
    bk_restore.close()

print(ItemSequence)

# load the datalog

dN = pd.read_csv(datalogName, names=datalogHeader)

# look for the sequence of Attempt started to get the sequence of items.
ObjectStatus = {"Attempt": [], "Type of Control": [], "Grasp Time started": [], "Grasp Time Finished": [],
                "Deposit Time Started": [], "Deposit Time Finished": [], "Object Broken Time": [],
                "Proctor Time Started": [], "Proctor Time Finished": []}
numAttempts = 0

InfoDateTime = datetime.strptime(Info_Time.strip(), "%Y-%m-%d %H:%M:%S,%f")

dN_datetime = np.array([datetime.strptime(x+","+str(dN["Milliseconds"][i]),"%Y-%m-%d %H:%M:%S,%f")for i,x in enumerate(dN["Date Time"])]) #convert to datetime object with microseconds
#dN_datetime = np.array([x.combine(InfoDateTime.date(),x.time()) for x in dN_datetime])

FailedGrasp_datetime = np.array(
    [datetime.strptime(x.strip(), "%Y-%m-%d %H:%M:%S,%f") for x in FailedGrasp_a["DateTime"]])
Broken_datetime = np.array([datetime.strptime(x.strip(), "%Y-%m-%d %H:%M:%S,%f") for x in BrokenItem_a["DateTime"]])
ResetSNS_datetime = np.array([datetime.strptime(x.strip(), "%Y-%m-%d %H:%M:%S,%f") for x in ResetSNS_a["DateTime"]])
Power_datetime = np.array([datetime.strptime(x.strip(), "%Y-%m-%d %H:%M:%S,%f") for x in Power_a["DateTime"]])
XY_datetime = np.array([datetime.strptime(x.strip(), "%Y-%m-%d %H:%M:%S,%f") for x in XY_a["DateTime"]])
XY_dataframe = pd.DataFrame.from_dict({"DateTime":XY_datetime,"XPos":[float(x) for x in (XY_a["XPos"])],"YPos":[float(y) for y in (XY_a["YPos"])]})

GraspStarted = InfoDateTime

StartSearchTime = InfoDateTime

for k, tstamp in enumerate(ItemSequence['DateTime']):

    if ItemSequence["Status"][k].strip() != 'inprogress':
        numAttempts = numAttempts + 1

        ### start from the 1st in the sequence to find times in hardwareDatalog when the gantry is wihtin 5 cm of the x-y position

        ProctorTimeFinished = ItemSequence["DateTime"][k]
        ProctorTimeFinished = datetime.strptime(ProctorTimeFinished.strip(),
                                                "%Y-%m-%d %H:%M:%S,%f")  # convert to datetime object
        EndSearchTime = ProctorTimeFinished
        itemNumber = int(ItemSequence["ItemNumber"][k])
        print(" \n\n\n------------\nItem: %i" % itemNumber)

        # Find indices between the start of the current attempt and the start of the next attempt

        idx = np.where((dN_datetime >= StartSearchTime) & (dN_datetime <= EndSearchTime))[0]

        # within these indices, find the ones where the distance is within ToleranceRadius_mm of the target
        objectPosition = ObjectPositions_mm[itemNumber]  # x, y position of object
        withinRange = (((dN.loc[idx, 'x'] - objectPosition[0]) ** 2 + (
                    dN.loc[idx, 'y'] - objectPosition[1]) ** 2) ** 0.5 <= ToleranceRadius_mm)

        ### For SNS control, see when either ZR first pressed in those times, or SR.  This is start time of grasp

        # check ZR first:

        ZRidx = np.where(dN.loc[withinRange.index, 'ZR'] == 1)[0]

        SRidx = np.where(dN.loc[withinRange.index, 'SR'] == 1)[0]

        gstart_idx = np.Inf

        if len(ZRidx) < 1:
            print('Could not find ZR press')



        else:
            gstart_idx = idx[ZRidx[0]]
            print('Found ZR press')


        if controlType == "SNS": #for SNS only

            if len(SRidx) < 1:
                print('Could not find SR press')

            else:
                gstart_idx = min(idx[SRidx[0]], gstart_idx)
                print('Found SR press')

        GraspTimeStart = dN_datetime[gstart_idx]
        print("Grasp Time Start: " + datetime.strftime(GraspTimeStart, "%H-%M-%S,%f"))

        ### See if item failed grasp or broken item.  This is end time of grasp and mark as failed or broken.  Don't look for deposit times
        GraspTimeEnd = 0
        # Check broken first if there is a date time within the idx

        Broken_idx = np.where((Broken_datetime >= min(dN_datetime[idx])) & (Broken_datetime <= max(dN_datetime[idx])))[
            0]
        itemBroke = False
        itemFailedGrasp = False
        if len(Broken_idx) < 1:
            print('Could not find broken item')


        else:
            print("# of broken items found in this run: %i" % (len(Broken_idx)))
            Broken_idx = Broken_idx[0]
            GraspTimeEnd = Broken_datetime[Broken_idx]
            print("Broken item:%i, %f" % (BrokenItem_a["Item"][Broken_idx], BrokenItem_a["Threshold"][Broken_idx]))
            print("Broken item status sanity check: %i" % (itemNumber == BrokenItem_a["Item"][Broken_idx]))
            itemBroke == True

        itemSuccess = False
        # Check failed item
        if controlType == "SNS": #only SNS has this failure mode



            FailedGrasp_idx = \
            np.where((FailedGrasp_datetime >= min(dN_datetime[idx])) & (FailedGrasp_datetime <= max(dN_datetime[idx])))[0]

            if len(FailedGrasp_idx) < 1:
                print('Could not find Failed Grasp')


            else:
                print("# of failed grasps: %i" % (len(FailedGrasp_idx)))
                FailedGrasp_idx2 = FailedGrasp_idx[0]
                GraspTimeEnd = FailedGrasp_datetime[FailedGrasp_idx2]
                itemFailedGrasp = True

            ### See if item was successfully picked up (Reset SNS doesn't have associated failed grasp or broken item).  This means that the object was was successfully picked up. Log this as the end time of the grasp

            if len(Broken_idx) < 1 and itemFailedGrasp == False:  # object was not broken

                SuccessGrasp_idx = \
                    np.where((ResetSNS_datetime >= GraspTimeStart) & (ResetSNS_datetime <= max(dN_datetime[idx])))[
                        0]

                if len(SuccessGrasp_idx) < 1:
                    print('Could not find a successful SNS pickup')


                else:
                    SuccessGrasp_idx = SuccessGrasp_idx[0]
                    GraspTimeEnd = ResetSNS_datetime[SuccessGrasp_idx]
                    print("Successfully picked up")
                    itemSuccess = True

            print("Grasp Time End %s" % (datetime.strftime(GraspTimeEnd, "%H-%M-%S,%f")))

        elif controlType == "Manual" and itemBroke == False:

            #for success: search for last R before xyz move to goal location
            idx = np.where((dN_datetime >= GraspTimeStart) & (dN_datetime <= EndSearchTime))[0] #update idx so it is looking after start time

            withinRange = withinRange[idx] #find the positions which are within the tolerance of the goal, updated with the start grasp time
            withinRange = dN.loc[withinRange.index, 'y'] >= goalPosition_mm[0][1] #update the range to exclude the goal area
            idx_w = withinRange.index
            #find xyz moves that happened in the time defined by withinRange whose target was the goal location

            XY_moves = XY_dataframe.loc[(XY_dataframe["DateTime"] >= min(dN_datetime[idx_w])) & (XY_dataframe["DateTime"] <= max(dN_datetime[idx_w])),:] #these are the XY commanded moves that were commanded in this attempts timeframe

            withinGoal = (XY_moves["XPos"] <= goalPosition_mm[0][0]) & (XY_moves["XPos"]>= goalPosition_mm[3][0]) & (XY_moves["YPos"] <= goalPosition_mm[0][1]) & (XY_moves["YPos"] >= goalPosition_mm[3][1]) # check to see if within goal.  recall that -ve y is further away from the user, and -ve x is to the right when facing the gantry.

            if np.all(withinGoal == False):
                print("No target presses found to goal.")
                itemFailedGrasp = True
                itemSuccess = False

                #validate this with the proctor classified success.  If the proctor says success and you couldn't find a within goal location, then the proctor may have pressed too early.


            else:
                MostRecentMoveToGoal = XY_moves["DateTime"][withinGoal.index[np.where(withinGoal == True)][-1]] #get time of the most recent move to goal
                MR_idx = dN_datetime[idx_w]<=MostRecentMoveToGoal
                dN_red=dN.loc[idx_w,'R']
                Ridx = dN_red.index[np.where(dN_red[MR_idx] == 1)[0]]#find the time in the datalog that is closest to this command and that had a R button press
                if len(Ridx)<1:
                    print("No R press found during lift after grasp")
                    GraspTimeEnd = MostRecentMoveToGoal

                else:
                    GraspTimeEnd = dN_datetime[Ridx[-1]]
                    print("Grasp Time End %s" % (datetime.strftime(GraspTimeEnd, "%H-%M-%S,%f")))
                itemSuccess = True

                # validate this with the proctor classified success. If the proctor classified this as a failure and this seems to be a success, then maybe they pressed the goal location in error.







            #for failure: search for last R before fail time was logged






        ### Look for eventLog XYcommand which is within the goal location and closest to the successfully picked up.  Look in datalog to see from this time until the time when success marked: when the first ZR is marked.  This is the start time of the deposit.
        # find the ZR within the goal location. This is the start time of the deposit
        if itemSuccess == True:
            idx = np.where((dN_datetime >= GraspTimeEnd) & (dN_datetime <= EndSearchTime))[0]

            withinGoal = (dN.loc[idx, 'x'] <= goalPosition_mm[0][0]) & (dN.loc[idx, 'x'] >= goalPosition_mm[3][0]) & (
                        dN.loc[idx, 'y'] <= goalPosition_mm[0][1]) & (dN.loc[idx, 'y'] >= goalPosition_mm[3][
                1])  # check to see if within goal.  recall that -ve y is further away from the user, and -ve x is to the right when facing the gantry.

            ### For SNS control, see when either ZR first pressed in those times, or SR.  This is start time of grasp
            dstart_idx = np.Inf
            # check ZR first:

            ZR_dep_idx = np.where(dN.loc[withinGoal.index, 'ZR'] == 1)[0]

            ZR_StartTime = datetime.max
            if len(ZR_dep_idx) < 1:
                print('Could not find ZR press for deposit')



            else:
                ZR_StartTime = dN_datetime[idx[ZR_dep_idx[0]]]
                print('Found ZR press for deposit')

            # check to see when they start to open the grasper
            OpenGrasper_idx = np.where((Power_datetime >= GraspTimeEnd) & (Power_datetime <= EndSearchTime))[0]

            Grasper_StartTime = datetime.max
            if len(OpenGrasper_idx) < 1:
                print('Could not find Grasper Open during Deposit')

            else:
                Grasper_StartTime = Power_datetime[OpenGrasper_idx[0]]
                print('Found Grasper open during deposit')

            DepositTimeStart = min(Grasper_StartTime,ZR_StartTime)
            print("Deposit Time Start: " + datetime.strftime(DepositTimeStart, "%H-%M-%S,%f"))

            ### when the last R press that happened within the goal location, or the last open command to the grasper, whichever came last.  This is the deposit finish time
            # start with R press
            dend_idx = np.Inf
            idx = np.where((dN_datetime >= DepositTimeStart) & (dN_datetime <= EndSearchTime))[0]
            R_dep_idx = np.where(dN.loc[idx, 'R'] == 1)[0]
            LiftEndTime = datetime.min
            if len(R_dep_idx) < 1:
                print('Could not find Grasper Raising with R button after deposit in goal')


            else:
                dend_idx = idx[R_dep_idx[-1]]
                print('Found R press after touch down in goal')
                LiftEndTime = dN_datetime[dend_idx]

            # check for power press

            OpenGrasper_idx = np.where((Power_datetime >= DepositTimeStart) & (Power_datetime <= EndSearchTime))[0]

            Grasper_EndTime = datetime.min
            if len(OpenGrasper_idx) < 1:
                print('Could not find Grasper modulation during end of deposit')


            else:
                #find which happens last
                Grasper_EndTime = Power_datetime[OpenGrasper_idx[-1]]
                print('Found Grasper open during deposit')


            DepositTimeEnd = max(Grasper_EndTime, LiftEndTime)
            print("Deposit Time End: " + datetime.strftime(DepositTimeEnd, "%H-%M-%S,%f"))

        ObjectStatus["Attempt"].append(numAttempts)
        StartSearchTime = ProctorTimeFinished

























