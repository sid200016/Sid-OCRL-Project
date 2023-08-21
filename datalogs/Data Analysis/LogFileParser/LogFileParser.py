import re
import pandas as pd
import numpy as np
import shelve
from datetime import datetime
from pathlib import Path


def ParseLogFile(usePickle=False, datalogName='', eventLogName='', controlType='', groupType='', ParticipantNumber=0):

    usePickle = False
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

    datalogHeader = ["Date Time", "Milliseconds", "Program", "Status",
                     "x", "y", "z",
                     "closure muscle pressure (psi)", "Jaw pressure 1 (psi)", "Jaw pressure 2 (psi)",
                     "Jaw pressure 3 (psi)",
                     "A", "B", "X", "Y", "SL", "SR", "+", "Stick In", "Home", "R", "ZR", "Joy x", "Joy y",
                     "Object 1", "Object 2", "Object 3", "Object 4", "Object 5", "Object 6", "Object 7", "Object 8",
                     "Object 9"]

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
    goalPosition_mm = [[180, -195], [-180, -195], [-180, -260],
                       [-180, -260]]  # bottom left, bottom right, top right, top left, in mm
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

    pickleFileName =  Path(__file__).with_name("%s_%s_%i_pickle.pkl"%(controlType,groupType,ParticipantNumber))

    if usePickle == False or (usePickle == True and Path.exists(pickleFileName.with_suffix(pickleFileName.suffix + ".dat"))==False):

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

        if usePickle == True:
            with shelve.open(str(pickleFileName), 'n') as bk:
                for k in dir():
                    try:
                        bk[k] = globals()[k]
                    except Exception as e:
                        pass

    if usePickle == True:
        bk_restore = shelve.open(str(pickleFileName))
        for k in bk_restore:
            globals()[k] = bk_restore[k]
        bk_restore.close()

    print(ItemSequence)

    # load the datalog

    dN = pd.read_csv(datalogName, names=datalogHeader)


    ObjectStatus = {"Attempt": [], "Group Type": [], "Type of Control": [], "Proctor Classification": [],
                            "Object Type":[],"Test Type": [], "Item Number": [], "ID code":[],
                            "Datalog Name":[], "Event Log Name":[], "Participant Number":[],
                            "Proctor Time Started": [], "Proctor Time Ended": [],
                            "Grasp Time Started": [], "Grasp Time Finished": [],
                            "Grasp Time Started idx": [], "Grasp Time Finished idx": [],"Grasp Time": [],
                            "Deposit Time Started": [], "Deposit Time Finished": [],
                            "Deposit Time Started idx": [], "Deposit Time Finished idx": [],"Deposit Time": [],
                            "Object Broken Time": [], "Total Grasp Time": [],
                            "Proctor Complete Time": [],
                            "ZR grasp press time": [], "SR grasp press time": [], "Power slider grasp press time":[],
                            "R post-grasp press time": [], "ZR deposit press time": [],
                            "Power slider deposit press time": [],
                            "R post-deposit press time": [], "Power slider post-deposit press time": [],
                            "X before Grasp attempt": [], "Y before Grasp Attempt": [], "Z before grasp attempt": [],
                            "X before SR button": [], "Y before SR button": [], "Z before SR button": [],"Post Calc Analysis Classificaiton":[]}

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
    XY_dataframe = pd.DataFrame.from_dict({"DateTime":XY_datetime,"XPos":[float(x)-20 for x in (XY_a["XPos"])],"YPos":[float(y)-20 for y in (XY_a["YPos"])]}) #-20 for both x and y to compensate because the xy that is logged in Hardware_SNS_Try doesn't have this correction.

    GraspStarted = InfoDateTime

    StartSearchTime = InfoDateTime

    for k, tstamp in enumerate(ItemSequence['DateTime']):
        # look for the sequence of Attempt started to get the sequence of items.

        storageDict = {k:0 for (k,v) in ObjectStatus.items()}


        ProctorTimeStarted = datetime.min
        ProctorTimeFinished = datetime.min
        GraspTimeStart = datetime.min
        GraspTimeEnd = datetime.min
        GraspTimeStartIdx = 0
        GraspTimeEndIdx = 0
        GraspTime = 0
        DepositTimeStart = datetime.max
        DepositTimeEnd = datetime.max
        DepositTimeStartIdx = 0
        DepositTimeEndIdx = 0
        DepositTime = 0
        BrokenTime = datetime.min
        TotalGraspTime = 0
        ProctorTime = 0
        ZRgraspTime = datetime.min
        PowerGraspTime = datetime.min
        SRgraspTime = datetime.min
        RpostgraspTime = datetime.min
        ZRdepositTime = datetime.min
        PowerSliderDepositTime = datetime.min
        R_postdeposit_Time = datetime.min
        PowerSlider_postdeposit_Time = datetime.min
        X_grasp = np.NaN
        Y_grasp = np.NaN
        Z_grasp = np.NaN
        X_SR = np.NaN
        Y_SR = np.NaN
        Z_SR = np.NaN
        itemSuccess = False








        if ItemSequence["Status"][k].strip() != 'inprogress' and k<22:
            numAttempts = numAttempts + 1

            ### start from the 1st in the sequence to find times in hardwareDatalog when the gantry is wihtin 5 cm of the x-y position

            ProctorTimeStarted = ItemSequence["DateTime"][k-1]
            ProctorTimeStarted = datetime.strptime(ProctorTimeStarted.strip(),
                                                    "%Y-%m-%d %H:%M:%S,%f")  # convert to datetime object

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

            ZRidx = np.where((dN.loc[withinRange.index, 'ZR'] == 1) | (dN.loc[withinRange.index, 'R'] == 1) )[0]

            SRidx = np.where(dN.loc[withinRange.index, 'SR'] == 1)[0]

            withinRange_start = dN_datetime[withinRange.index[0]]
            withinRange_end = dN_datetime[withinRange.index[-1]]
            OpenGrasper_idx = np.where((Power_datetime >= StartSearchTime) & (Power_datetime <= EndSearchTime) & (Power_datetime>=withinRange_start) & (Power_datetime<=withinRange_end))[0] #get the index of when the grasper is within range and a power command was triggered


            gstart_idx = np.Inf

            if len(ZRidx) < 1:
                print('Could not find ZR press')



            else:
                gstart_idx = idx[ZRidx[0]]
                print('Found ZR press')
                ZRgraspTime = dN_datetime[gstart_idx]
                X_grasp = dN.loc[gstart_idx, "x"]
                Y_grasp = dN.loc[gstart_idx, "y"]
                Z_grasp = dN.loc[gstart_idx, "z"]

            if len(OpenGrasper_idx)<1:
                print('Could not find grasper close during start of grasp')

            else:
                PowerGraspTime = Power_datetime[OpenGrasper_idx[0]]
                print("Power command for grasp time start: " + datetime.strftime(PowerGraspTime, "%H-%M-%S,%f"))
                if controlType != "SNS" and gstart_idx == np.Inf: #only if you can't find the ZR press do you use the power grasp time because it is is more unreliable
                    nearestIdx = np.where(dN_datetime<=PowerGraspTime)[0][-1]
                    gstart_idx = min(nearestIdx, gstart_idx)


            if controlType == "SNS": #for SNS only

                if len(SRidx) < 1:
                    print('Could not find SR press')

                else:
                    SR_start_idx = idx[SRidx[0]]
                    gstart_idx = min(SR_start_idx, gstart_idx)
                    print('Found SR press')
                    SRgraspTime = dN_datetime[SR_start_idx]
                    X_SR = dN.loc[SR_start_idx,"x"]
                    Y_SR = dN.loc[SR_start_idx, "y"]
                    Z_SR = dN.loc[SR_start_idx, "z"]

            if gstart_idx == np.Inf:
                print("Not able to find a good start index")
                continue

            else:

                GraspTimeStart = dN_datetime[gstart_idx]
                GraspTimeStartIdx = gstart_idx
                print("Grasp Time Start: " + datetime.strftime(GraspTimeStart, "%H-%M-%S,%f"))

            ### See if item failed grasp or broken item.  This is end time of grasp and mark as failed or broken.  Don't look for deposit times
            # Check broken first if there is a date time within the idx

            Broken_idx = np.where((Broken_datetime >= GraspTimeStart) & (Broken_datetime <= max(dN_datetime[idx])))[
                0]
            itemBroke = False
            itemFailedGrasp = False
            if len(Broken_idx) < 1:
                print('Could not find broken item')


            else:
                print("# of broken items found in this run: %i" % (len(Broken_idx)))
                Broken_idxv = Broken_idx[0]
                BrokenTime = Broken_datetime[Broken_idxv]
                GraspTimeEnd = BrokenTime
                print("Broken item:%s, %s" % (BrokenItem_a["Item"][Broken_idxv], BrokenItem_a["Threshold"][Broken_idxv]))
                print("Broken item status sanity check: %i" % (itemNumber == BrokenItem_a["Item"][Broken_idxv]))
                itemBroke = True


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
                    GraspTimeEndIdx = FailedGrasp_idx2
                    itemFailedGrasp = True

                ### See if item was successfully picked up (Reset SNS doesn't have associated failed grasp or broken item).  This means that the object was was successfully picked up. Log this as the end time of the grasp

                if itemBroke == False and itemFailedGrasp == False:  # object was not broken

                    SuccessGrasp_idx = \
                        np.where((ResetSNS_datetime >= GraspTimeStart) & (ResetSNS_datetime <= max(dN_datetime[idx])))[
                            0]

                    if len(SuccessGrasp_idx) < 1:
                        print('Could not find a successful SNS pickup')


                    else:
                        SuccessGrasp_idx = SuccessGrasp_idx[0]
                        GraspTimeEnd = ResetSNS_datetime[SuccessGrasp_idx]
                        GraspTimeEndIdx = np.where(dN_datetime <= GraspTimeEnd)[0][-1] #get closest position
                        print("Successfully picked up")
                        itemSuccess = True



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
                    dN_red = dN.loc[idx_w,'R']
                    Ridx = dN_red.index[np.where(dN_red == 1)[0]]  # find the time in the datalog that is closest to this command and that had a R button press
                    if len(Ridx) < 1:
                        print("No R press found during lift after grasp with no goal presses")
                        GraspTimeEnd = ProctorTimeFinished

                    else:
                        GraspTimeEnd = dN_datetime[Ridx[-1]]
                        GraspTimeEndIdx = Ridx[-1]
                        print("Grasp Time End %s" % (datetime.strftime(GraspTimeEnd, "%H-%M-%S,%f")))


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
                        GraspTimeEndIdx = Ridx[-1]
                        print("Grasp Time End %s" % (datetime.strftime(GraspTimeEnd, "%H-%M-%S,%f")))
                    itemSuccess = True

                    # validate this with the proctor classified success. If the proctor classified this as a failure and this seems to be a success, then maybe they pressed the goal location in error.







                #for failure: search for last R before fail time was logged

            print("Grasp Time End %s" % (datetime.strftime(GraspTimeEnd, "%H-%M-%S,%f")))


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
                    ZRdepositTime = ZR_StartTime
                    print('Found ZR press for deposit')

                    DepositTimeStartIdx = idx[ZR_dep_idx[0]] #temporary, should update so it is the closest index in the datalog to the DepositTimeStart

                # check to see when they start to open the grasper
                OpenGrasper_idx = np.where((Power_datetime >= GraspTimeEnd) & (Power_datetime <= EndSearchTime))[0]

                Grasper_StartTime = datetime.max
                if len(OpenGrasper_idx) < 1:
                    print('Could not find Grasper Open during Deposit')

                else:
                    Grasper_StartTime = Power_datetime[OpenGrasper_idx[0]]
                    PowerSliderDepositTime = Grasper_StartTime
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
                    R_postdeposit_Time = LiftEndTime
                    DepositTimeEndIdx = dend_idx

                # check for power press

                OpenGrasper_idx = np.where((Power_datetime >= DepositTimeStart) & (Power_datetime <= EndSearchTime))[0]

                Grasper_EndTime = datetime.min
                if len(OpenGrasper_idx) < 1:
                    print('Could not find Grasper modulation during end of deposit')


                else:
                    #find which happens last
                    Grasper_EndTime = Power_datetime[OpenGrasper_idx[-1]]
                    print('Found Grasper open during deposit')
                    PowerSlider_postdeposit_Time = Grasper_EndTime



                DepositTimeEnd = max(Grasper_EndTime, LiftEndTime)
                print("Deposit Time End: " + datetime.strftime(DepositTimeEnd, "%H-%M-%S,%f"))



            storageDict["Attempt"] = numAttempts
            storageDict["Group Type"] = groupType
            storageDict["Type of Control"] = controlType
            storageDict["Proctor Classification"] = ItemSequence["Status"][k]
            storageDict["Object Type"] = GraspTypes[int(ItemSequence["ItemNumber"][k])]
            storageDict["Test Type"] = ItemSequence["testType"][k]
            storageDict["Item Number"] = int(ItemSequence["ItemNumber"][k]) + 1 #so it is 1 through 9
            storageDict["ID code"] = IDcode
            storageDict["Participant Number"] = ParticipantNumber
            storageDict["Datalog Name"] = datalogName
            storageDict["Event Log Name"] = eventLogName
            storageDict["Proctor Time Started"] = ProctorTimeStarted
            storageDict["Proctor Time Ended"] = ProctorTimeFinished
            storageDict["Proctor Complete Time"] = ProctorTimeFinished-ProctorTimeStarted
            storageDict["Grasp Time Started"] = GraspTimeStart
            storageDict["Grasp Time Finished"] = GraspTimeEnd
            storageDict["Grasp Time Started idx"] = GraspTimeStartIdx
            storageDict["Grasp Time Finished idx"] = GraspTimeEndIdx
            storageDict["Grasp Time"] = (GraspTimeEnd-GraspTimeStart).total_seconds()
            storageDict["Deposit Time Started"] = DepositTimeStart
            storageDict["Deposit Time Finished"] = DepositTimeEnd
            storageDict["Deposit Time Started idx"] = DepositTimeStartIdx
            storageDict["Deposit Time Finished idx"] = DepositTimeEndIdx
            storageDict["Deposit Time"] = (DepositTimeEnd-DepositTimeStart).total_seconds()
            storageDict["Object Broken Time"] = BrokenTime
            storageDict["Total Grasp Time"] = storageDict["Grasp Time"] + storageDict["Deposit Time"]
            storageDict["ZR grasp press time"] = ZRgraspTime
            storageDict["SR grasp press time"] = SRgraspTime
            storageDict["Power slider grasp press time"] = PowerGraspTime
            storageDict["R post-grasp press time"] = RpostgraspTime
            storageDict["ZR deposit press time"] = ZRdepositTime
            storageDict["Power slider deposit press time"] = PowerSlider_postdeposit_Time
            storageDict["R post-deposit press time"] = R_postdeposit_Time
            storageDict["Power slider post-deposit press time"] = PowerSlider_postdeposit_Time
            storageDict["X before Grasp attempt"] =X_grasp
            storageDict["Y before Grasp Attempt"] = Y_grasp
            storageDict["Z before grasp attempt"] = Z_grasp
            storageDict["X before SR button"] = X_SR
            storageDict["Y before SR button"] = Y_SR
            storageDict["Z before SR button"] = Z_SR
            storageDict["Post Calc Analysis Classificaiton"] = itemSuccess

            for k,v in storageDict.items():
                ObjectStatus[k].append(v)



            StartSearchTime = ProctorTimeFinished



    ObjectDF = pd.DataFrame.from_dict(ObjectStatus)
    return(ObjectDF)





















