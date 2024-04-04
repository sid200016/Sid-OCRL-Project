import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objs as go
'''
Pseudocode

TODO: Produce Table of:
Attempt Time  Success   Failure Number_of_Attempts  Final_Attempt_average_radius Final_Attempt_average_pressure Final_Attempt_max_radius Final_Attempt_max_pressure Indices_of_attempts Indices_of_trial Max_Pressure_Per_Attempt

To define at attempt, look for where it goes from GO_HOME to USE_SNS and then from USE_SNS to NORMAL

'''


class AnalyzeDatalog():


    def __init__(self,fname = 'WithModulation_resaved.csv'):

        self.DF = pd.read_csv(fname)
        # self.DF.to_excel("WithModulation_resaved.xlsx")

        self.Episode_IDX = {"Date Classifier":[], "Start IDX":[], "End IDX":[]}

        self.Summary = {}
        self.Summary["Date Classifier"] = []
        self.Summary["Trial Number"] = []
        self.Summary["Start IDX"] = []
        self.Summary["End IDX"] = []
        self.Summary["Attempt start IDX"] = []
        self.Summary["Attempt end IDX"] = []
        self.Summary["Time Start"] = []
        self.Summary["Time End"] = []
        self.Summary["Success"] = []
        self.Summary["Number of Attempts"] = []
        self.Summary["Attempt Number"] = []
        self.Summary["Relative Attempt"] = []
        self.Summary["Transition Pressure psi"] = []
        self.Summary["Max Pressure during lift psi"] = []
        self.Summary["Max Pressure during grasp"] = []
        self.Summary["Transition Radius mm"] = []
        self.Summary["Max Radius during lift mm"] = []
        self.Summary["Max Radius during grasp mm"] = []

    def find_episodes(self):

        gd = self.DF.Mode[0:-1]
        gd2 = self.DF.Mode[1:]
        start_idx = list(np.where((np.array(gd)=="GO_HOME")&(np.array(gd2)=="USE_SNS"))[0] +1)

        end_idx = list(np.where((np.array(gd)=="USE_SNS")&(np.array(gd2)=="NORMAL"))[0])

        self.Episode_IDX["Start IDX"] = start_idx
        self.Episode_IDX["End IDX"] = end_idx
        self.Episode_IDX["Date Classifier"] = [self.DF.at[x,"Time Stamp"]+","+str(self.DF.at[x,"Seconds"]) for x in start_idx]

        list_of_indices = np.concatenate([np.array(range(start_idx[k], end_idx[k] + 1)) for k, v in enumerate(start_idx)])
        episode_labels = np.concatenate([np.ones(end_idx[k]-start_idx[k] + 1)*k for k, v in enumerate(start_idx)])

        self.DF["Valid Episode"] = [x in list_of_indices for x in self.DF.index]
        self.DF["Episode Label"] = [episode_labels[np.where(x ==list_of_indices)][0] if (x in list_of_indices) else np.NAN for x in self.DF.index ]
        
    def parse_episode(self):

        #Parse episode to determine the indices for attempts, i.e where
        #determine the start and end of each attempt based on activity of grasp neuron (i.e. where grasp goes from <0 to >0) and where
        #move_to_grasp starts decreasing.  Finish occurs when lift after release>10 and grasp >20
        
        for i,v in enumerate(self.Episode_IDX["Start IDX"]):
        #for i in [10]:
            print (i)
            st_idx = self.Episode_IDX["Start IDX"][i]
            end_idx = self.Episode_IDX["End IDX"][i]
            
            DFr = self.DF.loc[st_idx:end_idx,:]
            DFr = DFr.reset_index(drop=True)

            ##---- Success parsing ----##
            # find whether the grasp was successful or not -> need grasp>20 after lift_after_release>10

            grv = np.where(DFr["grasp"].to_numpy() > 15)[0][-1]  # get the last element
            lar = np.where(DFr["lift_after_release"].to_numpy() > 10)[0]  # get the last element
            if len(lar) == 0:
                success = False
                at_end_idx = end_idx
            else:
                lar = lar[-1]
                success = grv > lar
                at_end_idx = lar


            print("Trial %i was: %i"%(i,success))

            ##----Attempt parsing ----##
            #find where the move to grasp is less than 0 and the grasp neuron is activate, i.e. greater than 10
            mg = np.diff(DFr["move_to_grasp"].to_numpy())
            mgb = (mg < 0) & (DFr["move_to_grasp"][0:-1].to_numpy() > 10)
            
            #find where the grasp was less than 0 but crossed over to be positive
            gr = np.diff(DFr["grasp"].to_numpy())
            grb = (gr>0) & (DFr["grasp"][0:-1].to_numpy()<0.01) & (DFr["grasp"][1:].to_numpy()>0) #sometimes grasp starts near 0, so need to use <0.01 instead of <0
            
            idx = np.where(mgb & grb)[0]
            if success == True: #TODO: doesn't seem like the right transitions are being detected
                idx = idx[np.where(idx<lar)[0]] #for successful grasps, the end of the sequence where the grasper starts to inflate can falsely trigger as a grasp attempt
                print(idx)




            attempt_start_idx = [x for x in idx]
            num_attempts = len(attempt_start_idx)
            if (num_attempts ==1):
                attempt_end_idx = [at_end_idx]
            else:
                attempt_end_idx = [x for x in idx[1:]]
                attempt_end_idx.append(at_end_idx)


            ##---- For each attempt ----##
            for k in range(0,num_attempts):
                ##---- determine where the transition from grasp to lift-after-grasp occurs ----##

                DFk = DFr.loc[attempt_start_idx[k]:attempt_end_idx[k], :]
                DFk = DFk.reset_index(drop=True)



                zero_pressure = DFk.loc[1:40, ["P_jaw1_psi", "P_jaw2_psi", "P_jaw3_psi"]].mean().to_numpy()

                # find indices of transition from grasp to lift-after-grasp-> look for where gr>20 and the diff is <0
                grk = np.diff(DFk["grasp"].to_numpy())
                grkb = (grk < 0) & (DFk["grasp"][0:-1].to_numpy() > 0)

                # find indices of transition from grasp to lift-after-grasp-> look for where lift after grasp went from negative to positive
                lgk = np.diff(DFk["lift_after_grasp"].to_numpy())
                lgkb = (lgk > 0) & (DFk["lift_after_grasp"][0:-1].to_numpy() < 0) #& (DFk["lift_after_grasp"][1:].to_numpy() > 0)



                # find where the grasp was less than 0 but crossed over to be positive
                trans_press = [np.NaN,np.NaN, np.NaN]
                max_lift_press = [np.NaN,np.NaN, np.NaN]
                max_grasp_press = [np.NaN,np.NaN, np.NaN]

                trans_radius = np.NaN
                max_lift_radius = np.NaN
                max_grasp_radius = np.NaN


                if np.any(grkb & lgkb):
                    idx_lift_start = np.where(grkb & lgkb)[0][0]  #was -1 before


                    pressures_at_transition = (DFk.loc[idx_lift_start,["P_jaw1_psi",
                                                                      "P_jaw2_psi",
                                                                      "P_jaw3_psi"]] - zero_pressure).to_list()

                    print("Attempt %i: Pressures at transition %f, %f, %f"%(k,*pressures_at_transition))
                    trans_press = pressures_at_transition
                    trans_radius = DFk.loc[idx_lift_start,"commanded_radius_mm"]

                    #find max pressure during lift phase. First find where the end of the lift phase occurs
                    lift_phase_end = np.where((lgk<0) & (DFk["lift_after_grasp"][0:-1].to_numpy() > 0))[0]

                    if len(lift_phase_end)>0:
                        if np.any(lift_phase_end>idx_lift_start):
                            lift_phase_end_idx = lift_phase_end[np.where(lift_phase_end>idx_lift_start)[0]][0]

                            max_pressure_during_lift = DFk.loc[idx_lift_start:lift_phase_end_idx,["P_jaw1_psi",
                                                                              "P_jaw2_psi",
                                                                              "P_jaw3_psi"]].max() - zero_pressure

                            print("Attempt %i: Max pressure during lift %f, %f, %f" % (k,*max_pressure_during_lift.to_list()))

                            max_lift_press = max_pressure_during_lift.to_list()
                            max_lift_radius = DFk.loc[idx_lift_start:lift_phase_end_idx, "commanded_radius_mm"].max()

                            #find max pressure during grasp phase. #go from lift after grasp ends to the end of the total
                            max_pressure_during_grasp = DFk.loc[lift_phase_end_idx:, ["P_jaw1_psi",
                                                                                                   "P_jaw2_psi",
                                                                                                   "P_jaw3_psi"]].max() - zero_pressure
                            max_grasp_press = max_pressure_during_grasp.to_list()
                            max_grasp_radius = DFk.loc[lift_phase_end_idx:lift_phase_end_idx, "commanded_radius_mm"].max()
                            print("Attempt %i: Max pressure during grasp %f, %f, %f" % (k,*max_pressure_during_grasp.to_list()))


                self.Summary["Date Classifier"].append(i)
                self.Summary["Trial Number"].append(i)
                self.Summary["Start IDX"].append(np.NaN)
                self.Summary["End IDX"].append(np.NaN)
                self.Summary["Attempt start IDX"].append(np.NaN)
                self.Summary["Attempt end IDX"].append(np.NaN)
                self.Summary["Time Start"].append(np.NaN)
                self.Summary["Time End"].append(np.NaN)
                self.Summary["Success"].append(success)
                self.Summary["Number of Attempts"].append(num_attempts)
                self.Summary["Attempt Number"].append(k)
                self.Summary["Relative Attempt"].append(num_attempts-k)
                self.Summary["Transition Pressure psi"].append(trans_press)
                self.Summary["Max Pressure during lift psi"].append(max_lift_press)
                self.Summary["Max Pressure during grasp"].append(max_grasp_press)
                self.Summary["Transition Radius mm"].append(trans_radius)
                self.Summary["Max Radius during lift mm"].append(max_lift_radius)
                self.Summary["Max Radius during grasp mm"].append(max_grasp_radius)







            #Find maximum pressure during lift-after-grasp

            #Find maximum pressure applied during transport and release


            #
            # self.Summary = {"Date Classifier": [], "Start IDX": [], "End IDX": [], "Attempt start IDX": [],
            #                 "Attempt end IDX": [], "Time Start": [], "Time End": [],
            #                 "Success": [], "Failure": [], "Number of Attempts": [],
            #                 "Attempt average radius (mm)": [], "Attempt average jaw pressure (psi)": [],
            #                 "Attempt std. dev jaw pressure (psi)": [], "Attempt max jaw pressure (psi)": []}



            
            
            
        
        
            pass


# ------ success datalog, K = 30 ------ #
AD = AnalyzeDatalog()
AD.find_episodes()
AD.parse_episode()

SumDF = pd.DataFrame.from_dict(AD.Summary)
SumDF["Gain"]="K = 30"
expTransDF = SumDF[["Transition Pressure psi"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Transition Pressure 1","Transition Pressure 2", "Transition Pressure 3"]), axis=1, result_type="expand")
expMaxLiftDF = SumDF[["Max Pressure during lift psi"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Lift Max Pressure 1","Lift Max Pressure 2", "Lift Max Pressure 3"]), axis=1, result_type="expand")
expMaxGraspDF = SumDF[["Max Pressure during grasp"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Grasp Max Pressure 1","Grasp Max Pressure 2", "Grasp Max Pressure 3"]), axis=1, result_type="expand")

totalSumDF = pd.concat([SumDF,expTransDF,expMaxLiftDF,expMaxGraspDF],axis=1)

totalSumDF.query("`Trial Number` in [0,1,2,4,6,7,8,9,10,11]",inplace = True) #Filter to exclude the success on the 1st pass.

#melt down the dataframe
totalSumDFmelt = totalSumDF.melt(id_vars=['Date Classifier', 'Trial Number', 'Success','Number of Attempts', 'Attempt Number'],
                                 value_vars=['Transition Pressure 1','Transition Pressure 2','Transition Pressure 3', 'Lift Max Pressure 1','Lift Max Pressure 2', 'Lift Max Pressure 3',
                                             'Grasp Max Pressure 1', 'Grasp Max Pressure 2', 'Grasp Max Pressure 3',
                                             'Transition Radius mm', 'Max Radius during lift mm', 'Max Radius during grasp mm'],
                                 var_name='Variable',
                                 value_name='Values')

# ------ fail datalog, K = 1 ------ #
ADf = AnalyzeDatalog(fname = "HardwareDatalog_29_03_2024_21_40_45.csv")
ADf.find_episodes()
ADf.parse_episode()

SumDF_f = pd.DataFrame.from_dict(ADf.Summary)
SumDF_f["Gain"]="K = 1"
expTransDF_f = SumDF_f[["Transition Pressure psi"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Transition Pressure 1","Transition Pressure 2", "Transition Pressure 3"]), axis=1, result_type="expand")
expMaxLiftDF_f = SumDF_f[["Max Pressure during lift psi"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Lift Max Pressure 1","Lift Max Pressure 2", "Lift Max Pressure 3"]), axis=1, result_type="expand")
expMaxGraspDF_f = SumDF_f[["Max Pressure during grasp"]].apply(lambda x:  pd.Series([v for lst in x for v in lst],index = ["Grasp Max Pressure 1","Grasp Max Pressure 2", "Grasp Max Pressure 3"]), axis=1, result_type="expand")

totalSumDF_f = pd.concat([SumDF_f,expTransDF_f,expMaxLiftDF_f,expMaxGraspDF_f],axis=1)

#produce plot of pressure change at transition between success and failures(maybe just jaw 1)
#produce plot of radius at transition between success and failures
#produce plot of maximum pressure change during lift between success and failures (color), x axis is number of attempts
#produce plot of maximum pressure change during grasp between success and failures

# ------ Join the two dataframes ------ #

newDF = pd.concat([totalSumDF.query("Success == True"),totalSumDF_f]).reset_index()

# ------ Plotting ------ #


fig = px.box(newDF,x="Relative Attempt", y="Max Radius during lift mm", color="Gain",points = "all")
fig.show()

fig2 = px.box(newDF,x="Relative Attempt", y="Transition Pressure 1", color="Gain",points = "all")
fig2.show()

fig3 = px.line(AD.DF.query("`Episode Label` in [0]"),x = 'time_delta_s',y = ['commanded_radius_mm','grasp'])
fig3.show()

fig4 = px.line(newDF.query("`Gain`=='K = 30'"),x = 'Attempt Number',y = 'Lift Max Pressure 1', color = "Trial Number", symbol = "Trial Number")
fig4.update_traces(marker=dict(size=18), line=dict(width=3))
fig4.update_layout(font=dict(size=15))
fig4.show()

fig5 = px.line(newDF.query("`Gain`=='K = 1'"),x = 'Attempt Number',y = 'Lift Max Pressure 1', color = "Trial Number", symbol = "Trial Number")
fig5.update_traces(marker=dict(size=18), line=dict(width=3))
fig5.update_layout(font=dict(size=15))
fig5.show()


## subplot figure
newDF_melt = newDF.melt(id_vars=['Attempt Number','Trial Number','Gain'],
                        value_vars=['Lift Max Pressure 1', 'Max Radius during grasp mm'],var_name = 'Variable',value_name ='Values')
fig6 = px.line(newDF_melt,x = 'Attempt Number',y = 'Values', color = "Trial Number", symbol = "Trial Number",facet_col="Gain",facet_row = 'Variable')
fig6.update_traces(marker=dict(size=7), line=dict(width=0.8))
fig6.update_layout(font=dict(size=12))
fig6.update_yaxes(matches = None, showticklabels=True)


#set y limits
fig6.layout['yaxis3'].update(range=[-0.025,0.3])
fig6.layout['yaxis4'].update(range=[-0.025,0.3])
fig6.layout['yaxis'].update(range=[9,27.5])
fig6.layout['yaxis2'].update(range=[9,27.5])

fig6.layout['xaxis3'].update(range=[-0.5,5.5])
fig6.layout['xaxis4'].update(range=[-0.5,5.5])
fig6.layout['xaxis'].update(range=[-0.5,5.5])
fig6.layout['xaxis2'].update(range=[-0.5,5.5])
#fig6.for_each_yaxis(lambda yaxis: yaxis.update(range=[1,3]))
fig6.show()
fig6.write_image("LiftRadius_MaxPressure.svg")


## two plots per figure
fig7 = px.line(newDF,x = 'Attempt Number',y = 'Lift Max Pressure 1', color = "Trial Number", symbol = "Trial Number",facet_col="Gain")
fig7.update_traces(marker=dict(size=12), line=dict(width=1.4))
fig7.update_layout(font=dict(size=15))
fig7.show()
fig7.write_image("LiftMaxPressure.svg")

fig8 = px.line(newDF,x = 'Attempt Number',y = 'Max Radius during lift mm', color = "Trial Number", symbol = "Trial Number",facet_col="Gain")
fig8.update_traces(marker=dict(size=12), line=dict(width=1.4))
fig8.update_layout(font=dict(size=15))
fig8.show()
fig8.write_image("MaxRadiusDuringLift.svg")

plotDF = AD.DF.loc[~np.isnan(AD.DF["Episode Label"]),:]
plotDF = plotDF.melt(id_vars=['time_delta_s', 'Valid Episode', 'Episode Label'],
                     value_vars=['move_to_pre_grasp', 'move_to_grasp','grasp','lift_after_grasp',
                                 'move_to_pre_release','move_to_release','release','lift_after_release',
                                 'commanded_radius_mm'],
                     var_name='Variable', value_name='Values')



# g = sns.FacetGrid(plotDF, col="Episode Label", hue = 'Variable',col_wrap=6, height=2)
# g.map(sns.lineplot, "time_delta_s", 'Values')
# g.add_legend()
# plt.show()



# pd.options.plotting.backend = "plotly"
# fig = AD.DF.plot(title="Pandas Backend Example", template="simple_white",
#               labels=dict(index="time", value="money", variable="option"))
# fig.update_yaxes(tickprefix="$")
# fig.show()







