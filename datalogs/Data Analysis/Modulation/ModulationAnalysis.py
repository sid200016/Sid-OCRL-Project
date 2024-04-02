import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt


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
        self.Summary = {"Date Classifier":[],"Start IDX":[],"End IDX":[], "Attempt start IDX":[],"Attempt end IDX":[],
                        "Time Start":[], "Time End":[],
                        "Success":[], "Failure":[],
                        "Number of Attempts": [], "Attempt average radius (mm)": [], "Attempt average jaw pressure (psi)":[],
                        "Attempt std. dev jaw pressure (psi)": [], "Attempt max jaw pressure (psi)":[]}


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
            print (i)
            st_idx = self.Episode_IDX["Start IDX"][i]
            end_idx = self.Episode_IDX["End IDX"][i]
            
            DFr = self.DF.loc[st_idx:end_idx,:]
            DFr = DFr.reset_index(drop=True)

            ##----Attempt parsing ----##
            #find where the move to grasp is less than 0 and the grasp neuron is activate, i.e. greater than 10
            mg = np.diff(DFr["move_to_grasp"].to_numpy())
            mgb = (mg < 0) & (DFr["move_to_grasp"][0:-1].to_numpy() > 10)
            
            #find where the grasp was less than 0 but crossed over to be positive
            gr = np.diff(DFr["grasp"].to_numpy())
            grb = (gr>0) & (DFr["grasp"][0:-1].to_numpy()<0) & (DFr["grasp"][1:].to_numpy()>0)
            
            idx = np.where(mgb & grb)[0]
            print(idx)


            ##---- Success parsing ----##
            #find whether the grasp was successful or not -> need grasp>20 after lift_after_release>10

            grv = np.where(DFr["grasp"].to_numpy() >15) [0][-1] #get the last element
            lar = np.where(DFr["lift_after_release"].to_numpy()>10)[0] #get the last element
            if len(lar)==0:
                success = False
                at_end_idx = end_idx[i]
            else:
                lar = lar[-1]
                success = grv>lar
                at_end_idx = lar

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
                lgkb = (lgk > 0) & (DFk["lift_after_grasp"][0:-1].to_numpy() < 0) & (DFk["lift_after_grasp"][1:].to_numpy() > 0)



                # find where the grasp was less than 0 but crossed over to be positive
                idx_lift_start = np.where(grkb & lgkb)[0][-1]


                pressures_at_transition = (DFk.loc[idx_lift_start,["P_jaw1_psi",
                                                                  "P_jaw2_psi",
                                                                  "P_jaw3_psi"]] - zero_pressure).to_list()

                print("Attempt %i: Pressures at transition %f, %f, %f"%(k,*pressures_at_transition))

                #find max pressure during lift phase. First find where the end of the lift phase occurs
                lift_phase_end = np.where((lgk<0) & (DFk["lift_after_grasp"][0:-1].to_numpy() > 0))[0]
                lift_phase_end_idx = lift_phase_end[np.where(lift_phase_end>idx_lift_start)[0]][0]

                max_pressure_during_lift = DFk.loc[idx_lift_start:lift_phase_end_idx,["P_jaw1_psi",
                                                                  "P_jaw2_psi",
                                                                  "P_jaw3_psi"]].max() - zero_pressure

                print("Attempt %i: Max pressure during lift %f, %f, %f" % (k,*max_pressure_during_lift.to_list()))



                #find max pressure during grasp phase. #go from lift after grasp ends to the end of the total
                max_pressure_during_grasp = DFk.loc[lift_phase_end_idx:, ["P_jaw1_psi",
                                                                                       "P_jaw2_psi",
                                                                                       "P_jaw3_psi"]].max() - zero_pressure

                print("Attempt %i: Max pressure during grasp %f, %f, %f" % (k,*max_pressure_during_grasp.to_list()))









            #Find maximum pressure during lift-after-grasp

            #Find maximum pressure applied during transport and release


            #
            # self.Summary = {"Date Classifier": [], "Start IDX": [], "End IDX": [], "Attempt start IDX": [],
            #                 "Attempt end IDX": [], "Time Start": [], "Time End": [],
            #                 "Success": [], "Failure": [], "Number of Attempts": [],
            #                 "Attempt average radius (mm)": [], "Attempt average jaw pressure (psi)": [],
            #                 "Attempt std. dev jaw pressure (psi)": [], "Attempt max jaw pressure (psi)": []}



            
            
            
        
        
            pass


AD = AnalyzeDatalog()
AD.find_episodes()
AD.parse_episode()



plotDF = AD.DF.loc[~np.isnan(AD.DF["Episode Label"]),:]
plotDF = plotDF.melt(id_vars=['time_delta_s', 'Valid Episode', 'Episode Label'],
                     value_vars=['move_to_pre_grasp', 'move_to_grasp','grasp','lift_after_grasp',
                                 'move_to_pre_release','move_to_release','release','lift_after_release',
                                 'commanded_radius_mm'],
                     var_name='Variable', value_name='Values')


g = sns.FacetGrid(plotDF, col="Episode Label", hue = 'Variable',col_wrap=6, height=2)
g.map(sns.lineplot, "time_delta_s", 'Values')
g.add_legend()
plt.show()
# pd.options.plotting.backend = "plotly"
# fig = AD.DF.plot(title="Pandas Backend Example", template="simple_white",
#               labels=dict(index="time", value="money", variable="option"))
# fig.update_yaxes(tickprefix="$")
# fig.show()







