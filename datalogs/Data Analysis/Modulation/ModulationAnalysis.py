import pandas as pd
import numpy as np

DF=pd.read_excel("WithModulation.xlsx")

print(DF)

'''
Pseudocode

TODO: Produce Table of:
Attempt Time  Success   Failure Number_of_Attempts  Final_Attempt_average_radius Final_Attempt_average_pressure Final_Attempt_max_radius Final_Attempt_max_pressure Indices_of_attempts Indices_of_trial Max_Pressure_Per_Attempt

To define at attempt, look for where it goes from GO_HOME to USE_SNS and then from USE_SNS to NORMAL

'''


class AnalyzeDatalog():


    def __init__(self,fname = 'WithModulation.xlsx'):

        self.DF = pd.read_excel(fname)

        self.Episode_IDX = {"Date Classifier":[], "Start IDX":[], "End IDX":[]}
        self.Summary = {"Date Classifier":[],"Start IDX":[],"End IDX":[], "Attempt start IDX":[],"Attempt end IDX":[], "Time Start":[], "Time End":[],
                        "Success":[], "Failure":[], "Number of Attempts": [],
                        "Attempt average radius (mm)": [], "Attempt average jaw pressure (psi)":[],
                        "Attempt std. dev jaw pressure (psi)": [], "Attempt max jaw pressure (psi)":[]}


    def find_episodes(self):

        gd = self.DF.Mode[0:-1]
        gd2 = self.DF.Mode[1:]
        start_idx = np.where((np.array(gd)=="GO_HOME")&(np.array(gd2)=="USE_SNS"))+1

        end_idx = np.where((np.array(gd)=="USE_SNS")&(np.array(gd2)=="NORMAL"))

        self.Episode_IDX["Start IDX"] = start_idx
        self.Episode_IDX["End IDX"] = end_idx
        self.Episode_IDX["Date Classifier"] = [DF.at[x,"Time Stamp"]+","+str(DF.at[x,"Seconds"]) for x in start_idx]



AD = AnalyzeDatalog()
AD.find_episodes()





