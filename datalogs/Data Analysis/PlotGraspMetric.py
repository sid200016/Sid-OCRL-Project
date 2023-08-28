import pandas as pd
import numpy as np
import seaborn as sb
import matplotlib.pyplot as plt
import matplotlib as mpl
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import statsmodels.api as sm
from statsmodels.formula.api import ols
from scipy.stats import ttest_ind
from scipy.stats import mannwhitneyu


mpl.rcParams['font.family'] = 'Calibri'
mpl.rcParams.update({'font.size': 16})
mpl.rcParams['svg.fonttype'] = 'none'

#load dataframe
DF = pd.read_excel("UserExperimentsData_Updated.xlsx", skiprows=[0])
print(DF)

fname = "..\HardwareDatalog_17_08_2023_11_15_43.csv" #User # 2, Manual datalog

datalogHeader = ["Date Time", "Milliseconds", "Program", "Status",
                 "x", "y", "z",
                 "closure muscle pressure (psi)", "Jaw pressure 1 (psi)", "Jaw pressure 2 (psi)",
                 "Jaw pressure 3 (psi)",
                 "A", "B", "X", "Y", "SL", "SR", "+", "Stick In", "Home", "R", "ZR", "Joy x", "Joy y",
                 "Object 1", "Object 2", "Object 3", "Object 4", "Object 5", "Object 6", "Object 7", "Object 8",
                 "Object 9"]


u2DF = pd.read_csv(fname, names=datalogHeader)

#calculate the Pressure Range value

u2DF["ContactMetric"] = u2DF.apply(lambda x: max(min((max(x['Jaw pressure 1 (psi)'],x['Jaw pressure 2 (psi)'],x['Jaw pressure 3 (psi)'])-0.05),5),0)*100,axis=1)


#Broken and successful indices.  Using item # 6 from the Manual run of User #2
broken=[7600,8600] #start and stop index
success = [8600, 9618]#start and stop index

#new column
u2DF["PlaceHolder"] = "NA"
u2DF.loc[broken[0]:broken[1],"PlaceHolder"] ="Broke"
u2DF.loc[success[0]:success[1],"PlaceHolder"] ="Success"

u2DF["Idx"] = 0
u2DF.loc[broken[0]:broken[1],"Idx"] = list(range(0,1+broken[1]-broken[0]))
u2DF.loc[success[0]:success[1],"Idx"] = list(range(0,1+success[1]-success[0]))

u2DF_red = u2DF.loc[u2DF["PlaceHolder"]!="NA",:]

#new column to indicate whether it is

sb.lineplot(data=u2DF_red,x="Idx", y="ContactMetric", hue="PlaceHolder")

plt.show()