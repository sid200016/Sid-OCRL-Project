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
from scipy.stats import kstest
import scipy.stats as ss
from scipy.stats import shapiro
import seaborn.objects as so



mpl.rcParams['font.family'] = 'Calibri'
mpl.rcParams.update({'font.size': 16})
mpl.rcParams['svg.fonttype'] = 'none'

#load dataframe
DF = pd.read_excel("UserExperimentsData_Updated.xlsx", skiprows=[0])
print(DF)

#Filter to the successes
DFsuccess = DF.loc[(DF["Updated Classification"]==" success") & (DF["Immediate Reattempt"]==False) & (DF["Reattempt"]==False) ,:]
sb.boxplot(data=DFsuccess,x="Object Type", y="Total Grasp Time")

#two-way ANOVA sanity check for grasp time
model = ols('Q("Grasp Time") ~ C(Q("Group Type")) + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control"))',data=DFsuccess).fit()
model = ols('Q("Grasp Time") ~ C(Q("Group Type")) + C(Q("Object Type"))  + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control")):C(Q("Object Type"))+C(Q("Type of Control")):C(Q("Object Type"))+C(Q("Type of Control")):C(Q("Group Type"))+C(Q("Group Type")):C(Q("Object Type"))',data=DFsuccess).fit()

graspAnova = sm.stats.anova_lm(model,typ=3) #two way anova #https://www.statology.org/two-way-anova-python/

#two-way ANOVA sanity check for deposit time
#model = ols('Q("Deposit Time") ~ C(Q("Group Type")) + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control"))',data=DFsuccess).fit()
model = ols('Q("Deposit Time") ~ C(Q("Group Type")) + C(Q("Object Type"))  + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control")):C(Q("Object Type"))+C(Q("Type of Control")):C(Q("Object Type"))+C(Q("Type of Control")):C(Q("Group Type"))+C(Q("Group Type")):C(Q("Object Type"))',data=DFsuccess).fit()

depositAnova = sm.stats.anova_lm(model,typ=3) #two way anova #https://www.statology.org/two-way-anova-python/

#Check deposit time data for normality.
#SNS
resultSNS = shapiro(DFsuccess.loc[DFsuccess["Type of Control"]=="SNS","Deposit Time"])
print("SNS")
print(resultSNS)

resultSNS = shapiro(DFsuccess.loc[DFsuccess["Type of Control"]=="Manual","Deposit Time"])
print("Manual")
print(resultSNS)

#perform mann-whitney U
pval_Mann_depositTime = mannwhitneyu(DFsuccess.loc[DFsuccess["Type of Control"]=="Manual","Deposit Time"],
                                 DFsuccess.loc[DFsuccess["Type of Control"]=="SNS","Deposit Time"]).pvalue

#Pairwise comparison for grasp time:
DFsuccess_SNSfirst = DFsuccess.loc[DFsuccess["Group Type"]=="SNS_first"]
tukeySNS = pairwise_tukeyhsd(endog=DFsuccess_SNSfirst['Grasp Time'],groups=DFsuccess_SNSfirst['Type of Control'])


DFsuccess_Manualfirst = DFsuccess.loc[DFsuccess["Group Type"]=="Manual_first"]
tukeyManual = pairwise_tukeyhsd(endog=DFsuccess_Manualfirst['Grasp Time'],groups=DFsuccess_Manualfirst['Type of Control'])

orderType = DFsuccess["Group Type"]=="SNS_first"



def calcSuccessFunc(df):
    success = (df["Updated Classification"]== " success"  ).sum()
    return success


summaryDF = DFsuccess.groupby(["Group Type","Type of Control","Participant Number","Object Type"]).apply(calcSuccessFunc).to_frame("Number of Successes")
summaryDF.to_csv("SummarySuccess.csv")

def getCompareSamples(df):
    if "Manual" in list(df["Type of Control"]) and "SNS" in list(df["Type of Control"]):
        pval_Mann = mannwhitneyu(df.loc[df["Type of Control"]=="Manual","Grasp Time"],
                                 df.loc[df["Type of Control"]=="SNS","Grasp Time"]).pvalue

        pval = ttest_ind(df.loc[df["Type of Control"] == "Manual", "Grasp Time"],
                            df.loc[df["Type of Control"] == "SNS", "Grasp Time"])
        return(pval)

    else:
        return(100)

compareSamplesDF = DFsuccess.groupby(["Group Type","Participant Number"]).apply(getCompareSamples).to_frame("P value")
print(compareSamplesDF)


#Read Tabulated Success Table
DF_graspsuccess = pd.read_excel("SummarySuccess.xlsx",sheet_name="SummarySuccess")
print(DF_graspsuccess)


#Create a box plot with scatter for each user.
#g = sb.FacetGrid(DFsuccess, col="Participant Number")
#g.map(sb.boxplot, "Object Type","Total Grasp Time","Type of Control",palette='muted')

##--- Plot of success rate ---##

#g = sb.catplot(x="Object Type", y="Proctor Classification",  kind="bar",  data=DFsuccess,hue= "Type of Control")

#Per Object


##--- Plot of grasp time ---##
#Individuals
# g = sb.catplot(x="Object Type", y="Grasp Time", row="Participant Number", col="Group Type", kind="box", hue="Type of Control", data=DFsuccess.loc[orderType,:], width = 0.2)
# g.map_dataframe(sb.stripplot, x ="Object Type", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
#
# g = sb.catplot(x="Object Type", y="Grasp Time", row="Participant Number", col="Group Type", kind="box", hue="Type of Control", data=DFsuccess.loc[~orderType,:], width = 0.2)
# g.map_dataframe(sb.stripplot, x ="Object Type", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False)


#For SNS vs. non SNS
# g = sb.catplot(x="Group Type", y="Grasp Time",  col="Object Type", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
# g.map_dataframe(sb.stripplot, x="Group Type", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)



#For SNS vs. non SNS, split by the Group Type
# g = sb.catplot(x="Group Type", y="Grasp Time", kind="box", hue="Type of Control", row="Group Type", data=DFsuccess, width = 0.2)
# g.map_dataframe(sb.stripplot, x="Group Type", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

#For Breakable vs Graspable, SNS vs. Non-SNS
# g = sb.catplot(x="Type of Control", y="Grasp Time",  col="Group Type", kind="box", hue="Object Type", data=DFsuccess, width = 0.2)
# g.map_dataframe(sb.stripplot, x="Type of Control", y="Grasp Time", hue = "Object Type", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

#For SNS vs. non SNS but not considering object type
g = sb.catplot(x="Group Type", y="Grasp Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
g.savefig("GraspTime_vs_GroupType.svg",format="svg", dpi=1200)

# For SNS vs. non SNS for grasp time
g = sb.catplot(x="Type of Control", y="Grasp Time", kind="box", hue="Type of Control", data=DFsuccess,  height = 2*58.06/25.4, aspect = 77.16/58.06)
g.map_dataframe(sb.stripplot, x="Type of Control", y="Grasp Time", hue = "Object Type", palette=["#FF0000","#0000FF","#0FFF50"], alpha=1, size = 8,  dodge = False, jitter = False, native_scale = False)
g.savefig("GraspTime_vs_TypeOfControl.svg",format="svg", dpi=1200)

#Deposit Time for SNS vs. non SNS, split by the Group Type
g = sb.catplot(x="Group Type", y="Deposit Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Deposit Time", hue = "Object Type", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
g.savefig("GroupType_vs_DepositTime.svg",format="svg", dpi=1200)

# For SNS vs. non SNS for deposit time
g = sb.catplot(x="Type of Control", y="Deposit Time", kind="box", hue="Type of Control", data=DFsuccess,  height = 2*58.06/25.4, aspect = 77.16/58.06,)
g.map_dataframe(sb.stripplot, x="Type of Control", y="Deposit Time",hue="Object Type", palette=["#FF0000","#0000FF","#0FFF50"], alpha=1, size = 8,  dodge = False, jitter = False, native_scale = False)
g.savefig("TypeOfControl_vs_DepositTime.svg",format="svg", dpi=1200)


# For SNS vs. non SNS for grasp time
g = sb.catplot(x="Type of Control", y="Grasp Time", kind="box", hue="Group Type", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Type of Control", y="Grasp Time", hue = "Group Type", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
g.savefig("TypeOfControl_vs_GraspTime.svg",format="svg", dpi=1200)

# For SNS vs. non SNS for grasp time
g = sb.catplot(x="Group Type", y="Grasp Time", kind="box", hue="Group Type", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Grasp Time", hue = "Group Type", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
g.savefig("GroupType_vs_GraspTime_NoTypeOfControl.svg",format="svg", dpi=1200)

# For SNS vs. Non SNS
g = sb.catplot(x="Group Type", y="Grasp Time", kind="box", hue="Group Type", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Grasp Time", hue = "Group Type", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)
g.savefig("GroupType_vs_GraspTime_NoTypeOfControl.svg",format="svg", dpi=1200)


# For SNS vs. Non SNS, by participant by object
g = sb.catplot(x="Object Type", y="Grasp Time", kind="box", hue="Type of Control",col="Participant Number", data=DFsuccess, height = 2*55.02/25.4, aspect=0.25*161.5/55.02)
g.map_dataframe(sb.stripplot,x="Object Type", y="Grasp Time",  hue="Object Type", palette=["#FF0000","#0000FF","#0FFF50"], alpha=1, size = 8,  dodge = False, jitter = False, native_scale = False)
g.savefig("Object_Group_Partipant_ControlType_GraspTime.svg",format="svg", dpi=1200)


# For SNS vs. Non SNS, by participant by object
g = sb.catplot(x="Type of Control", y="Grasp Time", kind="box", hue="Type of Control",col="Participant Number", data=DFsuccess, height = 2*55.02/25.4, aspect=0.25*161.5/55.02, legend = "full")
g.map_dataframe(sb.stripplot,x="Type of Control", y="Grasp Time",  hue="Object Type", palette=["#FF0000","#0000FF","#0FFF50"], alpha=1, size = 8,  dodge = False, jitter = False, native_scale = False, legend = "full")
g.savefig("Group_Partipant_ControlType_GraspTime.svg",format="svg", dpi=1200)

#sb.stripplot(x="Type of Control", y="Grasp Time",  hue="Object Type", palette=["#FF0000","#0000FF","#0FFF50"], alpha=1, size = 8,  dodge = False, jitter = False, native_scale = False, legend = "full")

# Bar charts to compare success rate
g = sb.catplot(x="Object Type", y="Success Rate Percentage", kind="bar", hue="Type of Control", errorbar = "se", data=DF_graspsuccess,  legend = "full",height = 4*55.02/25.4, aspect=161.5/55.02,)
g.map_dataframe(sb.stripplot,x="Object Type", y="Success Rate Percentage", hue="Type of Control",  alpha=1, size = 9.5, dodge = True, jitter = 0.2, native_scale = False, legend = "full")
#g.map_dataframe(sb.scatterplot,x="Object Type", y="Success Rate Percentage", hue="Participant Number", style="Participant Number", markers=["D","o","s","^"], alpha=1, size = 18, legend = "full")
g.savefig("Group_ControlType_GraspTime_SuccessRate.svg",format="svg", dpi=1200)


# Bar charts to compare success rate by participant
# g = sb.catplot(x="Type of Control", y="Success Rate Percentage", kind="bar", hue="Type of Control",col="Participant Number", errorbar = "se", data=DF_graspsuccess,  legend = "full",height = 4*55.02/25.4, aspect=0.25*161.5/55.02)
# #g.map_dataframe(sb.stripplot,x="Type of Control", y="Success Rate Percentage",  hue="Object Type",  alpha=1, size = 9.5, palette="magma" , dodge = False, jitter = False, native_scale = False, legend = "full")
# g.map_dataframe(sb.scatterplot,x="Type of Control", y="Success Rate Percentage", hue="Object Type", style="Object Type", markers=["D","o","s"], alpha=1, size = [20,20,20,20,20,20], legend = "auto")
# g.savefig("Group_Participant_ControlType_GraspTime_SuccessRate.svg",format="svg", dpi=1200)


# For SNS vs. Non SNS success rate, by participant by object
g = sb.catplot(x="Type of Control", y="Success Rate Percentage", kind="bar", hue="Type of Control",col="Participant Number", data=DF_graspsuccess, errorbar = "se",height = 2*55.02/25.4, aspect=0.25*161.5/55.02, legend = "full")
g.map_dataframe(sb.stripplot,x="Type of Control", y="Success Rate Percentage", hue="Type of Control",  alpha=1, size = 8,  dodge = True, jitter = 0.2, native_scale = False, legend = "full")
g.savefig("Group_Participant_ControlType_GraspTime_SuccessRate.svg",format="svg", dpi=1200)

##--- Plot of grasp time ---##
# g = sb.catplot(x="Object Type", y="Grasp Time", row="Participant Number", kind="box", hue="Type of Control", data=DFsuccess)



plt.show()



##--- Plot of deposit time ---##



##--- Plot of contact force for successful and unsuccessful grasps ---##

##--- Plot of closure muscle pressure for successful and unsuccessful grasps ---##



##--- Plot of x-y positions for SNS successful and unsuccessful grasps ---##


