import pandas as pd
import numpy as np
import seaborn as sb
import matplotlib.pyplot as plt
import matplotlib as mpl
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import statsmodels.api as sm
from statsmodels.formula.api import ols



mpl.rcParams['font.family'] = 'Calibri'
mpl.rcParams.update({'font.size': 12})

#load dataframe
DF = pd.read_excel("UserExperimentsData_Updated.xlsx", skiprows=[0])
print(DF)

#Filter to the successes
DFsuccess = DF.loc[DF["Updated Classification"]==" success",:]
sb.boxplot(data=DFsuccess,x="Object Type", y="Total Grasp Time")

#two-way ANOVA sanity check for grasp time
model = ols('Q("Grasp Time") ~ C(Q("Group Type")) + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control"))',data=DFsuccess).fit()
graspAnova = sm.stats.anova_lm(model,typ=2) #two way anova #https://www.statology.org/two-way-anova-python/

#two-way ANOVA sanity check for deposit time
model = ols('Q("Deposit Time") ~ C(Q("Group Type")) + C(Q("Type of Control")) +C(Q("Group Type")):C(Q("Type of Control"))',data=DFsuccess).fit()
depositAnova = sm.stats.anova_lm(model,typ=2) #two way anova #https://www.statology.org/two-way-anova-python/

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

# For SNS vs. non SNS for grasp time
g = sb.catplot(x="Type of Control", y="Grasp Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Type of Control", y="Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

#Deposit Time for SNS vs. non SNS, split by the Group Type
g = sb.catplot(x="Group Type", y="Deposit Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Deposit Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

# For SNS vs. non SNS for deposit time
g = sb.catplot(x="Type of Control", y="Deposit Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Type of Control", y="Deposit Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)




##--- Plot of grasp time ---##
# g = sb.catplot(x="Object Type", y="Grasp Time", row="Participant Number", kind="box", hue="Type of Control", data=DFsuccess)



plt.show()



##--- Plot of deposit time ---##



##--- Plot of contact force for successful and unsuccessful grasps ---##

##--- Plot of closure muscle pressure for successful and unsuccessful grasps ---##



##--- Plot of x-y positions for SNS successful and unsuccessful grasps ---##


