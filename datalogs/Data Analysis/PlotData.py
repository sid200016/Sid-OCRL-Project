import pandas as pd
import numpy as np
import seaborn as sb
import matplotlib.pyplot as plt

#load dataframe
DF = pd.read_excel("UserExperimentsData_Updated.xlsx", skiprows=[0])
print(DF)

#Filter to the successes
DFsuccess = DF.loc[DF["Updated Classification"]==" success",:]
sb.boxplot(data=DFsuccess,x="Object Type", y="Total Grasp Time")

orderType = DFsuccess["Group Type"]=="SNS_first"

#Create a box plot with scatter for each user.
#g = sb.FacetGrid(DFsuccess, col="Participant Number")
#g.map(sb.boxplot, "Object Type","Total Grasp Time","Type of Control",palette='muted')

##--- Plot of success rate ---##

#g = sb.catplot(x="Object Type", y="Proctor Classification",  kind="bar",  data=DFsuccess,hue= "Type of Control")

#Per Object


##--- Plot of total grasp time ---##
#Individuals
g = sb.catplot(x="Object Type", y="Total Grasp Time", row="Participant Number", col="Group Type", kind="box", hue="Type of Control", data=DFsuccess.loc[orderType,:], width = 0.2)
g.map_dataframe(sb.stripplot, x ="Object Type", y="Total Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

g = sb.catplot(x="Object Type", y="Total Grasp Time", row="Participant Number", col="Group Type", kind="box", hue="Type of Control", data=DFsuccess.loc[~orderType,:], width = 0.2)
g.map_dataframe(sb.stripplot, x ="Object Type", y="Total Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False)


#For SNS vs. non SNS
g = sb.catplot(x="Group Type", y="Total Grasp Time",  col="Object Type", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Total Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

#For SNS vs. non SNS but not considering object type
g = sb.catplot(x="Group Type", y="Total Grasp Time", kind="box", hue="Type of Control", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Total Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

#For SNS vs. non SNS, split by the Group Type
g = sb.catplot(x="Group Type", y="Total Grasp Time", kind="box", hue="Type of Control", row="Group Type", data=DFsuccess, width = 0.2)
g.map_dataframe(sb.stripplot, x="Group Type", y="Total Grasp Time", hue = "Type of Control", palette=["#404040","#404040"], alpha=0.6, dodge = True, jitter = False, native_scale = False)

##--- Plot of grasp time ---##
g = sb.catplot(x="Object Type", y="Grasp Time", row="Participant Number", kind="box", hue="Type of Control", data=DFsuccess)
plt.show()

##--- Plot of deposit time ---##



##--- Plot of contact force for successful and unsuccessful grasps ---##

##--- Plot of closure muscle pressure for successful and unsuccessful grasps ---##



##--- Plot of x-y positions for SNS successful and unsuccessful grasps ---##


