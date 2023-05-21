from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os
plt.rcParams.update({'font.size': 18})#plt font size

files = [
  #"Date--10--05--23--Time--20--24--34--default_12_circle--rov3.csv",
#"Date--10--05--23--Time--21--05--16--default_14_circle--rov3.csv",
#"Date--10--05--23--Time--18--26--36--default_6_circle--rov2.csv",
"Date--10--05--23--Time--18--06--50--default_5_circle--rov2.csv",
#"Date--10--05--23--Time--22--05--52--default_17_circle--rov3.csv",
"Date--11--05--23--Time--07--12--35--default_43_circle--rov3.csv",
#"Date--10--05--23--Time--22--46--24--default_19_circle--rov2.csv",
"Date--11--05--23--Time--06--52--50--default_42_circle--rov3.csv",
]

names = ["Test:5 rov2", "Test:43 rov3", "Test:42 rov3"]
color = ["red", "blue", "green"]

folder = "default_school_computer"

df = []

for file in files:
    df.append(pd.read_csv(folder+"/"+file))

fig, ax = plt.subplots(3,2,sharey=False,sharex=True)

plt.subplots_adjust(#left=0.1, #Adjust spacing
                #bottom=0.1,
                #right=0.9,
                #top=0.9,
                #wspace=0.4,
                hspace=0.25)
fig.set_size_inches(26.5,16)

ax[0,0].plot(df[0]["time"], df[0]["x_ref"], label="Reference", color="black")
ax[1,0].plot(df[0]["time"], df[0]["y_ref"], color="black")
ax[2,0].plot(df[0]["time"], df[0]["z_ref"], color="black")
for i, df_ in enumerate(df):
    distance_rovs = [np.sqrt((df_['x'][i]-df_['real_x2'][i])**2+(df_['y'][i]-df_['real_y2'][i])**2+(df_['z'][i]-df_['real_z2'][i])**2) for i in range(len(df_['x']))]
    ax[0,0].plot(df_["time"], df_["x"], color = color[i], label=names[i])
    ax[1,0].plot(df_["time"], df_["y"], color = color[i])
    ax[2,0].plot(df_["time"], df_["z"], color = color[i])
    ax[0,1].plot(df_["time"], df_["angle2"], color = color[i])
    ax[1,1].plot(df_["time"], distance_rovs, color = color[i])

    ax[0,0].title.set_text('Position')
    #ax[0,0].legend()
    ax[0,0].set_ylabel('x [m]')

    fig.suptitle("Circle (Default PC 3) Breakdown Cases", fontsize=24)

    #ax[1,0].legend()
    ax[1,0].set_ylabel('y [m]')

    ax[2,0].set_ylabel('z [m]')
    ax[2,0].set_xlabel('Time [s]')
    #ax[2,0].legend()

    ax[0,1].title.set_text('Angle between ROVs')
    #ax[0,1].legend()
    ax[0,1].set_ylabel('Angle [$^\circ$]')


    ax[1,1].title.set_text('Distance between ROVs')
    ax[1,1].set_xlabel('Time [s]')
    ax[1,1].set_ylabel('Distance [m]')
    #plt.setp(ax[1,1].get_xticklabels(), visible=True)
    ax[1,1].xaxis.set_tick_params(labelbottom=True)
    ax[1,1].set_ylim([0,5])
    # ax[1,1].legend()
fig.delaxes(ax[-1,-1])
    #ax[-1,-1].axis('off')

fig.legend(loc='upper left',ncol=4, bbox_to_anchor=(0.525,0.34))
plt.savefig("breakdown"+'.pdf', format="pdf", bbox_inches='tight')
plt.show()