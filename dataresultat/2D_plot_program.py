import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from datetime import datetime
import os


#----------------------------Settings----------------------------#
n_agents = 2
folder = "current_03/current_-z03"
save_fig = True
display_fig = False

#----------------------------Functions----------------------------#
def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

def xyz_plot(df1, df2, save_fig, display_fig):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)

    fig.suptitle('Position')
    ax1.plot(full_sec1, df1['x_ref'], 'black', label='Ref x')
    ax1.plot(full_sec1, df1['x'], 'b', label='x1')
    ax1.plot(full_sec2, df2['x'], 'g', label='x2')
    ax1.legend(loc='upper left')
    ax1.grid()

    ax2.plot(full_sec1, df1['y_ref'], 'black', label='Ref y')
    ax2.plot(full_sec1, df1['y'], 'b', label='y1')
    ax2.plot(full_sec2, df2['y'], 'g', label='y2')
    ax2.legend(loc='upper left')
    ax2.grid()

    ax3.plot(full_sec1, df1['z_ref'], 'black', label='Ref z')
    ax3.plot(full_sec1, df1['z'], 'b', label='z1')
    ax3.plot(full_sec2, df2['z'], 'g', label='z2')
    ax3.legend(loc='upper left')
    ax3.grid()


    plot_name = "2D_xyz_"+file[43:].replace(".csv", ".png")
    plot_name = plot_name.replace("--rov2", "")
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        print("Displaying figure",file[45:])
        plt.show()

def angle_plot(df1,df2, save_fig, display_fig):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, ax = plt.subplots()

    ax.plot(full_sec1, df1['angle2'],'b', label='FOV angle 1')
    ax.plot(full_sec2, df2['angle2'],'g', label='FOV angle 2')
    ax.plot(full_sec1, [90 for i in range(len(full_sec1))], 'r', label='FOV limit')
    ax.legend(loc='upper left')

    ax.set(xlabel='time (s)', ylabel='angle (deg)',title="FOV angle")
    ax.grid()

    plot_name = "2D_FOV_"+file[45:].replace(".csv", ".png")
    plot_name = plot_name.replace("--rov2", "")
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        plt.show()

def distance_rovs(df1,df2, save_fig, display_fig):
    #### FUNKER IKKE
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, ax = plt.subplots()
    distance = []
    if(len(full_sec1)<len(full_sec2)):
        for i in range(len(full_sec1)):
            distance.append(np.sqrt((df1['x'][i]-df2['x'][i])**2+(df1['y'][i]-df2['y'][i])**2+(df1['z'][i]-df2['z'][i])**2))
        ax.plot(full_sec1, distance, 'b', label='Distance')

    else:
        for i in range(len(full_sec2)):
            distance.append(np.sqrt((df1['x'][i]-df2['x'][i])**2+(df1['y'][i]-df2['y'][i])**2+(df1['z'][i]-df2['z'][i])**2))
        ax.plot(full_sec2, distance, 'b', label='Distance')
        
    ax.set(xlabel='time (s)', ylabel='Distance (m)',title="Distance between ROVs")
    ax.grid()
    plot_name = "2D_Dist_ROV_"+file[45:].replace(".csv", ".png")
    plot_name = plot_name.replace("--rov2", "")
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        plt.show()

def distance_from_ref(df1,df2, save_fig, display_fig):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)

    fig.suptitle('Distance from reference')

    ax1.plot(full_sec1, abs(df1['x']-df1['x_ref']), 'r', label='Delta x')
    ax1.plot(full_sec1, abs(df1['y']-df1['y_ref']), 'g', label='Delta y')
    ax1.plot(full_sec1, abs(df1['z']-df1['z_ref']), 'b', label='Delta z')
    ax1.set(ylabel='Distance [m]', title="ROV 1")
    ax1.legend(loc='upper left')
    ax1.grid()

    ax2.plot(full_sec2, abs(df2['x']-df2['x_ref']), 'r', label='Delta x')
    ax2.plot(full_sec2, abs(df2['y']-df2['y_ref']), 'g', label='Delta y')
    ax2.plot(full_sec2, abs(df2['z']-df2['z_ref']), 'b', label='Delta z')
    ax2.set(xlabel='time (s)', ylabel='Distance [m]', title="ROV 2")
    ax2.legend(loc='upper left')
    ax2.grid()

    plot_name = "2D_Dist_Ref_"+file[45:].replace(".csv", ".png")
    plot_name = plot_name.replace("--rov2", "")
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        plt.show()


#----------------------------Plotting logic----------------------------#   
files = os.listdir(folder)


for i, file in enumerate(files):
    if (("rov2" in file) and (".csv" in file)):
        pass
    else:
        files[i] = ''
files = list(filter(None, files))
    

for file in files:
    df1 = pd.read_csv(folder+'/'+file, encoding='utf-8', engine='python')

    file2 = file
    file2 = file2.replace("rov2", "rov3")
    df2 = pd.read_csv(folder+'/'+file2, encoding='utf-8', engine='python')

    xyz_plot(df1, df2, save_fig, display_fig)
    angle_plot(df1, df2, save_fig, display_fig)
    #distance_rovs(df1, df2, save_fig, display_fig)
    distance_from_ref(df1, df2, save_fig, display_fig)

    


