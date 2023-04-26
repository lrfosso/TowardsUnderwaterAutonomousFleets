from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os

#----------------------------Settings----------------------------#
n_agents = 2
folder = "disturbances/plus_minus_3"
save_fig = False
display_fig = True

#----------------------------------------------------------------#

def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

files = os.listdir(folder)

for i, file in enumerate(files):
    if (("rov2" in file) and (".csv" in file)):
        pass
    else:
        files[i] = ''
files = list(filter(None, files))
    

for file in files:
    # Read data from csv file
    fig = plt.figure()
    
    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')


    df1 = pd.read_csv(folder+'/'+file)
    full_sec1 = full_sec(df1)
    ax.plot3D(df1['x'], df1['y'], df1['z'], 'gray')
    ax.plot3D(df1['x_ref'], df1['y_ref'], df1['z_ref'], 'red')
    #ax.plot3D(df1['x2'], df1['y2'], df1['z2'], 'green')
    if(n_agents > 1):
        file2 = file
        file2 = file2.replace("rov2", "rov3")
        df2 = pd.read_csv(folder+'/'+file2)
        full_sec2 = full_sec(df2)
        ax.plot3D(df2['x'], df2['y'], df2['z'], 'black')

    if(n_agents > 2):
        file3 = file
        file3 = file3.replace("rov2", "rov4")
        df3 = pd.read_csv(folder+'/'+file3)
        full_sec3 = full_sec(df3)
        ax.plot3D(df3['x'], df3['y'], df3['z'], 'blue')
    
    plot_name = "3D_"+file[43:].replace(".csv", ".png")
    plot_name = plot_name.replace("--rov2", "")
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        plt.show()

