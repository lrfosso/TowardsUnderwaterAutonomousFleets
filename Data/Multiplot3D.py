import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from mpl_toolkits import mplot3d

main_dir = "current_03"
dimensions = 2 #2 or 3

directories = os.listdir(main_dir)

def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

files = []
for directory in directories:
    files.append(os.listdir(main_dir+"/"+directory))

full_path_files = []

for i, list_of_file in enumerate(files):
    for file in list_of_file:
        print(file)
        file = main_dir+"/"+directories[i]+"/"+file
        print(file)
        full_path_files.append(file)

circle = []
torus = []
line = []
spiral = []

for i, file in enumerate(full_path_files):
    print(file)
    if (("circle" in file) and (".csv" in file)):
        circle.append(file)
    elif (("torus" in file) and (".csv" in file)):
        torus.append(file)
    elif (("line" in file) and (".csv" in file)):
        line.append(file)
    elif (("spiral" in file) and (".csv" in file)):
        spiral.append(file)
    else:
        pass
            
df_circle = []
df_torus = []
df_line = []
df_spiral = []
for file in circle:
    df_circle.append(pd.read_csv(file))

for file in torus:
    df_torus.append(pd.read_csv(file))

for file in line:
    df_line.append(pd.read_csv(file))

for file in spiral:
    df_spiral.append(pd.read_csv(file))

if(dimensions == 3):
    fig = plt.figure()

    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')

    for df in df_torus:
        ax.plot3D(df['x'], df['y'], df['z'], 'blue')
        ax.plot3D(df['x_ref'], df['y_ref'], df['z_ref'], 'red')

    plt.show()

if(dimensions == 2):
    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)

    for df in df_circle:
        full_sec1 = full_sec(df)
        ax1.plot(full_sec1, df['x'], 'red')
        ax1.plot(full_sec1, df['x_ref'], 'black')
        ax2.plot(full_sec1, df['y'], 'green')
        ax2.plot(full_sec1, df['y_ref'], 'black')
        ax3.plot(full_sec1, df['z'], 'blue')
        ax3.plot(full_sec1, df['z_ref'], 'black')

    plt.show()