from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os

#----------------------------Settings----------------------------#

folder = "default_school_computer"

max_angle = 60
min_dist = 0.75

#----------------------------------------------------------------#

files = os.listdir(folder)

for i, file in enumerate(files):
    if ((".csv" in file)):
        pass
    else:
        files[i] = ''
files = list(filter(None, files))
    
circle = []
torus = []
line = []
spiral = []

for i, file in enumerate(files):
    if (("circle" in file[40:]) and (".csv" in file)):
        circle.append(file)
    elif (("torus" in file[40:]) and (".csv" in file)):
        torus.append(file)
    elif (("line" in file[40:]) and (".csv" in file)):
        line.append(file)
    elif (("spiral" in file[40:]) and (".csv" in file)):
        spiral.append(file)
    else:
        pass

df_circle = []
df_torus = []
df_line = []
df_spiral = []
df_test = []
#df_test.append(file)
for file in circle:
    df_circle.append(pd.read_csv(folder+"/"+file))

for file in torus:
    df_torus.append(pd.read_csv(folder+"/"+file))

for file in line:
    df_line.append(pd.read_csv(folder+"/"+file))

for file in spiral:
    df_spiral.append(pd.read_csv(folder+"/"+file))

print(len(df_line), len(df_spiral), len(df_circle), len(df_torus))

print("Circle:-------------------------------------------------")
n_circle_FOV = 0
bad_circle_FOV = 0
n_circle_Dist = 0
bad_circle_Dist = 0
for df in df_circle:
    if(max(df['angle2']) >= max_angle):
        bad_circle_FOV += 1
    n_circle_FOV += 1

    distance = []
    for i in range(len(df['x'])):
        distance.append(np.sqrt((df['x'][i]-df['real_x2'][i])**2+(df['y'][i]-df['real_y2'][i])**2+(df['z'][i]-df['real_z2'][i])**2))
    if(min(distance) <= min_dist):
        bad_circle_Dist += 1
    n_circle_Dist += 1

    
print("FOV: \t", round((bad_circle_FOV/n_circle_FOV)*100,2))
print("Dist: \t", round((bad_circle_Dist/n_circle_Dist)*100,2))

print("Torus:-------------------------------------------------")
n_torus_FOV = 0
bad_torus_FOV = 0
n_torus_Dist = 0
bad_torus_Dist = 0
for i, df in enumerate(df_torus):
    #print(df_test[i])
    if(max(df['angle2']) >= max_angle):
        bad_torus_FOV += 1
    n_torus_FOV += 1

    distance = []
    for i in range(len(df['x'])):
        distance.append(np.sqrt((df['x'][i]-df['real_x2'][i])**2+(df['y'][i]-df['real_y2'][i])**2+(df['z'][i]-df['real_z2'][i])**2))
    if(min(distance) <= min_dist):
        bad_torus_Dist += 1
    n_torus_Dist += 1


print("FOV: \t", round((bad_torus_FOV/n_torus_FOV)*100,2))
print("Dist: \t", round((bad_torus_Dist/n_torus_Dist)*100,2))



print("Line:-------------------------------------------------")
n_line_FOV = 0
bad_line_FOV = 0
n_line_Dist = 0
bad_line_Dist = 0
for df in df_line:
    if(max(df['angle2']) >= max_angle):
        bad_line_FOV += 1
    n_line_FOV += 1

    distance = []
    for i in range(len(df['x'])):
        distance.append(np.sqrt((df['x'][i]-df['real_x2'][i])**2+(df['y'][i]-df['real_y2'][i])**2+(df['z'][i]-df['real_z2'][i])**2))
    if(min(distance) <= min_dist):
        bad_line_Dist += 1
    n_line_Dist += 1


print("FOV: \t", round((bad_line_FOV/n_line_FOV)*100,2))
print("Dist: \t", round((bad_line_Dist/n_line_Dist)*100,2))


print("Spiral:-------------------------------------------------")
n_spiral_FOV = 0
bad_spiral_FOV = 0
n_spiral_Dist = 0
bad_spiral_Dist = 0
for df in df_spiral:
    if(max(df['angle2']) >= max_angle):
        bad_spiral_FOV += 1
    n_spiral_FOV += 1

    distance = []
    for i in range(len(df['x'])):
        distance.append(np.sqrt((df['x'][i]-df['real_x2'][i])**2+(df['y'][i]-df['real_y2'][i])**2+(df['z'][i]-df['real_z2'][i])**2))
    if(min(distance) <= min_dist):
        bad_spiral_Dist += 1
    n_spiral_Dist += 1


print("FOV: \t", round((bad_spiral_FOV/n_spiral_FOV)*100,2))
print("Dist: \t", round((bad_spiral_Dist/n_spiral_Dist)*100,2))

