from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

#----------------------------Settings----------------------------#
n_agents = 2
df1_name = '2agents1setpoint/data2.csv'
df2_name = '2agents1setpoint/data3.csv'
df3_name = '2agents1setpoint/data4.csv'
plot_name = '300DPI'
save_fig = True
display_fig = True
radius = 2.0

#----------------------------------------------------------------#

def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec
 
# Read data from csv file
df1 = pd.read_csv(df1_name)
if(n_agents == 2):
    df2 = pd.read_csv(df2_name)
if(n_agents == 3):
    df2 = pd.read_csv(df3_name)

# Convert from sec and nanosec to full seconds
full_sec1 = full_sec(df1)
full_sec2 = full_sec(df2)

fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')
 
 
# plotting
ax.plot3D(df1['x'], df1['y'], df1['z'], 'green')
ax.plot3D(df2['x'], df2['y'], df2['z'], 'blue')
ax.plot3D(df1['ref_x'], df1['ref_y'], df1['ref_z'], 'red')



if(save_fig): 
    now = datetime.now()
    dt_string = now.strftime("C%H_%M_%S_D%d_%m_%y")
    print(type(dt_string))
    print("Fig saved as: "+'Resulting_figures/'+plot_name+'_'+dt_string+'.png')
    plt.savefig('Resulting_figures/'+plot_name+'_'+dt_string+'.png', dpi=300)
if(display_fig):
    plt.show()


