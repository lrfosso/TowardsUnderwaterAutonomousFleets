from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

#----------------------------Settings----------------------------#
n_agents = 2
df1_name = '2agents1setpoint/C18_34_42_D20_04_23data2.csv'
df2_name = '2agents1setpoint/C18_34_42_D20_04_23data3.csv'
df3_name = '2agents1setpoint/data4.csv'
plot_name = '300DPI'
save_fig = False
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
    full_sec1 = full_sec(df2)
if(n_agents == 3):
    df2 = pd.read_csv(df3_name)
    full_sec2 = full_sec(df2)

#print(df1.head())
#print(df1.sec)
# Convert from sec and nanosec to full seconds


fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')

def x(t):
    r = 1
    r_big = 5
    return r_big*np.cos(np.pi*t/150)+r*np.cos(np.pi*t/25)*np.cos(np.pi*t/150) -(r_big+r)
def y(t):
    r = 1
    r_big = 3
    b = 3
    return r_big*np.sin(np.pi*t/150)+r*np.cos(np.pi*t/25)*np.sin(np.pi*t/150) 
def z(t):
    r = 1
    r_big = 3
    b = 3
    return r*np.sin(np.pi*t/25) +5

print(x(0), y(0), z(0))
t = 10000
# plotting
#ax.plot3D([x(i) for i in range(t)], [y(i) for i in range(t)], [z(i) for i in range(t)], 'gray')

ax.plot3D(df1['x'], df1['y'], df1['z'], 'gray')
ax.plot3D(df2['x'], df2['y'], df2['z'], 'black')
ax.plot3D(df1['x_ref'], df1['y_ref'], df1['z_ref'], 'red')




if(save_fig): 
    now = datetime.now()
    dt_string = now.strftime("C%H_%M_%S_D%d_%m_%y")
    print(type(dt_string))
    print("Fig saved as: "+'Resulting_figures/'+plot_name+'_'+dt_string+'.png')
    plt.savefig('Resulting_figures/'+plot_name+'_'+dt_string+'.png', dpi=300)
if(display_fig):
    plt.show()


