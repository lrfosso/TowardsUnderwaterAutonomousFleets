import matplotlib.pyplot as plt
import numpy as np
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

# Plot data
fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, sharex=True)

fig.suptitle('Position')
ax1.plot(full_sec1, df1['ref_x'], 'r', label='ref_x')
ax1.plot(full_sec1, df1['x'], 'b', label='x')
ax1.legend(loc='upper left')

ax2.plot(full_sec1, df1['ref_y'], 'r', label='ref_y')
ax2.plot(full_sec1, df1['y'], 'b', label='y')
ax2.legend(loc='upper left')

ax3.plot(full_sec1, df1['ref_z'], 'r', label='ref_z')
ax3.plot(full_sec1, df1['z'], 'b', label='z')
ax3.legend(loc='upper left')

ax4.plot(full_sec1, [radius for i in range(len(df['x']))], 'r', label='radius')
ax4.plot(full_sec1, [np.sqrt((df1['ref_x'][i]-df1['x'][i])**2 + (df1['ref_y'][i]-df1['y'][i])**2) for i in range(len(df['x']))], 'b', label='distance')
ax4.legend(loc='upper left')
ax4.set(xlabel='time (s)', ylabel='position (m)')

ax5.plot(full_sec2, [np.sqrt((df1['x'][i]-df2['x'][i])**2+(df1['y'][i]-df2['y'][i])**2+(df1['z'][i]-df2['z'][i])**2) for i in range(len(df2['x']))], 'r', label='Distanse')


if(save_fig):
    now = datetime.now()
    dt_string = now.strftime("C%H_%M_%S_D%d_%m_%y")
    print("Fig saved as: "+'Resulting_figures/'+plot_name+'_'+dt_string+'.png')
    plt.savefig('Resulting_figures/'+plot_name+'_'+dt_string+'.png', dpi=300)
if(display_fig):
    plt.show()
