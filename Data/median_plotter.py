"""
_______________________________________________________________________________
This file creates a plot for the comparing the median of the different scenarios
How to use: 1. place the folders with the csvs in folderList
            2. change test df_"test" to circle, torus line or spiral.
_______________________________________________________________________________
"""

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
plt.rcParams.update({'font.size': 18})

#suptitle = 'Comparison of Spiral Test in Gazebo on Different Hardware and Python (Default)'
suptitle = 'Disturbances, Line Test'
#figname = 'spiral_default_pc_comparison'
figname = 'line_disturbances'
#folderList = ['default_lau_pc','default_tor_computer','default_school_computer','Python_sim_standardstest'] #Default comparisons
folderList = ['damping 2x', 'Damping x05', 'x2addedmass','linje_x05_added_mass', 'packetloss_70', 'waves_05','waves_025','mass_2x']
files = []
full_path_files = []
x_trajectories = []
y_trajectories = []
z_trajectories = []
angle2_trajectories = []
dist_trajectories = []
times_trajectories = []
for folder in folderList:
    files.append(os.listdir(folder))
    for i, list_of_file in enumerate(files):
        for file in list_of_file:
            file = folder+"/"+file
            full_path_files.append(file)
    
    circle = []
    torus = []
    line = []
    spiral = []
    
    for i, file in enumerate(full_path_files):
        print(file)
        if (("torus" in file) and (".csv" in file)):
            torus.append(file)
        elif (("circle" in file) and (".csv" in file)):
            circle.append(file)
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
        if("rov2" in file):
            rov_id = [2]*len(df_circle[-1]['x'])
            df_circle[-1]['rov_id'] = rov_id
        elif("rov3" in file):
            rov_id = [3]*len(df_circle[-1]['x'])
            df_circle[-1]['rov_id'] = rov_id
    
    for file in torus:
        df_torus.append(pd.read_csv(file))
    
        if("rov2" in file):
            rov_id = [2]*len(df_torus[-1]['x'])
            df_torus[-1]['rov_id'] = rov_id
        elif("rov3" in file):
            rov_id = [3]*len(df_torus[-1]['x'])
            df_torus[-1]['rov_id'] = rov_id
    
        
    for file in line:
        df_line.append(pd.read_csv(file))
    
        if("rov2" in file):
            rov_id = [2]*len(df_line[-1]['x'])
            df_line[-1]['rov_id'] = rov_id
    
        elif("rov3" in file):
            rov_id = [3]*len(df_line[-1]['x'])
            df_line[-1]['rov_id'] = rov_id
    
    
    for file in spiral:
        df_spiral.append(pd.read_csv(file))
    
        if("rov2" in file):
            rov_id = [2]*len(df_spiral[-1]['x'])
            df_spiral[-1]['rov_id'] = rov_id
    
        elif("rov3" in file):
            rov_id = [3]*len(df_spiral[-1]['x'])
            df_spiral[-1]['rov_id'] = rov_id

    print(len(df_circle))
    print(len(df_torus))
    print(len(df_line))
    print(len(df_spiral))
    x = []
    y = []
    z = []
    angle2 = []
    dist_betw_rovs = []
    times = []
    x_ref = []
    y_ref = []
    z_ref = []
    
    dist_min = 99999999
    dist_max = 0

    x1_worst = []
    x2_worst = []
    y1_worst = []
    y2_worst = []
    z1_worst = []
    z2_worst = []



    df_stats = []
    df_stat =  pd.DataFrame(columns=['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test','x2','y2','z2','time'])
    for time in df_line[0]['time']:
        df_stat =  pd.DataFrame(columns=['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test','x2','y2','z2','time'])
        for df in df_line:
            df_closest = df.iloc[(df['time']-time).abs().argsort()[:1]]
            df_stat = df_stat.append(df_closest)
        df_stats.append(df_stat)
    distss = []
    for dfn in df_stats:
        #Find median values
        x.append(dfn['x'].median())
        y.append(dfn['y'].median())
        z.append(dfn['z'].median())
        angle2.append(dfn['angle2'].median())
        times.append(dfn['time'].median())
        x_ref.append(dfn['x_ref'].median())
        y_ref.append(dfn['y_ref'].median())
        z_ref.append(dfn['z_ref'].median())

        x1_worst.append(dfn.loc[dfn['rov_id'] == 2]['x'])
        x2_worst.append(dfn.loc[dfn['rov_id'] == 2]['x2'])

        y1_worst.append(dfn.loc[dfn['rov_id'] == 2]['y'])
        y2_worst.append(dfn.loc[dfn['rov_id'] == 2]['y2'])

        z1_worst.append(dfn.loc[dfn['rov_id'] == 2]['z'])
        z2_worst.append(dfn.loc[dfn['rov_id'] == 2]['z2'])

        x1_worst[-1] = x1_worst[-1].values.tolist()
        x2_worst[-1] = x2_worst[-1].values.tolist()

        y1_worst[-1] = y1_worst[-1].values.tolist()
        y2_worst[-1] = y2_worst[-1].values.tolist()

        z1_worst[-1] = z1_worst[-1].values.tolist()
        z2_worst[-1] = z2_worst[-1].values.tolist()
 
        dists = np.array([])
        for i in range(len(x1_worst[-1])):
            dist = np.sqrt((x1_worst[-1][i]-x2_worst[-1][i])**2 + (y1_worst[-1][i]-y2_worst[-1][i])**2 + (z1_worst[-1][i]-z2_worst[-1][i])**2)
            dists = np.append(dists,[dist], axis=0)
            #print(x1_worst[-1][i][1])
            if (dist > dist_max):
                dist_max = dist
            if (dist < dist_min):
                dist_min = dist
        distss.append(dists)
        dist_max = 0
        dist_min = 999999

    d = 0
    dist_med = []
    print(distss)
    first_run = True
    while d in range(len(distss[d])): #Iterate though dists in distss
        y_temp = [] #Temporary y axis file
        for i in range(len(distss)): #Take the i-th index for every item in distss and append to y 
            if first_run:
                dist_med.append(np.median(distss[i]))#Find median at evey step
            y_temp.append(distss[i])
        first_run = False
        #ax5.plot(times, y,'red') #plot y
        #print(distss[i])
        d+=1
    

    x_trajectories.append(x)
    y_trajectories.append(y)
    z_trajectories.append(z)
    angle2_trajectories.append(angle2)
    dist_trajectories.append(dist_med)
    times_trajectories.append(times)
    files = []
    full_path_files = []





fig, ax = plt.subplots(3,2,sharey=False,sharex=True)

plt.subplots_adjust(#left=0.1, #Adjust spacing
                #bottom=0.1,
                #right=0.9,
                #top=0.9,
                #wspace=0.4,
                hspace=0.25)
fig.set_size_inches(26.5,16)
ax[0,0].plot(times_trajectories[-1], x_ref, 'black', label='Reference')
#ax[0,0].plot(times_trajectories[0], x_trajectories[0], 'red', label='PC 1 (Gazebo)')
#ax[0,0].plot(times_trajectories[1], x_trajectories[1], 'green', label='PC 2 (Gazebo)')
#ax[0,0].plot(times_trajectories[2], x_trajectories[2], 'blue', label='PC 3 (Gazebo)')
#ax[0,0].plot(times_trajectories[3], x_trajectories[3], 'orange', label='Python Sim')

ax[0,0].plot(times_trajectories[0], x_trajectories[0], 'red', label='2x Damping')
ax[0,0].plot(times_trajectories[1], x_trajectories[1], 'green', label='0.5x Damping')
ax[0,0].plot(times_trajectories[2], x_trajectories[2], 'blue', label='2x Added Mass')
ax[0,0].plot(times_trajectories[3], x_trajectories[3], 'orange', label='0.5x Added Mass')
ax[0,0].plot(times_trajectories[4], x_trajectories[4], 'cyan', label='70% Packet Loss')
ax[0,0].plot(times_trajectories[5], x_trajectories[5], 'purple', label='0.5 m/s Waves')
ax[0,0].plot(times_trajectories[6], x_trajectories[6], 'pink', label='0.25 m/s Waves')
ax[0,0].plot(times_trajectories[7], x_trajectories[7], 'grey', label='2x Mass')
#ax[0,0].legend()
ax[0,0].set_ylabel('x [m]')

fig.suptitle(suptitle, fontsize=24)

ax[1,0].plot(times_trajectories[-1], y_ref, 'black', label='Ref')
#ax[1,0].plot(times_trajectories[0], y_trajectories[0], 'red', label='PC 1')
#ax[1,0].plot(times_trajectories[1], y_trajectories[1], 'green', label='PC 2')
#ax[1,0].plot(times_trajectories[2], y_trajectories[2], 'blue', label='PC 3')
#ax[1,0].plot(times_trajectories[3], y_trajectories[3], 'orange', label='Python Sim')

ax[1,0].plot(times_trajectories[0], y_trajectories[0], 'red', label='2x Damping')
ax[1,0].plot(times_trajectories[1], y_trajectories[1], 'green', label='0.5x Damping')
ax[1,0].plot(times_trajectories[2], y_trajectories[2], 'blue', label='2x Added Mass')
ax[1,0].plot(times_trajectories[3], y_trajectories[3], 'orange', label='0.5x Added Mass')
ax[1,0].plot(times_trajectories[4], y_trajectories[4], 'cyan', label='70% Packet Loss')
ax[1,0].plot(times_trajectories[5], y_trajectories[5], 'purple', label='0.5 m/s Waves')
ax[1,0].plot(times_trajectories[6], y_trajectories[6], 'pink', label='0.25 m/s Waves')
ax[1,0].plot(times_trajectories[7], y_trajectories[7], 'grey', label='2x Mass')

#ax[1,0].legend()
ax[1,0].set_ylabel('y [m]')
#ax[2,0].plot(times_trajectories[0], z_trajectories[0], 'red', label='PC 1')
#ax[2,0].plot(times_trajectories[1], z_trajectories[1], 'green', label='PC 2')
#ax[2,0].plot(times_trajectories[2], z_trajectories[2], 'blue', label='PC 3')
ax[2,0].plot(times_trajectories[-1], z_ref, 'black', label='z Ref')
#ax[2,0].plot(times_trajectories[3], z_trajectories[3], 'orange', label='Python Sim')


ax[2,0].plot(times_trajectories[0], z_trajectories[0], 'red', label='2x Damping')
ax[2,0].plot(times_trajectories[1], z_trajectories[1], 'green', label='0.5x Damping')
ax[2,0].plot(times_trajectories[2], z_trajectories[2], 'blue', label='2x Added Mass')
ax[2,0].plot(times_trajectories[3], z_trajectories[3], 'orange', label='0.5x Added Mass')
ax[2,0].plot(times_trajectories[4], z_trajectories[4], 'cyan', label='70% Packet Loss')
ax[2,0].plot(times_trajectories[5], z_trajectories[5], 'purple', label='0.5 m/s Waves')
ax[2,0].plot(times_trajectories[6], z_trajectories[6], 'pink', label='0.25 m/s Waves')
ax[2,0].plot(times_trajectories[7], z_trajectories[7], 'grey', label='2x Mass')

ax[2,0].set_ylabel('z [m]')
ax[2,0].set_xlabel('Time [s]')
#ax[2,0].legend()

#ax[0,1].plot(times_trajectories[0], angle2_trajectories[0], 'red', label='PC 1')
#ax[0,1].plot(times_trajectories[1], angle2_trajectories[1], 'green', label='PC 2')
#ax[0,1].plot(times_trajectories[2], angle2_trajectories[2], 'blue', label='PC 3')
#ax[0,1].plot(times_trajectories[3], angle2_trajectories[3], 'orange', label='Python Sim')

ax[0,1].plot(times_trajectories[0], angle2_trajectories[0], 'red', label='2x Damping')
ax[0,1].plot(times_trajectories[1], angle2_trajectories[1], 'green', label='0.5x Damping')
ax[0,1].plot(times_trajectories[2], angle2_trajectories[2], 'blue', label='2x Addded Mass')
ax[0,1].plot(times_trajectories[3], angle2_trajectories[3], 'orange', label='0.5x Added Mass')
ax[0,1].plot(times_trajectories[4], angle2_trajectories[4], 'cyan', label='70% Packet Loss')
ax[0,1].plot(times_trajectories[5], angle2_trajectories[5], 'purple', label='0.5 m/s Waves')
ax[0,1].plot(times_trajectories[6], angle2_trajectories[6], 'pink', label='0.25 m/s Waves')
ax[0,1].plot(times_trajectories[7], angle2_trajectories[7], 'grey', label='2x Mass')

ax[0,1].title.set_text('Angle between ROVs')
#ax[0,1].legend()
ax[0,1].set_ylabel('Angle [$^\circ$]')

#ax[1,1].plot(times_trajectories[0], dist_trajectories[0], 'red', label='PC 1')
#ax[1,1].plot(times_trajectories[1], dist_trajectories[1], 'green', label='PC 2')
#ax[1,1].plot(times_trajectories[2], dist_trajectories[2], 'blue', label='PC 3')
#ax[1,1].plot(times_trajectories[3], dist_trajectories[3], 'orange', label='Python Sim')


ax[1,1].plot(times_trajectories[0], dist_trajectories[0], 'red', label='2x Damping')
ax[1,1].plot(times_trajectories[1], dist_trajectories[1], 'green', label='0.5x Damping')
ax[1,1].plot(times_trajectories[2], dist_trajectories[2], 'blue', label='2x Addded Mass')
ax[1,1].plot(times_trajectories[3], dist_trajectories[3], 'orange', label='0.5x Added Mass')
ax[1,1].plot(times_trajectories[4], dist_trajectories[4], 'cyan', label='70% Packet Loss')
ax[1,1].plot(times_trajectories[5], dist_trajectories[5], 'purple', label='0.5 m/s Waves')
ax[1,1].plot(times_trajectories[6], dist_trajectories[6], 'pink', label='0.25 m/s Waves')
ax[1,1].plot(times_trajectories[7], dist_trajectories[7], 'grey', label='2x Mass')

ax[1,1].title.set_text('Distance between ROVs')
ax[1,1].set_xlabel('Time [s]')
ax[1,1].set_ylabel('Distance [m]')
#plt.setp(ax[1,1].get_xticklabels(), visible=True)
ax[1,1].xaxis.set_tick_params(labelbottom=True)
ax[1,1].set_ylim([0,5])
#ax[1,1].legend()
fig.delaxes(ax[-1,-1])
#ax[-1,-1].axis('off')
lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
purged_labels = []
purged_lines = []
#folderList = ['damping 2x', 'Damping x05', 'x2_added_mass','x05addedmass', 'packetloss_70', 'waves_05']
for i, label in enumerate(labels):
    #if((label == 'PC 1 (Gazebo)' or label == 'PC 2 (Gazebo)' or label == 'PC 3 (Gazebo)' or label == 'Python Sim' or label == 'Low' or label == 'Reference') and label not in purged_labels): #Default
    if((label == '2x Damping' or label == '0.5x Damping' or label == '2x Added Mass' or label == '0.5x Added Mass' or label == '70% Packet Loss' or label == '0.5 m/s Waves' or label == 'Reference' or label == '2x Mass' or label == '0.25 m/s Waves') and label not in purged_labels): #Disturbance
        purged_labels.append(label)
        purged_lines.append(lines[i])
fig.legend(purged_lines, purged_labels,loc='upper left',ncol=4, bbox_to_anchor=(0.525,0.34))

#xcorr = scipy.signal.signaltools.correlate(x,x_ref)
#recoverd_time_shift = times[xcorr.argmax()]
#print(recovered_time_shift)

plt.savefig(figname+'.pdf', format="pdf", bbox_inches='tight')
plt.show()


