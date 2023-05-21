"""
_______________________________________________________________________________
This file creates a plot for the comparing the median of the different scenarios
How to use: 1. place the folders with the csvs in folderList
            2. change test df_"test" to circle, torus line or spiral. on line 270 and 272
_______________________________________________________________________________
"""

import matplotlib.pyplot as plt
import statistics
import pandas as pd
import numpy as np
import os
from mpl_toolkits import mplot3d
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning) #ignore depriciation warning
plt.rcParams.update({'font.size': 18})#plt font size
main_dir = "x05addedmass"
dimensions = 1 #2 or 3
plot = 1 # 1 circle 2 torus 3 line 4 spiral
suptitle = 'Spiral (0.5 Added Mass)'
figname = 'spiral_x05_added_mass'
directories = os.listdir(main_dir)

def interpolate_point(x, full_sec, values):
    if x == 0:
        return values[0]
    bigger = []
    bigger_index = []
    smaller = []
    smaller_index = []
    check1 = False
    check2 = False 
    for i, element in enumerate(full_sec):
        if (x < element):
            bigger.append(element)
            bigger_index.append(i)
            check1 = True
        elif (x > element):
            smaller.append(element)
            smaller_index.append(i)
            check2 = True
    if(check1 and check2):
        top_value_index = full_sec.index(max(smaller))
        bottom_value_index = full_sec.index(min(bigger))
        return ((values[top_value_index]-values[bottom_value_index])/(full_sec[top_value_index]-full_sec[bottom_value_index]))*(x-full_sec[bottom_value_index])+values[bottom_value_index]
    else:
        return values[-1]


def interpolate(full_sec1, values1, full_sec2, values2):
    interpo_list = []
    test = True
    values1 = values1.tolist()
    values2 = values2.tolist()
    if(len(full_sec1)<len(full_sec2)):
        for i in range(len(full_sec1)):
            interpo_list.append(interpolate_point(full_sec1[i], full_sec2, values2))
        interpo_full_sec = full_sec1
        interpo_df_number = 2
    else:
        for i in range(len(full_sec2)):
            interpo_list.append(interpolate_point(full_sec2[i], full_sec1, values1))
        interpo_full_sec = full_sec2
        interpo_df_number = 1
    return interpo_full_sec, interpo_list, interpo_df_number

def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

def distance_rovs(df1,df2):

    full_sec1 = df1['time'].values.tolist()
    full_sec2 = df2['time'].values.tolist()

    fig, ax = plt.subplots()
    interpo_list_x = interpolate(full_sec1, df1['x'], full_sec2, df2['x'])
    interpo_list_y = interpolate(full_sec1, df1['y'], full_sec2, df2['y'])
    interpo_list_z = interpolate(full_sec1, df1['z'], full_sec2, df2['z'])
    #for i in range(len(interpo_list[1])):
    #    print("{}\t{}\t{}\t{}\t{}\t{}".format(interpo_list[0][i], interpo_list[1][i], full_sec1[i], df1['x'][i], full_sec2[i], df2['x'][i]))

    if(interpo_list_x[2]==1):
        distance = [np.sqrt((interpo_list_x[1][i]-df2['x'][i])**2+(interpo_list_y[1][i]-df2['y'][i])**2+(interpo_list_z[1][i]-df2['z'][i])**2) for i in range(len(interpo_list_x[1]))]
    elif(interpo_list_x[2]==2):
        distance = [np.sqrt((interpo_list_x[1][i]-df1['x'][i])**2+(interpo_list_y[1][i]-df1['y'][i])**2+(interpo_list_z[1][i]-df1['z'][i])**2) for i in range(len(interpo_list_x[1]))]
    return distance
def dist_betw_rovs(x1,x2,y1,y2,z1,z2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

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


print(df_spiral[0])
print(df_spiral[1])
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
        #full_sec1 = full_sec(df)
        full_sec1 = df['time']
        ax1.plot(full_sec1, df['x'], 'red')
        ax1.plot(full_sec1, df['x_ref'], 'black')
        ax2.plot(full_sec1, df['y'], 'green')
        ax2.plot(full_sec1, df['y_ref'], 'black')
        ax3.plot(full_sec1, df['z'], 'blue')
        ax3.plot(full_sec1, df['z_ref'], 'black')

    plt.show()
if(dimensions == 1):
    x  = []
    y  = []
    z  = []

    dist_betw_rovs = []
    dist_betw_rovs_worst_high = []
    dist_betw_rovs_worst_low = []
    
    x_90_high = []
    x_90_low = []

    y_90_high = []
    y_90_low = []

    z_90_high = []
    z_90_low = []
    
    angle2_90_high = []
    angle2_90_low = []

    angle2 = []
    x_worst_high = []
    y_worst_high = []
    z_worst_high = []
    angle2_worst_high = []

    x_worst_low = []
    y_worst_low = []
    z_worst_low = []
    angle2_worst_low = []
    angle2 = []

    times = []
    x_ref  = []
    y_ref  = []
    z_ref  = []

    x2 = []
    y2 = []
    z2 = []

    x1 = []
    y1 = []
    z1 = []

    dist_min = 999999999
    dist_max = 0

    dist_high = []
    dist_low = []

    x1_worst = []
    y1_worst = []
    z1_worst = []

    x2_worst = []
    y2_worst = []
    z2_worst = []

    rov1_dfs = []
    rov2_dfs = []

    df_stats = []
    df_stat =  pd.DataFrame(columns=['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test','x2','y2','z2','time'])
    for time in df_spiral[0]['time']:
        df_stat =  pd.DataFrame(columns=['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test','x2','y2','z2','time'])
        for df in df_spiral:
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

        #Find best case and worst case
        x_worst_low.append(dfn['x'].min())
        y_worst_low.append(dfn['y'].min())
        z_worst_low.append(dfn['z'].min())
        angle2_worst_low.append(dfn['angle2'].min())
        

        x_worst_high.append(dfn['x'].max())
        y_worst_high.append(dfn['y'].max())
        z_worst_high.append(dfn['z'].max())
        angle2_worst_high.append(dfn['angle2'].max())
        
        #Find 90th percentile
        x_90_high.append(dfn['x'].quantile(0.9))
        x_90_low.append(dfn['x'].quantile(0.1))

        y_90_high.append(dfn['y'].quantile(0.9))
        y_90_low.append(dfn['y'].quantile(0.1))

        z_90_high.append(dfn['z'].quantile(0.9))
        z_90_low.append(dfn['z'].quantile(0.1))

        angle2_90_high.append(dfn['angle2'].quantile(0.9))
        angle2_90_low.append(dfn['angle2'].quantile(0.1))
        #Find distance between ROVs 
        #x2.append(dfn['x2'].median())
        #y2.append(dfn['y2'].median())
        #z2.append(dfn['z2'].median())
        rov1_dfs.append(dfn.loc[dfn['rov_id'] == 2])
        rov2_dfs.append(dfn.loc[dfn['rov_id'] == 3])



        x1.append(dfn.loc[dfn['rov_id'] == 2]['x'].median())
        x2.append(dfn.loc[dfn['rov_id'] == 3]['x'].median())
                                                    
        y1.append(dfn.loc[dfn['rov_id'] == 2]['y'].median())
        y2.append(dfn.loc[dfn['rov_id'] == 3]['y'].median())
                                                    
        z1.append(dfn.loc[dfn['rov_id'] == 2]['z'].median())
        z2.append(dfn.loc[dfn['rov_id'] == 3]['z'].median())

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
        dist_high.append(dist_max)
        dist_low.append(dist_min)
        dist_max = 0
        dist_min = 999999

        dist_betw_rovs.append(np.sqrt((x1[-1]-x2[-1])**2 + (y1[-1]-y2[-1])**2 + (z1[-1]-z2[-1])**2))

        dist_betw_rovs_worst_high = []
        dist_betw_rovs_worst_low = []

    fig, ax = plt.subplots(3,2,sharey=False,sharex=True)

    plt.subplots_adjust(#left=0.1, #Adjust spacing
                    #bottom=0.1,
                    #right=0.9,
                    #top=0.9,
                    #wspace=0.4,
                    hspace=0.25)
    fig.set_size_inches(26.5,16)
    ax[0,0].plot(times, x_ref, 'black', label='Reference')
    ax[0,0].plot(times, x, 'royalblue', label='Median')
    ax[0,0].plot(times, x_worst_high, 'red', label='High')
    ax[0,0].plot(times, x_worst_low, 'red', label='Low')
    ax[0,0].plot(times,x_90_high, 'orange', label='90th percentile')
    ax[0,0].plot(times,x_90_low, 'orange', label='10th percentile')
    ax[0,0].fill_between(times, x_worst_high, x_90_high, alpha=0.2, color='red')
    ax[0,0].fill_between(times, x_worst_low, x_90_low, alpha=0.2, color='red')
    ax[0,0].fill_between(times, x_90_high, x_90_low, alpha=0.2, color='orange')
    ax[0,0].title.set_text('Position')
    #ax[0,0].legend()
    ax[0,0].set_ylabel('x [m]')

    fig.suptitle(suptitle, fontsize=24)

    ax[1,0].plot(times, y, 'royalblue', label='y')
    ax[1,0].plot(times, y_ref, 'black', label='y Ref')
    ax[1,0].plot(times, y_worst_high, 'red', label='y High')
    ax[1,0].plot(times, y_worst_low, 'red', label='y Low')
    ax[1,0].plot(times,y_90_high, 'orange', label='90th percentile')
    ax[1,0].plot(times,y_90_low, 'orange', label='10th percentile')
    ax[1,0].fill_between(times, y_worst_high, y_90_high, alpha=0.2, color='red')
    ax[1,0].fill_between(times, y_worst_low, y_90_low, alpha=0.2, color='red')
    ax[1,0].fill_between(times, y_90_high, y_90_low, alpha=0.2, color='orange')
    #ax[1,0].legend()
    ax[1,0].set_ylabel('y [m]')

    ax[2,0].plot(times, z, 'royalblue', label='z')
    ax[2,0].plot(times, z_ref, 'black', label='z Ref')
    ax[2,0].plot(times, z_worst_high, 'red', label='z High')
    ax[2,0].plot(times, z_worst_low, 'red', label='z Low')
    ax[2,0].plot(times,z_90_high, 'orange', label='90th percentile')
    ax[2,0].plot(times,z_90_low, 'orange', label='10th percentile')
    ax[2,0].fill_between(times, z_worst_high, z_90_high, alpha=0.2, color='red')
    ax[2,0].fill_between(times, z_worst_low, z_90_low, alpha=0.2, color='red')
    ax[2,0].fill_between(times, z_90_high, z_90_low, alpha=0.2, color='orange')
    ax[2,0].set_ylabel('z [m]')
    ax[2,0].set_xlabel('Time [s]')
    #ax[2,0].legend()

    ax[0,1].plot(times, angle2, 'royalblue', label='Angle')
    #ax[0,1].plot(times, [60]*len(x), 'green', label='Hard Constraint')
    ax[0,1].plot(times, angle2_worst_high, 'red', label='Angle High')
    ax[0,1].plot(times, angle2_worst_low, 'red', label='Angle Low')
    ax[0,1].plot(times, angle2_90_high, 'orange', label='90th percentile')
    ax[0,1].plot(times, angle2_90_low, 'orange', label='10th percentile')
    ax[0,1].fill_between(times, angle2_worst_high, angle2_90_high, alpha=0.2, color='red')
    ax[0,1].fill_between(times, angle2_90_high, angle2_90_low, alpha=0.2, color='orange')
    ax[0,1].fill_between(times, angle2_90_low, angle2_worst_low, alpha=0.2, color='red')
    ax[0,1].title.set_text('Angle between ROVs')
    #ax[0,1].legend()
    ax[0,1].set_ylabel('Angle [$^\circ$]')
    ## TO be Implemented
    ## Distance between ROVs (with hard constraint)

    d = 0
    dist_med = []
    dist_90_high = []
    dist_90_low = []
    print(distss)
    first_run = True
    while d in range(len(distss[d])): #Iterate though dists in distss
        y = [] #Temporary y axis file
        for i in range(len(distss)): #Take the i-th index for every item in distss and append to y 
            if first_run:
                dist_med.append(np.median(distss[i]))#Find median at evey step
                dist_90_high.append(np.quantile(distss[i],0.9))
                dist_90_low.append(np.quantile(distss[i], 0.1))
            y.append(distss[i])
        first_run = False
        #ax5.plot(times, y,'red') #plot y
        #print(distss[i])
        d+=1

    dist_med_med = statistics.median(dist_med) #median of median distances between rovs
    ax[1,1].plot(times, dist_med, 'royalblue', label='Median')
    #ax[1,1].plot(times, [2]*len(dist_med), 'black', label='Ref')
    ax[1,1].plot(times, dist_high,'red', label='High')
    ax[1,1].plot(times, dist_low,'red', label='Low')
    ax[1,1].plot(times, dist_90_high,'orange', label='90th percentile')
    ax[1,1].plot(times, dist_90_low, 'orange', label='10th percentile')
    ax[1,1].fill_between(times, dist_high, dist_90_high, alpha=0.2, color='red')
    ax[1,1].fill_between(times, dist_low, dist_90_low, alpha=0.2, color='red')
    ax[1,1].fill_between(times, dist_90_high, dist_90_low, alpha=0.2, color='orange')
    ax[1,1].title.set_text('Distance between ROVs')
    ax[1,1].set_xlabel('Time [s]')
    ax[1,1].set_ylabel('Distance [m]')
    #plt.setp(ax[1,1].get_xticklabels(), visible=True)
    ax[1,1].xaxis.set_tick_params(labelbottom=True)
    ax[1,1].set_ylim([0,5])
   # ax[1,1].legend()
    fig.delaxes(ax[-1,-1])
    #ax[-1,-1].axis('off')
    lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    purged_labels = []
    purged_lines = []
    for i, label in enumerate(labels):
        if((label == 'Median' or label == '90th percentile' or label == '10th percentile' or label == 'Hard Constraint' or label == 'High' or label == 'Low' or label == 'Reference') and label not in purged_labels):
            purged_labels.append(label)
            purged_lines.append(lines[i])
    fig.legend(purged_lines, purged_labels,loc='upper left',ncol=4, bbox_to_anchor=(0.525,0.34))
    ## Distance to REF?
    plt.savefig(figname+'.pdf', format="pdf", bbox_inches='tight')
    plt.show()
