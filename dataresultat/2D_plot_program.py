import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from datetime import datetime
import os


#----------------------------Settings----------------------------#
n_agents = 2  #DENNE ER IKKE TILPASSET 3 AGENT ENDA
folder = "disturbances/plus_minus_3"
save_fig = False
display_fig = True
FOV_max = 60
distance_ROVs = 1
title = "Disturbance +/-3m:"

#----------------------------Functions----------------------------#
def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

def xyz_plot(df1, df2, save_fig, display_fig, type_fig, title):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    fig.suptitle(title+' Position '+type_fig)

    ax1.set(ylabel='x [m]')
    ax1.plot(full_sec1, df1['x_ref'], 'black', label='Ref x')
    ax1.plot(full_sec1, df1['x'], 'b', label='x1')
    ax1.plot(full_sec2, df2['x'], 'g', label='x2')
    ax1.legend(loc='upper left')
    ax1.grid()

    ax2.set(ylabel='y [m]')
    ax2.plot(full_sec1, df1['y_ref'], 'black', label='Ref y')
    ax2.plot(full_sec1, df1['y'], 'b', label='y1')
    ax2.plot(full_sec2, df2['y'], 'g', label='y2')
    ax2.legend(loc='upper left')
    ax2.grid()

    ax3.set(ylabel='z [m]', xlabel='Time [s]')
    ax3.plot(full_sec1, df1['z_ref'], 'black', label='Ref z')
    ax3.plot(full_sec1, df1['z'], 'b', label='z1')
    ax3.plot(full_sec2, df2['z'], 'g', label='z2')
    ax3.legend(loc='upper left')
    ax3.grid()


    plot_name = "2D_xyz_"+type_fig
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        print("Displaying figure:",plot_name)
        plt.show()

def angle_plot(df1,df2, save_fig, display_fig, type_fig, FOV_max, title):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, ax = plt.subplots()

    ax.plot(full_sec1, df1['angle2'],'b', label='FOV angle 1')
    ax.plot(full_sec2, df2['angle2'],'g', label='FOV angle 2')
    ax.plot(full_sec1, [FOV_max for i in range(len(full_sec1))], 'r', label='FOV limit')
    ax.legend(loc='upper left')

    ax.set(xlabel='Time [s]', ylabel='Angle [deg]',title=title+' FOV angle '+type_fig)
    ax.grid()

    plot_name = "2D_FOV_"+type_fig
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        print("Displaying figure:",plot_name)
        plt.show()

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


def distance_rovs(df1,df2, save_fig, display_fig, type_fig, title, distance_ROVs):
    #### FUNKER IKKE
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, ax = plt.subplots()
    interpo_list_x = interpolate(full_sec1, df1['x'], full_sec2, df2['x'])
    interpo_list_y = interpolate(full_sec1, df1['y'], full_sec2, df2['y'])
    interpo_list_z = interpolate(full_sec1, df1['z'], full_sec2, df2['z'])
    #for i in range(len(interpo_list[1])):
    #    print("{}\t{}\t{}\t{}\t{}\t{}".format(interpo_list[0][i], interpo_list[1][i], full_sec1[i], df1['x'][i], full_sec2[i], df2['x'][i]))

    if(interpo_list_x[2]==1):
        ax.plot(full_sec2, [np.sqrt((interpo_list_x[1][i]-df2['x'][i])**2+(interpo_list_y[1][i]-df2['y'][i])**2+(interpo_list_z[1][i]-df2['z'][i])**2) for i in range(len(interpo_list_x[1]))], 'r', label='Distance')
        ax.plot(full_sec2, [distance_ROVs for i in range(len(full_sec2))], 'b', label='Ideal distance')
    elif(interpo_list_x[2]==2):
        ax.plot(full_sec1, [np.sqrt((interpo_list_x[1][i]-df1['x'][i])**2+(interpo_list_y[1][i]-df1['y'][i])**2+(interpo_list_z[1][i]-df1['z'][i])**2) for i in range(len(interpo_list_x[1]))], 'r', label='Distance')
        ax.plot(full_sec1, [distance_ROVs for i in range(len(full_sec1))], 'b', label='Ideal distance')
    ax.legend(loc='upper left')
    ax.set(xlabel='time (s)', ylabel='Distance [m]',title=title+' Distance ROVs '+type_fig)
    ax.set_ylim(0, 5)
    ax.grid()



    plot_name = "2D_Dist_ROV_"+type_fig
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        print("Displaying figure:", plot_name)
        plt.show()

def distance_from_ref(df1,df2, save_fig, display_fig, type_fig, title):
    full_sec1 = full_sec(df1)
    full_sec2 = full_sec(df2)

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)

    fig.suptitle(title+' Distance between ROV and reference '+type_fig)

    ax1.plot(full_sec1, abs(df1['x']-df1['x_ref']), 'r', label='Δx')
    ax1.plot(full_sec1, abs(df1['y']-df1['y_ref']), 'g', label='Δy')
    ax1.plot(full_sec1, abs(df1['z']-df1['z_ref']), 'b', label='Δz')
    ax1.set(ylabel='Distance [m]', title="ROV 1")
    ax1.legend(loc='upper left')
    ax1.grid()

    ax2.plot(full_sec2, abs(df2['x']-df2['x_ref']), 'r', label='Δx')
    ax2.plot(full_sec2, abs(df2['y']-df2['y_ref']), 'g', label='Δy')
    ax2.plot(full_sec2, abs(df2['z']-df2['z_ref']), 'b', label='Δz')
    ax2.set(xlabel='Time [s]', ylabel='Distance [m]', title="ROV 2")
    ax2.legend(loc='upper left')
    ax2.grid()

    plot_name = "2D_Distance_Ref_"+type_fig
    if(save_fig):
        print("Saving figure as:",plot_name)
        plt.savefig((folder+"/"+plot_name), dpi=300)
    if(display_fig):
        print("Displaying figure:", plot_name)
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
    if("circle" in file):
        type_fig = "circle"
    elif("torus" in file):
        type_fig = "torus"
    elif("spiral" in file):
        type_fig = "spiral"
    elif("line" in file):
        type_fig = "line"
    else:
        type_fig = "unknown"


    df1 = pd.read_csv(folder+'/'+file, encoding='utf-8', engine='python')

    file2 = file
    file2 = file2.replace("rov2", "rov3")
    df2 = pd.read_csv(folder+'/'+file2, encoding='utf-8', engine='python')

    xyz_plot(df1, df2, save_fig, display_fig, type_fig, title)
    angle_plot(df1, df2, save_fig, display_fig, type_fig, FOV_max, title)
    distance_rovs(df1, df2, save_fig, display_fig, type_fig, title, distance_ROVs)
    distance_from_ref(df1, df2, save_fig, display_fig,type_fig, title)

    


