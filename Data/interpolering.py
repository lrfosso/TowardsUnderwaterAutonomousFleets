import matplotlib.pyplot as plt
import pandas as pd
import time
import numpy as np

def full_sec(df):
    """Convert from sec and nanosec to full seconds"""
    full_sec = []
    init_time = df['sec'][0] + df['nanosec'][0]/1000000000
    for i, val in enumerate(df['sec']):
        full_sec.append((val + df['nanosec'][i]/1000000000)-init_time)
    return full_sec

df1 = pd.read_csv('weight_test/mass_1kg/Date--24--04--23--Time--15--03--58--mass_1kg--rov2.csv')
df2 = pd.read_csv('weight_test/mass_1kg/Date--24--04--23--Time--15--03--58--mass_1kg--rov3.csv')

full_sec1 = full_sec(df1)
full_sec2 = full_sec(df2)

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
        if(full_sec2[-1]>full_sec1[-1]):
            print("Spotted a problem2")
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


interpolerte_verdier = interpolate(full_sec1, df1['x'], full_sec2, df2['x'])

#if(interpolerte_verdier[2]==1):
#    plt.plot(full_sec2, df2['x'], 'b', label='2')
#    plt.plot(interpolerte_verdier[0], interpolerte_verdier[1], 'g', label='1')
#elif(interpolerte_verdier[2]==2):
#    plt.plot(full_sec1, df1['x'], 'r', label='1')
#    plt.plot(interpolerte_verdier[0], interpolerte_verdier[1], 'g', label='2')
#
#plt.legend()
#plt.show()


