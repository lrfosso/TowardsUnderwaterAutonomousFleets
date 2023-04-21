from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

#----------------------------Settings----------------------------#
n_agents = 3
df1_name = '2agents1setpoint/C10_57_32_D21_04_23data2.csv'
df2_name = '2agents1setpoint/C10_57_32_D21_04_23data3.csv'
df3_name = '2agents1setpoint/C10_57_32_D21_04_23data4.csv'
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
df1.columns = ['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test']

full_sec1 = full_sec(df1)
if(n_agents > 1):
    df2 = pd.read_csv(df2_name)
    df2.columns = ['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test']
    full_sec2 = full_sec(df2)
if(n_agents > 2):
    df3 = pd.read_csv(df3_name)
    df3.columns = ['x_ref','y_ref','z_ref','x','y','z','eta','e1','e2','e3','u','v','w','p','q','r','sec','nanosec','angle2','angle3','control_mode','std_test']
    full_sec3 = full_sec(df3)


#print(df1.head())
#print(df1.sec)
# Convert from sec and nanosec to full seconds


fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')

def x_torus(t):
    r = 1
    r_big = 5
    return r_big*np.cos(np.pi*(t*0.5)/150)+r*np.cos(np.pi*(t*0.5)/25)*np.cos(np.pi*(t*0.5)/150) -(r_big+r)
def y_torus(t):
    r = 1
    r_big = 3
    b = 3
    return r_big*np.sin(np.pi*(t*0.5)/150)+r*np.cos(np.pi*(t*0.5)/25)*np.sin(np.pi*(t*0.5)/150) 
def z_torus(t):
    r = 1
    r_big = 3
    b = 3
    return r*np.sin(np.pi*(t*0.5)/25) +5


def x_spiral(t):
    return (4-(t*0.5)*0.15)*np.cos(np.pi*(t*0.5)/(100-0.3*(t*0.5))) - 4

def y_spiral(t):
    return (4-(t*0.5)*0.15)*np.sin(np.pi*(t*0.5)/(100-0.3*(t*0.5)))

def z_spiral(t):
    return 5.0


def x_line(t):
    return 1.33226763*10**(-16)*t+1.20000000*10**(-2)*t**2-1.20000000*10**(-4)*t**3







t = int(240*2)
# plotting
#ax.plot3D([x_spiral(i) for i in range(t)], [y_spiral(i) for i in range(t)], [z_spiral(i) for i in range(t)], 'gray')

t0 = 0
avg_speed = 0.3
t1 = np.sqrt(10 **2)/avg_speed
dx0 = 0
dx1 = 0.3

x0 = 0
x1 = 10

A =np.array([[1,t0, t0**2, t0**3],
                    [0, 1, 2*t0, 3*t0**2],
                    [1, t1, t1**2, t1**3],
                    [0, 1, 2*t1, 3*t1**2]])
    
y = np.array([x0,dx0,x1,dx1]).reshape(-1,1)
b = (np.linalg.inv(A)@y).reshape(1,-1)
b = b[0]

print(b)




ax.plot3D(df1['x'], df1['y'], df1['z'], 'gray')
ax.plot3D(df2['x'], df2['y'], df2['z'], 'black')
ax.plot3D(df3['x'], df3['y'], df3['z'], 'blue')
ax.plot3D(df1['x_ref'], df1['y_ref'], df1['z_ref'], 'red')




if(save_fig): 
    now = datetime.now()
    dt_string = now.strftime("C%H_%M_%S_D%d_%m_%y")
    print(type(dt_string))
    print("Fig saved as: "+'Resulting_figures/'+plot_name+'_'+dt_string+'.png')
    plt.savefig('Resulting_figures/'+plot_name+'_'+dt_string+'.png', dpi=300)
if(display_fig):
    plt.show()


