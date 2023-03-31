import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import pandas as pd
from IPython import display
from mpl_toolkits.mplot3d import Axes3D
import math

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

df1 = pd.read_csv("quart1.csv")

q_1 = list(df1[df1.columns[0]])
e1_1 = list(df1[df1.columns[1]])
e2_1 = list(df1[df1.columns[2]])
e3_1 = list(df1[df1.columns[3]])


df2 = pd.read_csv("quart2.csv")

q_2 = list(df2[df2.columns[0]])
e1_2 = list(df2[df2.columns[1]])
e2_2 = list(df2[df2.columns[2]])
e3_2 = list(df2[df2.columns[3]])

df3 = pd.read_csv("quart_sp1.csv")
q_1_sp = list(df3[df3.columns[0]])
e1_1_sp = list(df3[df3.columns[1]])
e2_1_sp = list(df3[df3.columns[2]])
e3_1_sp = list(df3[df3.columns[3]])

df4 = pd.read_csv("quart_sp2.csv")
q_2_sp = list(df4[df4.columns[0]])
e1_2_sp = list(df4[df4.columns[1]])
e2_2_sp = list(df4[df4.columns[2]])
e3_2_sp = list(df4[df4.columns[3]])

print(q_2)


fig, ax = plt.subplots(nrows=2, ncols=2)
ax[0,0].plot(q_1)
ax[0,0].plot(q_1_sp)
ax[0,0].set_title('q_1')
ax[0,1].plot(e1_1)
ax[0,1].plot(e1_1_sp)
ax[0,1].set_title('e1_1')
ax[1,0].plot(e2_1)
ax[1,0].plot(e2_1_sp)
ax[1,0].set_title('e2_1')
ax[1,1].plot(e3_1)
ax[1,1].plot(e3_1_sp)
ax[1,1].set_title('e3_1')

plt.show()



rpy = []
for i in range(len(q_1)):
    rpy.append(euler_from_quaternion(e1_1[i], e2_1[i], e3_1[i], q_1[i]))

for i in range(len(q_1)):
    for j in range(3):
        rpy[i][j] = rpy[i][j]*180/np.pi

for i in range(len(rpy)):
    rpy[i] = [float(rpy[i][j]) for j in range(len(rpy[i]))]
data = [list(rpy[i]) for i in range(len(rpy))]
df = pd.DataFrame(data, columns=['roll','pitch','yaw'])
df.to_csv('rpy1.csv', index=False)


