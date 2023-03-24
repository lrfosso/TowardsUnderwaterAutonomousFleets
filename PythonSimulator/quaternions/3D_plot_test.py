import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import pandas as pd
from IPython import display
from mpl_toolkits.mplot3d import Axes3D

"""
Kilder: 
https://stackoverflow.com/questions/55169099/animating-a-3d-vector-with-matplotlib
https://matplotlib.org/stable/gallery/animation/random_walk.html
https://coderslegacy.com/python/save-animations-in-matplotlib/
"""

df = pd.read_csv("data1.csv")

x = list(df[df.columns[0]])
y = list(df[df.columns[1]])
z = list(df[df.columns[2]])
t = list(range(1, len(x)+1))
t = [float(i) for i in t]
global phi
global theta
phi = list(df[df.columns[3]]) #Yaw
theta = list(df[df.columns[4]]) #Pitch

df2 = pd.read_csv("data2.csv")

#print(df.head())
x2 = list(df2[df2.columns[0]])
y2 = list(df2[df2.columns[1]])
z2 = list(df2[df2.columns[2]])
t2 = list(range(1, len(x2)+1))
t2 = [float(i) for i in t2]
global phi2
global theta2
phi2 = list(df2[df2.columns[3]]) #Yaw
theta2 = list(df2[df2.columns[4]]) #Pitch


df = pd.DataFrame({"time": t ,"x" : x, "y" : y, "z" : z})
df2 = pd.DataFrame({"time": t2 ,"x" : x2, "y" : y2, "z" : z2})
#df = pd.DataFrame({"time": [1,2,3,4,5,6,7,8,9,10] ,"x" : [1,2,3,4,5,6,7,8,9,10], "y" : [1,2,3,4,5,6,7,8,9,10], "z" : [1,2,3,4,5,6,7,8,9,10]})
#print(df)

def get_arrow(theta):
    x = np.cos(theta)
    y = np.sin(theta)
    z = 0
    u = np.sin(2*theta)
    v = np.sin(3*theta)
    w = np.cos(3*theta)
    return x,y,z,u,v,w

def get_vector(phi, theta):
        u = np.cos(phi)*np.cos(theta)
        v = np.sin(phi)*np.cos(theta)
        w = np.sin(theta)
        return [u,v,w]

def update_graph(num):
    data=df[df['time']==num]
    data2=df2[df2['time']==num]
    global phi
    global theta
    global phi2
    global theta2
    global quiver
    global quiver2
    quiver.remove()
    quiver = ax.quiver(data.x,data.y,data.z,*get_vector(phi[num], theta[num]), length=3, normalize=True, color="red")
    quiver2.remove()
    quiver2 = ax.quiver(data2.x,data2.y,data2.z,*get_vector(phi2[num], theta2[num]), length=3, normalize=True, color="red")
    graph.set_data (data.x, data.y)
    graph.set_3d_properties(data.z)
    graph2.set_data (data2.x, data2.y)
    graph2.set_3d_properties(data2.z)
    liste = data.values.tolist()
    liste2 = data2.values.tolist()
    x = 0
    y = 0
    z = 0
    for var in liste:
        x = var[1]
        y = var[2]
        z = var[3]
    x2 = 0
    y2 = 0
    z2 = 0
    for var in liste2:
        x = var[1]
        y = var[2]
        z = var[3]
    title.set_text("MPC plot t: {} \nx1:{} y1:{} z1:{} \nx2:{} y2:{} z2:{}".format(num, f'{x:10.2f}', f'{y:10.2f}', f'{z:10.2f}', f'{x-1:10.2f}', f'{y-1:10.2f}', f'{z-1:10.2f}'))
    return title, graph, graph2


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

quiver = ax.quiver(1,1,1,1,1,1)
quiver2 = ax.quiver(1,1,1,1,1,1)

ax.set(xlim3d=(-10, 10), xlabel='X')
ax.set(ylim3d=(-10, 10), ylabel='Y')
ax.set(zlim3d=(-10, 10), zlabel='Z')

title = ax.set_title('MPC plot')


data=df[df['time']==0]
data2=df2[df2['time']==0]
graph, = ax.plot(data.x, data.y, data.z, linestyle="", marker="o")
graph2, = ax.plot(data2.x, data2.y, data2.z, linestyle="", marker="o", color="green")  

ani = animation.FuncAnimation(fig, update_graph, len(t), 
                               interval=10, blit=True)
#ani.save('myanimation.gif', writer='imagemagick', fps=20)
plt.show()

