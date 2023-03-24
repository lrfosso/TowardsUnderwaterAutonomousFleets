import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
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
     
        return roll_x, pitch_y, yaw_z # in radians

def calculate_quaternion(current_position, desired_position):
    # Calculate the vector pointing from current position to desired position
    vector = np.array(desired_position) - np.array(current_position)
    vector = vector / np.linalg.norm(vector)

    # Calculate the quaternion angles
    theta = np.arccos(np.dot([0, 0, 1], vector))
    axis = np.cross([0, 0, 1], vector)
    axis = axis / np.linalg.norm(axis)

    w = np.cos(theta/2)
    x = axis[0] * np.sin(theta/2)
    y = axis[1] * np.sin(theta/2)
    z = axis[2] * np.sin(theta/2)

    return (w, x, y, z)

def x_directional_vector_from_quaternion(q0, e1, e2, e3):
    x = 1-(2*e2**2+2*e3**2)
    y = 2*e1*e2+2*e3*q0
    z = 2*e1*e3-2*e2*q0
    return [x, y, z]

def y_directional_vector_from_quaternion(q0, e1, e2, e3):
    x = 2*(e1*e3 - e2*q0)
    y = 1-(2*e1**2+2*e3**2)
    z = (2*e2*e3 + 2*e1*q0)
    return [x, y, z]
def z_directional_vector_from_quaternion(q0, e1, e2, e3):
    x = 2*(e1*e2 + e3*q0)
    y = 2*(e2*e3 - e1*q0)
    z = 1-(2*e1**2+2*e2**2)
    return [x, y, z]
def vector_between_rovs(x1,y1,z1,x2,y2,z2):
    x = (x2-x1)
    y = (y2-y1)
    z = (z2-z1)
    return [x, y, z]

""" #+200*(-
                #(((1-(2*_x_rov1['e_2']**2))+(2*_x_rov1['e_3']**2))*(_tvp_rov1['x_2']-_x_rov1['x']))
                #+ (((2*_x_rov1['e_1']*_x_rov1['e_2'])+(2*_x_rov1['e_3']*_x_rov1['q_0']))*(_tvp_rov1['y_2']-_x_rov1['y']))
                #+ (((2*_x_rov1['e_1']*_x_rov1['e_3'])-(2*_x_rov1['e_2']*_x_rov1['q_0']))*(_tvp_rov1['z_2']-_x_rov1['z'])))**2
                # self.mpc.set_nl_cons("FOV", 
        #(-_x_rov1['x'])
        #, 0)"""

### VAR ###
x1 = 0
y1 = 0
z1 = 0
q0ref = 0
e1ref = 0
e2ref = 0
e3ref = 1

x2 = -4
y2 = 0
z2 = 0


######################################################################################

v1 = vector_between_rovs(x1, y1, z1, x2, y2, z2)
v2 = x_directional_vector_from_quaternion(q0ref, e1ref, e2ref, e3ref)
#
#
#
#angle = np.arccos((np.dot(v1, v2))/(np.linalg.norm(v1)*np.linalg.norm(v2)))
#print("Angle",angle)
######################################################################################
print("V1: ",v1)
print("V2: ",v2)
print("|V1|: ",np.linalg.norm(v1))
print("|V2|: ",np.linalg.norm(v2))
print("Dot: {}\nCosNorm: {}".format(-(np.dot(v1,v2)), -(np.cos(np.pi)*np.linalg.norm(v2))))
print("Dot2: {}\nCosNorm2: {}".format((-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)+(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)+(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1))), -(np.cos(np.pi)*np.linalg.norm(v2))))

#print("Uttrykk:", ((np.cos(np.pi)*np.linalg.norm(v2)*np.linalg.norm(v1))-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)+(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)+(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1))<0))

print("\nImpl Uttrykk:\n", (-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)+(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)+(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1))))

expr = (-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)+(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)+(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1)
        +np.cos(np.pi)*np.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)))
#print(expr)

def FOV(x1, y1, z1, q0, e1, e2, e3, x2, y2, z2):
    f1 = -np.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
    f2 = (1-(2*e2**2+2*e3**2))*(x2-x1)
    f3 = (2*e1*e2+2*e3*q0)*(y2-y1)
    f4 = (2*e1*e3-2*e2*q0)*(z2-z1)
    return [f1, f2, f3, f4]

#print(-sum(FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2)))


#print("f1= {}\nf2= {}\nf3= {}\nf4= {}\nsum= {}".format(FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2)[0], FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2)[1], FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2)[2], FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2)[3], sum(FOV(x1, y1, z1, q0ref, e1ref, e2ref, e3ref, x2, y2, z2))))
#print(calculate_quaternion([0,0,0], [4,0,0]))




fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlim3d(-4, 4)
ax.set_ylim3d(-4, 4)
ax.set_zlim3d(-4, 4)
vektorx = x_directional_vector_from_quaternion(q0ref, e1ref, e2ref, e3ref)
vektory = y_directional_vector_from_quaternion(q0ref, e1ref, e2ref, e3ref)
vektorz = z_directional_vector_from_quaternion(q0ref, e1ref, e2ref, e3ref)
ax.quiver(x1, y1, z1, vektorx[0], vektorx[1], vektorx[2], length = 0.5, normalize = True, color='r')
ax.quiver(x1, y1, z1, vektory[0], vektory[1], vektory[2], length = 0.5, normalize = True)
ax.quiver(x1, y1, z1, vektorz[0], vektorz[1], vektorz[2], length = 0.5, normalize = True)
#ax.quiver(0, 0, 0, vektor[0], vektor[1], vektor[2], scale=1, color='g')
ax.plot(x1, y1, z1, linestyle="", marker="o", color="blue", label="Agent1", markersize=9)
ax.plot(x2, y2, z2, linestyle="", marker="o", color="green", label="Agent2", markersize=9)  

plt.show()