
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd
from scipy.spatial.transform import Rotation

import do_mpc

from rovModel import *

from rovController import *

sample_time = 0.1

#def quat_to_euler(q_0, e_1, e_2, e_3):
#    #normalized quaternions
#    sinr_cosp = 2 * (q_0 * e_1 * e_2 * e_3)
#    cosr_cosp = 1 - 2 * (e_1**2  + e_2**2)
#    roll = np.arctan2(sinr_cosp, cosr_cosp)
#
#    sinp = np.sqrt(1 + 2 * (q_0 * e_2 - e_1 * e_3))
#    cosp = np.sqrt(1 - 2 * (q_0 * e_2 - e_1 * e_3))
#    pitch = 2 * np.arctan2(sinp, cosp) - np.pi/2
#
#def euler_to_quat(roll, pitch, yaw):
#    cr = np.cos(roll * 0.5)
#    sr = np.sin(roll * 0.5)
#    cp = np.cos(pitch * 0.5)
#    sp = np.sin(pitch* 0.5)
#    cy = np.cos(yaw * 0.5)
#    sy = np.sim(yaw * 0.5)
#    
#    q_0 = cr * cp * cy + sr * sp * sy
#    e_1 = sr * cp * cy - cr * sp * sy
#    e_2 = cr * sp * cy + sr * cp * sy
#    e_3 = cr * cp * sy - sr * sp * cy
#    return [q_0, e_1, e_2, e_3]

def get_odom(odom):
    #Write this function such that it collects the latest odometry published
    return odom

def send_u(u):
    print(u)

def sine_wave(t):
    z = 1*np.sin(np.pi*t/10)
    x = t/10
    y = 0
    return x,y,z 

#Creates parameters for cubic polynomial path
def trajectory_parameter_generation(x0,dx0,x1,dx1, t0, t1):
    
    A =np.array([[1,t0, t0**2, t0**3],
                [0, 1, 2*t0, 3*t0**2],
                [1, t1, t1**2, t1**3],
                [0, 1, 2*t1, 3*t1**2]])

    #b = np.array = ([a0,a1,a2,a3]).reshape(-1,1)
    y = np.array([x0,dx0,x1,dx1]).reshape(-1,1)
    b = ((np.linalg.inv(A)@y).reshape(1,-1))[0]
    print(b)
    return b
#Takes in the parameters as a list, and time t, returns the setpoint at that given time.
def trajectory_generation(a,t):

    qt = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    return qt

def init_path():
    x=[]
    y=[]
    z=[]
    u=[]
    v=[]
    w=[]
    t=[]
    n_waypoints = int(input("Enter number of waypoints: "))
    for i in range(n_waypoints):
        inp = input("enter waypoint " +str(i+1)+": x y z u v w t: ")
        inp = list(map(int, inp.split()))

        x.append(inp[0])
        y.append(inp[1])
        z.append(inp[2])
        u.append(inp[3])
        v.append(inp[4])
        w.append(inp[5])
        t.append(inp[6])

    return x,y,z,u,v,w,t    
        
modelRov1 = MyROVModel()

mpc1 = MyController(modelRov1,2,[10,0,0,1,0,0,0,0,0,0,0,0,0])
estimator1 = do_mpc.estimator.StateFeedback(modelRov1.model)


simulator1 = do_mpc.simulator.Simulator(modelRov1.model)
tvp_template1 = simulator1.get_tvp_template()

simulator1.set_tvp_fun(tvp_template1)

params_simulator = {
    # Note: cvode doesn't support DAE systems.
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': sample_time,

}

simulator1.set_param(**params_simulator)

simulator1.setup()

x0_1 = np.array([0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)

mpc1.x0 = x0_1


mpc1.mpc.set_initial_guess()
""" 





    #print('###############################' + str(i) + '################################')
    
    u0_1 = mpc1.mpc.make_step(x0_1)
    send_u(u0_1)
    #y_next_1 = simulator1.make_step(u0_1)
    
    #x0_1 = estimator1.make_step(y_next_1) #replace with odometry vector
    x0_1 = get_odom()
       
    mpc1.x_setp = 1
    mpc1.y_setp = 2
    mpc1.z_setp = 3
 """
