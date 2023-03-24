
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd
from scipy.spatial.transform import Rotation

import do_mpc

from rovModel import *

from rovController import *
import math


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

def sine_wave(t):
    z = 1*np.sin(np.pi*t/10)
    x = t/10
    y = 0
    return x,y,z 

def vector_between_rovs(x1,y1,z1,x2,y2,z2):
    x = (x2-x1)
    y = (y2-y1)
    z = (z2-z1)
    return [x, y, z]

def x_directional_vector_from_quaternion(q0, e1, e2, e3):
    x = 1-(2*e2**2+2*e3**2)
    y = 2*e1*e2+2*e3*q0
    z = 2*e1*e3-2*e2*q0
    return [x, y, z]

def euler_from_quaternion(w, x, y, z):
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

#Creates parameters for cubic polynomial path
def trajectory_parameter_generation(x0,dx0,x1,dx1, t0, t1):
    
    A =np.array([[1,t0, t0**2, t0**3],
                [0, 1, 2*t0, 3*t0**2],
                [1, t1, t1**2, t1**3],
                [0, 1, 2*t1, 3*t1**2]])

    #b = np.array = ([a0,a1,a2,a3]).reshape(-1,1)
    y = np.array([x0,dx0,x1,dx1]).reshape(-1,1)
    b = np.linalg.inv(A)@y
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
modelRov2 = MyROVModel()

mpc1 = MyController(modelRov1,modelRov2,2,[1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0], nl_setting=True)
mpc2 = MyController(modelRov2,modelRov1,2,[2,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0], nl_setting=False)
estimator1 = do_mpc.estimator.StateFeedback(modelRov1.model)
estimator2 = do_mpc.estimator.StateFeedback(modelRov1.model)


simulator1 = do_mpc.simulator.Simulator(modelRov1.model)
simulator2 = do_mpc.simulator.Simulator(modelRov2.model)

tvp_template1 = simulator1.get_tvp_template()
tvp_template2 = simulator2.get_tvp_template()

#def tvp_fun(tvp_template,t_now):
#        tvp_template['x_sp'] = 0
#        tvp_template['y_sp'] = 0
#        tvp_template['z_sp'] = 0
#        tvp_template['phi_sp'] = 0
#        tvp_template['theta_sp'] = 0
#        tvp_template['psi_sp'] = 0
#        return tvp_template


simulator1.set_tvp_fun(tvp_template1)
simulator2.set_tvp_fun(tvp_template2)

params_simulator = {
    # Note: cvode doesn't support DAE systems.
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.1,

}

simulator1.set_param(**params_simulator)
simulator2.set_param(**params_simulator)

simulator1.setup()
simulator2.setup()

#x0 = np.array([20, -11.4, -1.5, 10, 20, 20, -10, 1,1,2,3,4]).reshape(-1,1)
#               x,y,z,q_0,e_1,e_2,e_3,u,v,w,p,q,r
x0_1 = np.array([1, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)
x0_2 = np.array([2, 0, 0, 1, 0, 1, 0, 0,0,0,0,0,0]).reshape(-1,1)

mpc1.x0 = x0_1
mpc2.x0 = x0_2

estimator1.x0 = x0_1
simulator1.x0 = x0_1

estimator2.x0 = x0_2
simulator2.x0 = x0_2

mpc1.mpc.set_initial_guess()
mpc2.mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc1.mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator1.data)

fig, ax = plt.subplots(3, sharex=True)
fig.align_ylabels


for g in [sim_graphics, mpc_graphics]:
    # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    g.add_line(var_type='_x', var_name='x', axis=ax[0])
    g.add_line(var_type='_x', var_name='y', axis=ax[0])
    g.add_line(var_type='_x', var_name='z', axis=ax[0])
    g.add_line(var_type='_x', var_name='u', axis=ax[0])
    g.add_line(var_type='_x', var_name='v', axis=ax[0])
    g.add_line(var_type='_x', var_name='w', axis=ax[0])
    
    g.add_line(var_type='_x', var_name='q_0', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_1', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_3', axis=ax[2])
    g.add_line(var_type='_x', var_name='p', axis=ax[2])
    g.add_line(var_type='_x', var_name='q', axis=ax[2])
    g.add_line(var_type='_x', var_name='r', axis=ax[2])

    g.add_line(var_type='_u', var_name='u_1', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_3', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_4', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_5', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_6', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_7', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_8', axis=ax[1])

    # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:


ax[0].set_ylabel('Position [m], velocity [m/s]')
ax[1].set_ylabel('Input [N]')
ax[2].set_ylabel('Angle [rad]')

#u0 = mpc.make_step(x0)
#y_next = simulator.make_step(u0)

plot1 = []
plot2 = []

nl_cons_state = []

u0_1 = np.zeros((8,1))
u0_2 = np.zeros((8,1))

#r = init_path()


#x_waypoints = []
#y_waypoints = []
#z_waypoints = []
#u_waypoints = []
#v_waypoints = []
#w_waypoints = []
#t_waypoints = []#

#x_waypoints.append(x0_1[0][0])
#y_waypoints.append(x0_1[1][1])
#z_waypoints.append(x0_1[2][2])
#u_waypoints.append(x0_1[7][7])
#v_waypoints.append(x0_1[8][8])
#w_waypoints.append(x0_1[9][9])
#t_waypoints.append(0)

#x_waypoints=r[0]
#y_waypoints=r[1]
#z_waypoints=r[2]
#u_waypoints=r[3]
#v_waypoints=r[4]
#w_waypoints=r[5]
#t_waypoints=r[6]
#j = 0
for i in range(300):
    print(x0_1)
    print(x0_2)
    u0_1 = mpc1.mpc.make_step(x0_1)
    u0_2 = mpc2.mpc.make_step(x0_2)

    y_next_1 = simulator1.make_step(u0_1)
    y_next_2 = simulator2.make_step(u0_2)
    
    x0_1 = estimator1.make_step(y_next_1)
    x0_2 = estimator2.make_step(y_next_2)

    mpc1.x_2 = x0_2[0]
    mpc1.y_2 = x0_2[1]
    mpc1.z_2 = x0_2[2]

    mpc2.x_2 = x0_1[0]
    mpc2.y_2 = x0_1[1]
    mpc2.z_2 = x0_1[2]


    if i == 100:
        mpc2.x_setp -= 3
        mpc2.y_setp += 3
        mpc1.x_setp += 3
        mpc1.y_setp -= 3
    if i == 150:
        mpc2.x_setp += 5
        mpc2.y_setp -= 5
    if i == 200:
        mpc2.x_setp -= 7
        mpc2.y_setp += 7


    q0ref = float(x0_1[3])
    e1ref = float(x0_1[4])
    e2ref = float(x0_1[5])
    e3ref = float(x0_1[6])
    x1 = float(x0_1[0])
    y1 = float(x0_1[1])
    z1 = float(x0_1[2])
    x2 = float(x0_2[0])
    y2 = float(x0_2[1])
    z2 = float(x0_2[2])

    for k in range(35):
        utrk1 = (-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)
                +(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)
                +(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1)))
        #utrk1 = (-
        #        (((1-(2*e2ref**2))+(2*e3ref**2))*(x2-x1))
        #        + (((2*e1ref*e2ref)+(2*e3ref*q0ref))*(y2-y1))
        #        + (((2*e1ref*e3ref)-(2*e2ref*q0ref))*(z2-z1)))
        #print("\t\t\t\t\t\t\t\t\tUttrykk:\t", (-((1-(2*e2ref**2+2*e3ref**2))*(x2-x1)+(2*e1ref*e2ref+2*e3ref*q0ref)*(y2-y1)+(2*e1ref*e3ref-2*e2ref*q0ref)*(z2-z1))<=0))

    v1 = vector_between_rovs(x1, y1, z1, x2, y2, z2)
    v2 = x_directional_vector_from_quaternion(q0ref, e1ref, e2ref, e3ref)

    angle = ((np.arccos((np.dot(v1, v2))/(np.linalg.norm(v1)*np.linalg.norm(v2))))/np.pi)*180
    for k in range(25):
        print("\t\t\t\t\t\t\t\t\tUttrykk:\t", utrk1,"\t", round(angle,3),"\titr:", i)

    nl_cons_state.append([round(angle,3),round(utrk1,3), (((angle>90 and utrk1>0) or (angle<90 and utrk1<=0))), i])



    x0_1_euler = np.copy(x0_1)
    x0_1_euler[3] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[0]
    x0_1_euler[4] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[1]
    x0_1_euler[5] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[2]
    x0_1_euler = np.delete(x0_1_euler, 6)



    x0_2_euler = np.copy(x0_2)
    x0_2_euler[3] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[0]
    x0_2_euler[4] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[1]
    x0_2_euler[5] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[2]
    x0_2_euler = np.delete(x0_2_euler, 6)

    
    plot1.append(x0_1_euler)
    plot2.append(x0_2_euler)

print(plot1)
print(nl_cons_state)

######################### DETTE ER FOR PLOT ########################################
for i in range(len(plot1)):
    plot1[i] = [float(plot1[i][j]) for j in range(len(plot1[i]))]
data = [list(plot1[i]) for i in range(len(plot1))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data1.csv', index=False)
print(df)
#####################################################################################
for i in range(len(plot2)):
    plot2[i] = [float(plot2[i][j]) for j in range(len(plot2[i]))]
data = [list(plot2[i]) for i in range(len(plot2))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data2.csv', index=False)
print(df)
###################################################################################
nl_cons_state
for i in range(len(nl_cons_state)):
    nl_cons_state[i] = [float(nl_cons_state[i][j]) for j in range(len(nl_cons_state[i]))]
data = [list(nl_cons_state[i]) for i in range(len(nl_cons_state))]
df = pd.DataFrame(data, columns=['angle', 'Nl value', 'match', 'itr'])
df = df.sort_values(by=['angle'])
df.to_csv('nl_con.csv', index=False)
print(df)



lines = (sim_graphics.result_lines['_x', 'x']+
        sim_graphics.result_lines['_x', 'y']+
        sim_graphics.result_lines['_x', 'z']+
        sim_graphics.result_lines['_x', 'u']+
        sim_graphics.result_lines['_x', 'v']+
        sim_graphics.result_lines['_x', 'w']
        )
ax[0].legend(lines,'xyzuvw',title='position')

lines = (sim_graphics.result_lines['_u', 'u_1']+
        sim_graphics.result_lines['_u', 'u_2']+
        sim_graphics.result_lines['_u', 'u_3']+
        sim_graphics.result_lines['_u', 'u_4']+
        sim_graphics.result_lines['_u', 'u_5']+
        sim_graphics.result_lines['_u', 'u_6']+
        sim_graphics.result_lines['_u', 'u_7']+
        sim_graphics.result_lines['_u', 'u_8']
        )

ax[1].legend(lines,'12345678',title='input')

lines = (sim_graphics.result_lines['_x', 'q_0']+
        sim_graphics.result_lines['_x', 'e_1']+
        sim_graphics.result_lines['_x', 'e_2']+
        sim_graphics.result_lines['_x', 'e_3'])
       # sim_graphics.result_lines['_x', 'p']+
       # sim_graphics.result_lines['_x', 'q']+
       # sim_graphics.result_lines['_x', 'r'])
ax[2].legend(lines,'0123pqr',title='quaternions')
sim_graphics.plot_results()

sim_graphics.reset_axes()

plt.show()
