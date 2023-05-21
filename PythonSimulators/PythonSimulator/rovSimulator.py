
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd

import do_mpc

from rovModel import *

from rovController import *

def yaw_setpoint(direction_vector, old_yaw):
    """Returns the yaw setpoint based on the direction vector"""
    yaw = np.arctan2(direction_vector[1], direction_vector[0])
    return yaw, old_yaw

def torus(t):
    #Moves in a spiral shape around a torus of diameter 5
    r_big = 7
    r = 2
    theta = 35
    phi = 200
    x = r_big*cos(pi*t/phi)+r*cos(pi*t/theta)*cos(pi*t/phi) - (r_big+r)
    y = r_big*sin(pi*t/phi)+r*cos(pi*t/theta)*sin(pi*t/phi)
    z = r*sin(pi*t/theta) +5
    #z = 1*sin(pi*t/25)+2
    #x = 5*cos(t/100) + 1*cos(pi*t/25)
    #y = 5*sin(t/100) + 1*sin(pi*t/25)

    return x,y,z

def sine_wave(t):
        #Creates a sine wave in the z-axis, moves in a circle in the xy-plane 
        z = 1*sin(pi*t/25)+5
        x = 5*cos(pi*t/100) - 5
        y = 5*sin(pi*t/100)
        return x,y,z 

def line(t):
    #Moves in a straight line in the x-axis
    # [ 0.00000000e+00  1.33226763e-16  1.20000000e-02 -1.20000000e-04] 0.3
    # [ 0.00000000e+00  0.00000000e+00  5.33333333e-03 -3.55555556e-05] 0.25
    x = 5.33*10**(-3)*t**2-3.55*10**(-5)*t**3
    y = 0.0
    z = 5.0
    return x,y,z

def spiral(t):
    x = (4-0.015*t)*cos(pi*(t)/(100-0.3*(t))) - 4
    y = (4-0.015*t)*sin(pi*(t)/(100-0.3*(t)))
    z = 5.0
    return x,y,z

def get_vector(pitch, yaw):
    """Get directional vector from oriantation angles phi and theta"""
    u = np.cos(yaw)*np.cos(pitch)
    v = np.sin(yaw)*np.cos(pitch)
    w = np.sin(pitch)
    return [u,v,w]

def vector_between_rovs(x1,y1,z1,x2,y2,z2):
    """Gets xyz coords of ROVs as input. Returns the vector between them (- heave)"""
    x = (x2-x1)
    y = (y2-y1)
    z = (z2-z1)
    return [x, y, z]

modelRov1 = MyROVModel()
modelRov2 = MyROVModel()

mpc1 = MyController(modelRov1,modelRov2, 2)
mpc2 = MyController(modelRov2,modelRov1, 2)
estimator1 = do_mpc.estimator.StateFeedback(modelRov1.model)
estimator2 = do_mpc.estimator.StateFeedback(modelRov2.model)


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
#               x,y,z,phi(roll 3),theta(pitch 4),psi(yaw 5),u,v,w,p,q,r
x0_1 = np.array([0, 0, 5, 0, 0, 0, 0, 0,0,0,0,0]).reshape(-1,1)
x0_2 = np.array([2, 0, 5, 0, 0, -3.14, 0, 0,0,0,0,0]).reshape(-1,1)

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
    
    g.add_line(var_type='_x', var_name='phi', axis=ax[2])
    g.add_line(var_type='_x', var_name='theta', axis=ax[2])
    g.add_line(var_type='_x', var_name='psi', axis=ax[2])
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

setpoint1 = [[],[],[]]
setpoint2 = [[],[],[]]


u0_1 = np.zeros((8,1))
u0_2 = np.zeros((8,1))
j = 0
n_sims = 1300
function_line = True
firt_itr = True
rot1_count = 0
rot2_count = 0

for i in range(n_sims):

    for i in range(30):
        print("\t\t\t\t\t\t\t\t\t\t\t\t{}/{}".format(j,n_sims))
    j += 1


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

    if(function_line):
        mpc1.x_setp, mpc1.y_setp, mpc1.z_setp = line(j*0.1)
        mpc2.x_setp, mpc2.y_setp, mpc2.z_setp = line(j*0.1)
    else:
        mpc1.x_setp = 15
        mpc1.y_setp = 0
        mpc1.z_setp = 5
        mpc2.x_setp = 15
        mpc2.y_setp = 0
        mpc2.z_setp = 5
    if(mpc1.x_setp >= 15):
        function_line = False
        
    direction_vector1 = [float(x0_2[0] - x0_1[0]), float(x0_2[1] - x0_1[1]), float(x0_2[2] - x0_1[2])]
    direction_vector2 = [float(x0_1[0] - x0_2[0]), float(x0_1[1] - x0_2[1]), float(x0_1[2] - x0_2[2])]


    yaw1 = np.arctan2(direction_vector1[1], direction_vector1[0])# if np.arctan2(direction_vector1[1], direction_vector1[0]) > 0 else np.arctan2(direction_vector1[1], direction_vector1[0])+2*np.pi
    yaw2 = np.arctan2(direction_vector2[1], direction_vector2[0])# if np.arctan2(direction_vector2[1], direction_vector2[0]) > 0 else np.arctan2(direction_vector2[1], direction_vector2[0])+2*np.pi

    if firt_itr:
        yaw1_old = yaw1
        yaw2_old = yaw2
        firt_itr = False
 
    if yaw1 - yaw1_old < -np.pi:
        rot1_count += 1
    elif yaw1 - yaw1_old > np.pi:
        rot1_count -= 1

    if yaw2 - yaw2_old < -np.pi:
        rot2_count += 1
    elif yaw2 - yaw2_old > np.pi:
        rot2_count -= 1
    
    yaw1_out = yaw1 + rot1_count*2*np.pi #yaw_out is the correct yaw angle
    yaw2_out = yaw2 + rot2_count*2*np.pi #yaw_out is the correct yaw angle
    yaw1_old = yaw1
    yaw2_old = yaw2


    mpc1.psi_setp = yaw1_out
    mpc2.psi_setp = yaw2_out



    mpc1.theta_setp = np.arctan2(direction_vector1[2], np.sqrt(direction_vector1[0]**2 + direction_vector1[1]**2))
    mpc2.theta_setp = np.arctan2(direction_vector2[2], np.sqrt(direction_vector2[0]**2 + direction_vector2[1]**2))
    

    #if(j == 1):
    #    mpc1.psi_setp = 0
    #    mpc2.psi_setp = 0
    #if(j == 100):
    #    mpc1.psi_setp = np.pi
    #    mpc2.psi_setp = np.pi
    #if(j == 200):
    #    mpc1.psi_setp = 0
    #    mpc2.psi_setp = 0
    #if(j == 300):
    #    mpc1.psi_setp = -np.pi
    #    mpc2.psi_setp = -np.pi


    setpoint1[0].append(mpc1.x_setp)
    setpoint1[1].append(mpc1.y_setp)
    setpoint1[2].append(mpc1.z_setp)

    setpoint2[0].append(mpc2.x_setp)
    setpoint2[1].append(mpc2.y_setp)
    setpoint2[2].append(mpc2.z_setp)
    #angle2_to_3 = np.arccos()
    v1_1 = vector_between_rovs(x0_1[0].item(), x0_1[1].item(), x0_1[2].item(), x0_2[0].item(), x0_2[1].item(), x0_2[2].item())
    v2_1 = get_vector(x0_1[4].item(), x0_1[5].item())

    v1_2 = vector_between_rovs(x0_2[0].item(), x0_2[1].item(), x0_2[2].item(), x0_1[0].item(), x0_1[1].item(), x0_1[2].item())
    v2_2 = get_vector(x0_2[4].item(), x0_2[5].item())

    angle2_to_3 = round(((180*(np.arccos(np.dot(v1_1, v2_1)/(np.linalg.norm(v1_1)*np.linalg.norm(v2_1)))))/np.pi),2)
    angle3_to_2 = round(((180*(np.arccos(np.dot(v1_2, v2_2)/(np.linalg.norm(v1_2)*np.linalg.norm(v2_2)))))/np.pi),2)
    
    plot_data_1 = np.append(x0_1, (angle2_to_3, j*0.1, yaw1_out))
    plot_data_2 = np.append(x0_2, (angle3_to_2, j*0.1, yaw2_out))
    print(plot_data_1)
    plot1.append(plot_data_1)
    plot2.append(plot_data_2)


######################### DETTE ER FOR PLOT ########################################
for i in range(len(plot1)):
    plot1[i] = [float(plot1[i][j]) for j in range(len(plot1[i]))]
data = [list(plot1[i]) for i in range(len(plot1))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r','angle2', 'time', 'yaw_ref'])
df['x_ref'] = [float(a) for a in setpoint1[0]]
df['y_ref'] = [float(a) for a in setpoint1[1]]
df['z_ref'] = [float(a) for a in setpoint1[2]]
df.to_csv('data1.csv', index=False)
print(df)
#####################################################################################
for i in range(len(plot2)):
    plot2[i] = [float(plot2[i][j]) for j in range(len(plot2[i]))]
data = [list(plot2[i]) for i in range(len(plot2))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r', 'angle2', 'time', 'yaw_ref'])
df['x_ref'] = [float(a) for a in setpoint2[0]]
df['y_ref'] = [float(a) for a in setpoint2[1]]
df['z_ref'] = [float(a) for a in setpoint2[2]]
df.to_csv('data2.csv', index=False)
print(df)
#####################################################################################




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

lines = (sim_graphics.result_lines['_x', 'phi']+
        sim_graphics.result_lines['_x', 'theta']+
        sim_graphics.result_lines['_x', 'psi']+
        sim_graphics.result_lines['_x', 'p']+
        sim_graphics.result_lines['_x', 'q']+
        sim_graphics.result_lines['_x', 'r'])
ax[2].legend(lines,'φθψpqr',title='euler angles')
sim_graphics.plot_results()

sim_graphics.reset_axes()

plt.show()
