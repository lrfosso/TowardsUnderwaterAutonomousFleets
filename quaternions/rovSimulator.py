
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd
from scipy.spatial.transform import Rotation

import do_mpc

from rovModel import *

from rovController import *


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


modelRov1 = MyROVModel()
modelRov2 = MyROVModel()

mpc1 = MyController(modelRov1,modelRov2,2,[10,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
mpc2 = MyController(modelRov2,modelRov1, 2)
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
x0_1 = np.array([2, 3, 2, 1, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)
x0_2 = np.array([1, 4, -1, 1, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)

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


u0_1 = np.zeros((8,1))
u0_2 = np.zeros((8,1))
j = 0
for i in range(100):
    print('###############################' + i + '################################')

    u0_1 = mpc1.mpc.make_step(x0_1)
    u0_2 = mpc2.mpc.make_step(x0_2)

    y_next_1 = simulator1.make_step(u0_1)
    y_next_2 = simulator2.make_step(u0_2)
    
    x0_1 = estimator1.make_step(y_next_1)
    x0_2 = estimator2.make_step(y_next_2)
   
    if(i == 0): #creates the parameters for the cubic polynomials
        cubic_path_params_x = trajectory_parameter_generation(x0_1[0],x0_1[7],2,0,i,i+100)
        cubic_path_params_y = trajectory_parameter_generation(x0_1[1],x0_1[8],5,0,i,i+100)
        cubic_path_params_z = trajectory_parameter_generation(x0_1[2],x0_1[9],-2,0,i,i+100)

    mpc1.x_setp = trajectory_generation(cubic_path_params_x,i)
    mpc1.y_setp = trajectory_generation(cubic_path_params_y,i)
    mpc1.z_setp = trajectory_generation(cubic_path_params_z,i)
    

   # mpc2.x_setp = x0_1[0]
   # mpc2.y_setp = x0_1[1]
   # mpc2.z_setp = x0_1[2]
   # mpc2.phi_setp = x0_1[3]
   # mpc2.theta_setp = x0_1[4]
   # mpc2.psi_setp = x0_1[5]

    x0_1_euler = np.copy(x0_1)
    rot = Rotation.from_quat(x0_1_euler[4:8].reshape(1,-1))
    x0_1_euler[4:7] = rot.as_euler('xyz').reshape(-1,1)
    x0_1_euler = np.delete(x0_1_euler, 8, 0)


    x0_2_euler = np.copy(x0_2)
    rot = Rotation.from_quat(x0_2_euler[4:8].reshape(1,-1))
    x0_2_euler[4:7] = rot.as_euler('xyz').reshape(-1,1)
    #print(x0_1_euler)
    x0_2_euler = np.delete(x0_2_euler, 8, 0)

    
    plot1.append(x0_1_euler)
    plot2.append(x0_2_euler)


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
