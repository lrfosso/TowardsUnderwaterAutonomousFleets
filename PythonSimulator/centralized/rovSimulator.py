########## CENTRALIZED CONTROL
###### This is the main sim file, it imports the other file,
##### And simulates the system. It will produce a plot
#### and output 2 csv files, csv1 and csv2, both containing 
#### the the states of their respective rov at each time step
#### an animation can be made after running this file,
### with 3D_plot_test.py



import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd

import do_mpc

from rovModel import *

from rovController import *

modelRov = MyROVModel()

mpc = MyController(modelRov,0)
estimator = do_mpc.estimator.StateFeedback(modelRov.model)

simulator = do_mpc.simulator.Simulator(modelRov.model)

tvp_template = simulator.get_tvp_template()

#def tvp_fun(tvp_template,t_now):
#        tvp_template['x_sp'] = 0
#        tvp_template['y_sp'] = 0
#        tvp_template['z_sp'] = 0
#        tvp_template['phi_sp'] = 0
#        tvp_template['theta_sp'] = 0
#        tvp_template['psi_sp'] = 0
#        return tvp_template


simulator.set_tvp_fun(tvp_template)

params_simulator = {
    # Note: cvode doesn't support DAE systems.
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.1,

}

simulator.set_param(**params_simulator)

simulator.setup()

#x0 = np.array([20, -11.4, -1.5, 10, 20, 20, -10, 1,1,2,3,4]).reshape(-1,1)
############## INIT CONDITIONS ROV1, ROV2 ##################
#               x,y,z,phi,theta,psi,u,v,w,p,q,r
x0_1 = np.array([2, 3, 2, 0, 1/2, 0, 0, 0,0,0,0,0])
x0_2 = np.array([1, 0, -1, 0, 2/2, 0, 0, 0,0,0,0,0])

#Create one large init condition list for centralized MPC

x0 = np.array([])
x0 = np.append(x0,x0_1)
x0 = np.append(x0,x0_2)
x0 = x0.reshape(-1,1)
mpc.x0 = x0

estimator.x0 = x0
simulator.x0 = x0

mpc.mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

fig, ax = plt.subplots(3, sharex=True)
fig.align_ylabels


for g in [sim_graphics, mpc_graphics]:
    # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    g.add_line(var_type='_x', var_name='x_2', axis=ax[0])
    g.add_line(var_type='_x', var_name='y_2', axis=ax[0])
    g.add_line(var_type='_x', var_name='z_2', axis=ax[0])
    g.add_line(var_type='_x', var_name='u_2', axis=ax[0])
    g.add_line(var_type='_x', var_name='v_2', axis=ax[0])
    g.add_line(var_type='_x', var_name='w_2', axis=ax[0])
    
    g.add_line(var_type='_x', var_name='phi_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='theta_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='psi_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='p_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='q_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='r_2', axis=ax[2])

    g.add_line(var_type='_u', var_name='u_1_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_2_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_3_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_4_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_5_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_6_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_7_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_8_2', axis=ax[1])

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
for i in range(400):
    print(i)
    j += 1
    u0 = mpc.mpc.make_step(x0)

    y_next = simulator.make_step(u0)
    
    

    x0 = estimator.make_step(y_next)
    if (j == 50):
        mpc.x_setp += 5
    if (j  == 100):
        mpc.y_setp += 5
    if (j == 150):
        mpc.x_setp -= 5
    if (j == 200):
        mpc.y_setp -= 5
    x0_1 = x0[:12]
    x0_2 = x0[12:]
    plot1.append(x0_1)
    plot2.append(x0_2)


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
#####################################################################################




lines = (sim_graphics.result_lines['_x', 'x_2']+
        sim_graphics.result_lines['_x', 'y_2']+
        sim_graphics.result_lines['_x', 'z_2']+
        sim_graphics.result_lines['_x', 'u_2']+
        sim_graphics.result_lines['_x', 'v_2']+
        sim_graphics.result_lines['_x', 'w_2']
        )
ax[0].legend(lines,'xyzuvw',title='position')

lines = (sim_graphics.result_lines['_u', 'u_1_2']+
        sim_graphics.result_lines['_u', 'u_2_2']+
        sim_graphics.result_lines['_u', 'u_3_2']+
        sim_graphics.result_lines['_u', 'u_4_2']+
        sim_graphics.result_lines['_u', 'u_5_2']+
        sim_graphics.result_lines['_u', 'u_6_2']+
        sim_graphics.result_lines['_u', 'u_7_2']+
        sim_graphics.result_lines['_u', 'u_8_2']
        )

ax[1].legend(lines,'12345678',title='input')

lines = (sim_graphics.result_lines['_x', 'phi_2']+
        sim_graphics.result_lines['_x', 'theta_2']+
        sim_graphics.result_lines['_x', 'psi_2']+
        sim_graphics.result_lines['_x', 'p_2']+
        sim_graphics.result_lines['_x', 'q_2']+
        sim_graphics.result_lines['_x', 'r_2'])
ax[2].legend(lines,'φθψpqr',title='euler angles')
sim_graphics.plot_results()

sim_graphics.reset_axes()

plt.show()
