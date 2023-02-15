#test
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *

import do_mpc

model_type = 'continuous'
model = do_mpc.model.Model(model_type)

#Params

m = 2 # ROV masse kg
r = 0.6 #radius fra massesetner m

J = m*r*r #ROV treghetsmoment
b = 1 #friksjonskoeffisient

#settpunkt
x_sp = 10 
y_sp = 20
z_sp = 30
theta_sp = np.pi


# states

x_m = model.set_variable('_x', 'x_m')
y_m = model.set_variable('_x', 'y_m')
z_m = model.set_variable('_x', 'z_m')
theta = model.set_variable('_x', 'theta')

dot_x_m = model.set_variable('_x', 'dot_x_m')
dot_y_m = model.set_variable('_x', 'dot_y_m')
dot_z_m = model.set_variable('_x', 'dot_z_m')
dot_theta = model.set_variable('_x', 'dot_theta')

# Input

u_1= model.set_variable('_u', 'u_1')
u_2= model.set_variable('_u', 'u_2')
u_3= model.set_variable('_u', 'u_3')
u_4= model.set_variable('_u', 'u_4')
u_5= model.set_variable('_u', 'u_5')
u_6= model.set_variable('_u', 'u_6')

#angle of thrusters u_1 to  u_4
theta_1= model.set_variable('_u', 'theta_1')
theta_2= model.set_variable('_u', 'theta_2')
theta_3= model.set_variable('_u', 'theta_3')
theta_4= model.set_variable('_u', 'theta_4')

# Algebraiske ligninger
#theta_1=theta_2=theta_3=theta_4=0 #placeholder


Fx = u_1*np.cos(theta_1)+u_2*cos(theta_2)+u_3*cos(theta_3)+u_4*cos(theta_4)
Fy = u_1*np.sin(theta_1)+u_2*sin(theta_2)+u_3*sin(theta_3)+u_4*sin(theta_4)
Fz = u_5+u_6

#Diff ligninger
dot_x_m = dot_x_m
model.set_rhs('x_m',dot_x_m)

dot_y_m = dot_y_m
model.set_rhs('y_m',dot_y_m)

dot_z_m = dot_z_m
model.set_rhs('z_m',dot_z_m)

dot_theta = dot_theta
model.set_rhs('theta',dot_theta)

model.set_rhs('dot_x_m',-b/m*dot_x_m + Fx/m)
model.set_rhs('dot_y_m',-b/m*dot_y_m + Fy/m)
model.set_rhs('dot_z_m',-b/m*dot_z_m + Fz/m)

model.set_rhs('dot_theta',(u_2+u_4-u_3-u_4)*r/J)


model.setup()

# mpc controller
def controller():
    global mpc
    mpc = do_mpc.controller.MPC(model)
    
    setup_mpc = {
            'n_horizon':20,
            't_step':0.1,
            'n_robust': 0,
            'store_full_solution': True,
    
            }
    
    mpc.set_param(**setup_mpc)
    
    #mterm = x_m**2 + y_m**2 + z_m**2 + theta**2 
    #lterm = x_m**2 + y_m**2 + z_m**2 + theta**2 
    _x = model.x
    mterm = (_x['x_m'] - x_sp)**2 + (_x['y_m'] - y_sp)**2 + (_x['z_m'] - z_sp)**2 + (_x['theta'] - theta_sp)**2 
    lterm = ((_x['x_m'] - x_sp)**2 + (_x['y_m'] - y_sp)**2 + (_x['z_m'] - z_sp)**2 + (_x['theta'] - theta_sp)**2 
            + (u_1**2 + u_2**2 + u_3**2 + u_4**2 + u_5**2 + u_6**2)
            + 0.001*(theta_1**2 + theta_2**2 + theta_3**2 + theta_4**2)
            )
    
    
    mpc.set_rterm(
            u_1 = 0.1,
            u_2 = 0.1,
            u_3 = 0.1,
            u_4 = 0.1,
            u_5 = 0.1,
            u_6 = 0.1,
            
            theta_1 = 0.001,
            theta_2 = 0.001,
            theta_3 = 0.001,
            theta_4 = 0.001
            )
    
    mpc.set_objective(mterm=mterm, lterm=lterm)
    
    mpc.bounds['lower','_u', 'u_1' ]= -2
    mpc.bounds['lower','_u', 'u_2' ]= -2
    mpc.bounds['lower','_u', 'u_3' ]= -2
    mpc.bounds['lower','_u', 'u_4' ]= -2
    mpc.bounds['lower','_u', 'u_5' ]= -2
    mpc.bounds['lower','_u', 'u_6' ]= -2
    mpc.bounds['lower','_u', 'theta_1' ]= -np.pi/2
    mpc.bounds['lower','_u', 'theta_2' ]= -np.pi/2
    mpc.bounds['lower','_u', 'theta_3' ]= -np.pi/2
    mpc.bounds['lower','_u', 'theta_4' ]= -np.pi/2
    
    mpc.bounds['upper','_u', 'u_1' ]= 2
    mpc.bounds['upper','_u', 'u_2' ]= 2
    mpc.bounds['upper','_u', 'u_3' ]= 2
    mpc.bounds['upper','_u', 'u_4' ]= 2
    mpc.bounds['upper','_u', 'u_5' ]= 2
    mpc.bounds['upper','_u', 'u_6' ]= 2
    mpc.bounds['upper','_u', 'theta_1' ]= np.pi/2
    mpc.bounds['upper','_u', 'theta_2' ]= np.pi/2
    mpc.bounds['upper','_u', 'theta_3' ]= np.pi/2
    mpc.bounds['upper','_u', 'theta_4' ]= np.pi/2
    
    
    
    
    
    mpc.setup()

controller()

#simulator

simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step = 0.1)


p_template = simulator.get_p_template()

simulator.setup()

estimator = do_mpc.estimator.StateFeedback(model)


#control loop

# x_m y_m z_m theta    dx_m dy_m dz_m dtheta

x0 = np.array([20, -11.4, -1.5, 10, 20, 20, -10, 1]).reshape(-1,1)

simulator.x0 = x0
mpc.x0 = x0

mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)


fig, ax = plt.subplots(3, sharex=True)
fig.align_ylabels

for g in [sim_graphics, mpc_graphics]:
    g.add_line(var_type='_x', var_name='x_m', axis=ax[0])
    g.add_line(var_type='_x', var_name='y_m', axis=ax[0])
    g.add_line(var_type='_x', var_name='z_m', axis=ax[0])
    g.add_line(var_type='_x', var_name='theta', axis=ax[0])

    #inputs
    g.add_line(var_type='_u', var_name='u_1', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_3', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_4', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_5', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_6', axis=ax[1])
    

    #angles motors

    g.add_line(var_type='_u', var_name='theta_1', axis=ax[2])
    g.add_line(var_type='_u', var_name='theta_2', axis=ax[2])
    g.add_line(var_type='_u', var_name='theta_3', axis=ax[2])
    g.add_line(var_type='_u', var_name='theta_4', axis=ax[2])


ax[0].set_ylabel('Position [XYZ]')
ax[1].set_ylabel('Actuation')
ax[2].set_xlabel('time')








u0 = np.zeros((10,1))

for i in range(400):
    u0 = mpc.make_step(x0)
    y_next= simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    if i == 200:
        x_sp = 0
        y_sp = -10
        controller()
        mpc.x0 = x0
        mpc.set_initial_guess()

lines = (sim_graphics.result_lines['_x', 'x_m'] +
         sim_graphics.result_lines['_x', 'y_m'] +
         sim_graphics.result_lines['_x', 'z_m'] +
         sim_graphics.result_lines['_x', 'theta'] 
         )
        
ax[0].legend(lines,'xyzt',title='position')

lines = (sim_graphics.result_lines['_u', 'u_1'] +
         sim_graphics.result_lines['_u', 'u_2'] +
         sim_graphics.result_lines['_u', 'u_3'] +
         sim_graphics.result_lines['_u', 'u_4'] +
         sim_graphics.result_lines['_u', 'u_5'] +
         sim_graphics.result_lines['_u', 'u_6']
         )

ax[1].legend(lines,'123456',title='u')

lines = (sim_graphics.result_lines['_u', 'theta_1'] +
         sim_graphics.result_lines['_u', 'theta_2'] +
         sim_graphics.result_lines['_u', 'theta_3'] +
         sim_graphics.result_lines['_u', 'theta_4'] 
  
         )
ax[2].legend(lines,'123456',title='theta')

sim_graphics.plot_results()
sim_graphics.reset_axes()
plt.show()








