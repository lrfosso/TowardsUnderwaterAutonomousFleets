import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd

import do_mpc

model_type = 'continuous'
model = do_mpc.model.Model(model_type)

m = 11.5                    # Kg
W = 112.8                   # Newton
#W = 114.8                   # Newton neutral buoyancy
B = 114.8                   # Newton
r_b = np.array([[0],        # m     
                [0],
                [0]])
r_g = np.array([[0],        # m
                [0],
                [0.02]])

z_g = r_g[2]
y_g = 0
x_g = 0
#inertia
I_x = 0.16                  # Kg m**2
I_y = 0.16                  # Kg m**2
I_z = 0.16                  # Kg m**2


X_u = -4.03                 # Ns/m
Y_v = -6.22                 # Ns/m
Z_w = -5.18
K_p = -0.07                 # Ns/m
M_q = -0.07                 # Ns/m
N_r = -0.07                 # Ns/m

#Quadratic dampenining
X_u_abs = -18.18            # Ns**2/m**2
Y_v_abs = -21.66            # Ns**2/m**2
Z_w_abs = -36.99            # Ns**2/m**2
K_p_abs = -1.55             # Ns**2/m**2
M_q_abs = -1.55             # Ns**2/m**2
N_r_abs = -1.55

#added mass
X_udot = -5.5               # Kg
Y_vdot = -12.7              # Kg
Z_wdot = -14.57             # Kg
K_pdot = -0.12              # Kg m**2/rad
M_qdot = -0.12              # Kg m**2/rad
N_rdot = -0.12              # Kg m**2/rad



#states
#pos
x = model.set_variable('_x', 'x')
y = model.set_variable('_x', 'y')
z = model.set_variable('_x', 'z')
#euler angles
phi = model.set_variable('_x','phi')
theta = model.set_variable('_x','theta')
psi = model.set_variable('_x','psi')
#lin vel
u = model.set_variable('_x', 'u')
v = model.set_variable('_x', 'v')
w = model.set_variable('_x', 'w')
#lin acc
u_dot = model.set_variable('_z', 'u_dot')
v_dot = model.set_variable('_z', 'v_dot')
w_dot = model.set_variable('_z', 'w_dot')
#ang vel
p = model.set_variable('_x', 'p')
q = model.set_variable('_x', 'q')
r = model.set_variable('_x', 'r')
#ang acc
p_dot = model.set_variable('_z', 'p_dot')
q_dot = model.set_variable('_z', 'q_dot')
r_dot = model.set_variable('_z', 'r_dot')

#u_vec = model.set_variable('_u', 'u_vec', shape=(8,1))
u_1 = model.set_variable('_u', 'u_1')
u_2 = model.set_variable('_u', 'u_2')
u_3 = model.set_variable('_u', 'u_3')
u_4 = model.set_variable('_u', 'u_4')
u_5 = model.set_variable('_u', 'u_5')
u_6 = model.set_variable('_u', 'u_6')
u_7 = model.set_variable('_u', 'u_7')
u_8 = model.set_variable('_u', 'u_8')
u_vec = np.transpose(np.array([u_1,u_2,u_3,u_4,u_5,u_6,u_7,u_8],dtype=object))



T_mat = np.array([[0.707,0.707,-0.707,-0.707, 0, 0, 0, 0],
		    [-0.707,0.707,-0.707,0.707, 0, 0, 0, 0 ],
		    [0 , 0, 0, 0, -1, 1, 1, -1],
		    [0.06, -0.06, 0.06, -0.06, -0.218, -0.218,0.218, 0.218],
		    [0.06, 0.06, -0.06, -0.06, 0.120, -0.120, 0.120, -0.120],
		    [-0.1888, 0.1888, 0.1888, -0.1888, 0, 0, 0, 0]],dtype=object)

tau_vec = T_mat@u_vec




x_sp = model.set_variable('_tvp', 'x_sp')
y_sp = model.set_variable('_tvp', 'y_sp')
z_sp = model.set_variable('_tvp', 'z_sp')
phi_sp = model.set_variable('_tvp', 'phi_sp')
theta_sp = model.set_variable('_tvp', 'theta_sp')
psi_sp = model.set_variable('_tvp', 'psi_sp')


model.set_rhs('x', cos(psi)*cos(theta)*u + (-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi))*v + (sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*w)
model.set_rhs('y', sin(psi)*cos(theta)*u + (cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi))*v + (-cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi))*w)
model.set_rhs('z', -sin(theta)*u + cos(theta)*sin(phi)*v + cos(theta)*cos(phi)*w)

model.set_rhs('phi', p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r)
model.set_rhs('theta',  cos(phi)*q - sin(phi)*r)
model.set_rhs('psi', (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r)

model.set_rhs('p', p_dot)
model.set_rhs('q', q_dot)
model.set_rhs('r', r_dot)

model.set_rhs('u', u_dot)
model.set_rhs('v', v_dot)
model.set_rhs('w', w_dot)




#Building the equations of motion for the ROV

M_rb = np.array([[m, 0, 0, 0, m*z_g, 0],
		 [0, m, 0, -m*z_g, 0 ,0],
		 [0, 0, m, 0, 0, 0],
		 [0, -m*z_g, 0, I_x, 0 ,0],
		 [m*z_g, 0, 0, 0, I_y, 0],
		 [0, 0, 0, 0, 0, I_z]],dtype=object)
print(np.size(M_rb))
print(v.shape)
M_a = -np.diag([X_udot,Y_vdot,Z_wdot,K_pdot,M_qdot,N_rdot])

M = M_rb+M_a

C_a = np.array([[0, 0, 0, 0, Z_wdot*w, 0],
		 [0, 0, 0, -Z_wdot*w, 0 ,-X_udot*u],
		 [0, 0, 0, -Y_vdot*v, X_udot*u ,0],
		 [0, -Z_wdot*w, Y_vdot*v,0, -N_rdot*r, M_qdot*q],
		 [Z_wdot*w, 0, -X_udot*u, N_rdot*r, 0 ,-K_pdot*p],
		 [-Y_vdot*v, X_udot*u, 0, -M_qdot*q, K_pdot*p,0]], dtype=object)
print(M_rb)



C_rb = np.array([[0, 0, 0, 0, m*w, 0],
		 [0, 0, 0, -m*w, 0 ,0],
		 [0, 0, 0, m*v, -m*u ,0],
		 [0, 0, m*w, -m*v, I_z*r ,-I_y*q],
		 [-m*w, 0, -m*v, -I_z*r, 0, -I_x*p],
		 [m*v, -m*u, 0, I_y*q, -I_x*p, 0]], dtype=object)


print(model.u)
C = C_a + C_rb


#D matrix written as a column vector, to avoid problesm with nmpy and casadi, possible since the D vector is a diagonal matrix
D = -np.transpose(np.array([X_u+X_u_abs*u, Y_v+Y_v_abs*v, Z_w+Z_w_abs*v, K_p+K_p_abs*p, M_q+M_q_abs*q, N_r+N_r_abs*r],dtype=object))








v_dot_vec = np.transpose(np.array([u_dot,v_dot,w_dot,p_dot,q_dot,r_dot]))

v_vec = np.transpose(np.array([u,v,w,p,q,r],dtype=object))



#hydrostatics
g_1 = (W-B)*sin(theta)
g_2 = -(W-B)*cos(theta)*sin(phi)
g_3 = -(W-B)*cos(theta)*cos(phi)
g_4 = -z_g*W*cos(theta)*sin(phi)
g_5 = z_g*W*sin(theta)
g_6 = 0

#tau forces
tau_1 = (0.707*u_1 + 0.707*u_2-0.707*u_3-0.707*u_4)
tau_2 =(-0.707*u_1 + 0.707*u_2-0.707*u_3 + 0.707*u_4)
tau_3 =( -u_5 + u_6 + u_7 - u_8)
tau_4 =(0.06*u_1 - 0.06*u_2 + 0.06*u_3 - 0.06*u_4 -0.218*u_5 - 0.218*u_6 + 0.218*u_7 + 0.218*u_8)
tau_5 =(0.06*u_1 + 0.06*u_2 - 0.06*u_3 - 0.06*u_4 + 0.120*u_5 - 0.120*u_6 + 0.120*u_7 - 0.120*u_8)
tau_6 =(-0.1888*u_1 + 0.1888*u_2 + 0.1888*u_3 - 0.1888*u_4)

#M_rb
M_rb_1 = m*(u_dot + z_g*q_dot)
M_rb_2 = m*(v_dot - z_g*p_dot)
M_rb_3 = m*w_dot
M_rb_4 = -m*z_g*v_dot + I_x*p_dot
M_rb_5 = m*z_g*u_dot + I_y*q_dot
M_rb_6 = I_z*r_dot

#C_rb
C_rb_1 = m*w*q
C_rb_2 =  -m*w*p
C_rb_3 =  m*(v*p - u*q)
C_rb_4 =  m*(w*v - v*w) +I_z*r*q - I_y*q*r
C_rb_5 = -m*(w*u - u*w) - I_z*r*p + I_x*p*r
C_rb_6 = m*(v*u - u*v) + I_y*q*p -I_x*p*q

# M_a
M_a_1 = -X_udot*u_dot
M_a_2 = -Y_vdot*v_dot
M_a_3 = -Z_wdot*w_dot
M_a_4 = -K_pdot*p_dot
M_a_5 = -M_qdot*q_dot
M_a_6 = -N_rdot*r_dot

#C_a
C_a_1 = Z_wdot*w*q
C_a_2 = -Z_wdot*w*p - X_udot*u*r
C_a_3 = -Y_vdot*v*p + X_udot*u*q
C_a_4 = -Z_wdot*w*v + Y_vdot*v*w - N_rdot*r*q + M_qdot*q*r
C_a_5 = Z_wdot*w*u - X_udot*u*w + N_rdot*r*p - K_pdot*p*r
C_a_6 = - Y_vdot*v*u + X_udot*u*v - M_qdot*q*p + K_pdot*p*q


#D_l
D_l_1 = -X_u*u 
D_l_2 = -Y_v*v
D_l_3 = -Z_w*w
D_l_4 = -K_p*p
D_l_5 = -M_q*q
D_l_6 = -N_r*r

#D_nl
D_nl_1 = -(X_u_abs*fabs(u))*u
D_nl_2 = -(Y_v_abs*fabs(v))*v 
D_nl_3 = -(Z_w_abs*fabs(w))*w
D_nl_4 = -(K_p_abs*fabs(p))*p
D_nl_5 = -(M_q_abs*fabs(q))*q
D_nl_6 = -(N_r_abs*fabs(r))*r


#f_1 = M_rb_1 + M_a_1 + C_rb_1 + C_a_1 + D_l_1  + g_1 - tau_1
#f_2 = M_rb_2 + M_a_2 + C_rb_2 + C_a_2 + D_l_2  + g_2 - tau_2
#f_3 = M_rb_3 + M_a_3 + C_rb_3 + C_a_3 + D_l_3  + g_3 - tau_3
#f_4 = M_rb_4 + M_a_4 + C_rb_4 + C_a_4 + D_l_4  + g_4 - tau_4
#f_5 = M_rb_5 + M_a_5 + C_rb_5 + C_a_5 + D_l_5  + g_5 - tau_5
#f_6 = M_rb_6 + M_a_6 + C_rb_6 + C_a_6 + D_l_6  + g_6 - tau_6

f_1 = M_rb_1 + M_a_1 + C_rb_1 + C_a_1 + D_l_1 + D_nl_1 + g_1 - tau_1
f_2 = M_rb_2 + M_a_2 + C_rb_2 + C_a_2 + D_l_2 + D_nl_2 + g_2 - tau_2
f_3 = M_rb_3 + M_a_3 + C_rb_3 + C_a_3 + D_l_3 + D_nl_3 + g_3 - tau_3
f_4 = M_rb_4 + M_a_4 + C_rb_4 + C_a_4 + D_l_4 + D_nl_4 + g_4 - tau_4
f_5 = M_rb_5 + M_a_5 + C_rb_5 + C_a_5 + D_l_5 + D_nl_5 + g_5 - tau_5
f_6 = M_rb_6 + M_a_6 + C_rb_6 + C_a_6 + D_l_6 + D_nl_6 + g_6 - tau_6




#dynamics = vertcat(
#    M[0]@v_dot_vec + C[0]@v_vec + D[0]*v_vec[0] - tau_1,
#	M[1]@v_dot_vec + C[1]@v_vec + D[1]*v_vec[1] - tau_2,
#	M[2]@v_dot_vec + C[2]@v_vec + D[2]*v_vec[2] - tau_3,
#	M[3]@v_dot_vec + C[3]@v_vec + D[3]*v_vec[3] - tau_4,
#	M[4]@v_dot_vec + C[4]@v_vec + D[4]*v_vec[4] - tau_5,
#	M[5]@v_dot_vec + C[5]@v_vec + D[5]*v_vec[5] - tau_6)
#
#DAE written out
#	M				C		D			tau
#f_1 = (m-X_udot)*u_dot + m*z_g*q_dot + (Z_wdot+m)*w*q - (X_u+X_u_abs*fabs(u))*u + g_1 - tau_1
#
##	M				C			   D				tau
#f_2  = (m-Y_vdot)*v_dot -m*z_g*p_dot - (Z_wdot+m)*w*p - X_udot*u - (Y_v + Y_v_abs*fabs(v))*v + g_2 - tau_2
##	M		                C					     D				tau
#f_3 = (m-Z_wdot)*w_dot + (m*v*p - m*u*q -Y_vdot*v*p + X_udot*u*r) - (Z_w+Z_w_abs*fabs(v))*w  + g_3 - tau_3
##	M				                        C
#f_4 = (-m*z_g*v_dot + (I_x-K_pdot)*p_dot) + (-Z_wdot*w+m*w)*v + (-m*v+Y_vdot*v)*w + (I_z*r-N_rdot*r)*q + (-I_y*q+M_qdot*q)*r + (K_p+K_p_abs*fabs(p))*p + g_4 - tau_4
##       M                           
#f_5 = (m*z_g*u_dot - M_qdot*q_dot) + (-m*w*u - m*u*w -I_z*r*p + I_x*p*r) + (Z_wdot*w*u - X_udot*u*w + N_rdot*r*p - K_pdot*p*r) - (M_q+M_q_abs*fabs(q))*q + g_5 - tau_5
##       M
#f_6 = (I_z-N_rdot)*r_dot + (m*v*u - m*u*v + I_y*q*p - I_x*p*q) + (-Y_vdot*v*u + X_udot*u*v - M_qdot*q*p + K_pdot*p*q) -(N_r+N_rdot*fabs(r))*r - tau_6

#f_1 = (u_dot - u-u_vec[1])
#f_2 = (v_dot - v-u_vec[1])
#f_3 = (w_dot - w-u_vec[1])
#f_4 = (p_dot - p-u_vec[1])
#f_5 = (q_dot - q-u_vec[1])
#f_6 = (r_dot - r-u_vec[1])





dynamics = vertcat(f_1,f_2,f_3,f_4,f_5,f_6)

model.set_alg('dynamics',dynamics)

model.setup()



mpc = do_mpc.controller.MPC(model)

setup_mpc = {
        'n_horizon':40,
        't_step':0.1,
        'n_robust':0,
        'store_full_solution':True,

        }

mpc.set_param(**setup_mpc)
_x = model.x
_u = model.u
tvp = model.tvp

# SETPOINT
x_sp = 4
y_sp = -1
z_sp = 2
phi_sp = -3.14
theta_sp = 0
psi_sp = 3.14

#mterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01)
#lterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01
#       + (u_1**2+u_2**2+u_3**2+u_4**2+u_5**2 + u_6**2+u_7**2+u_8**2)*0.001)
#mterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
#lterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
mterm =   (_x['z']-tvp['z_sp'])**2 + (_x['y']-tvp['y_sp'])**2 +  (_x['phi']-tvp['phi_sp'])**2 + (_x['theta']-tvp['theta_sp'])**2 + (_x['psi']-tvp['psi_sp'])**2 + (_x['x']-tvp['x_sp'])**2
lterm =   (_x['z']-tvp['z_sp'])**2 + (_x['y']-tvp['y_sp'])**2 +  (_x['phi']-tvp['phi_sp'])**2 + (_x['theta']-tvp['theta_sp'])**2 + (_x['psi']-tvp['psi_sp'])**2 + (_x['x']-tvp['x_sp'])**2 + (u_1**2+u_2**2+u_3**2+u_4**2+u_5**2 + u_6**2+u_7**2+u_8**2)*0.01

global x_setp
global y_setp
global z_setp
global phi_setp
global theta_setp
global psi_setp


x_setp = 1
y_setp = 1
z_setp = 1
phi_setp = 0
theta_setp = 0
psi_setp = 0

tvp_template = mpc.get_tvp_template()
def tvp_fun(t_now):
    global x_setp
    global y_setp
    global z_setp
    global phi_setp
    global theta_setp
    global psi_setp
    tvp_template = mpc.get_tvp_template()
    for k in range(21):
        tvp_template['_tvp',k,'x_sp'] = x_setp
        tvp_template['_tvp',k,'y_sp'] = y_setp
        tvp_template['_tvp',k,'z_sp'] = z_setp
        tvp_template['_tvp',k,'phi_sp'] = phi_setp
        tvp_template['_tvp',k,'theta_sp'] = theta_setp
        tvp_template['_tvp',k,'psi_sp'] = psi_setp

    return tvp_template

mpc.set_tvp_fun(tvp_fun)

#_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
#_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
mpc.set_rterm(
        u_1 = 0.01,
        u_2 = 0.01,
        u_3 = 0.01,
        u_4 = 0.01,
        u_5 = 0.01,
        u_6 = 0.01,
        u_7 = 0.01,
        u_8 = 0.01
        )



mpc.set_objective(mterm=mterm,lterm=lterm)

mpc.bounds['lower','_u', 'u_1'] = - 10
mpc.bounds['lower','_u', 'u_2'] = - 10
mpc.bounds['lower','_u', 'u_3'] = - 10
mpc.bounds['lower','_u', 'u_4'] = - 10
mpc.bounds['lower','_u', 'u_5'] = - 10
mpc.bounds['lower','_u', 'u_6'] = - 10
mpc.bounds['lower','_u', 'u_7'] = - 10
mpc.bounds['lower','_u', 'u_8'] = - 10



mpc.bounds['upper','_u', 'u_1'] =  10
mpc.bounds['upper','_u', 'u_2'] =  10
mpc.bounds['upper','_u', 'u_3'] =  10
mpc.bounds['upper','_u', 'u_4'] =  10
mpc.bounds['upper','_u', 'u_5'] =  10
mpc.bounds['upper','_u', 'u_6'] =  10
mpc.bounds['upper','_u', 'u_7'] =  10
mpc.bounds['upper','_u', 'u_8'] =  10


mpc.setup()

estimator = do_mpc.estimator.StateFeedback(model)


simulator = do_mpc.simulator.Simulator(model)

tvp_template = simulator.get_tvp_template()

def tvp_fun(t_now):
        tvp_template['x_sp'] = 0
        tvp_template['y_sp'] = 0
        tvp_template['z_sp'] = 0
        tvp_template['phi_sp'] = 0
        tvp_template['theta_sp'] = 0
        tvp_template['psi_sp'] = 0
        return tvp_template


simulator.set_tvp_fun(tvp_fun)


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
#               x,y,z,phi,theta,psi,u,v,w,p,q,r
x0 = np.array([2, 3, 2, 0, 0, 0, 0, 0,0,-0,-0,0]).reshape(-1,1)
mpc.x0 = x0
estimator.x0 = x0
simulator.x0 = x0

mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

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


lagring = [] ######################### DENNE ER FOR PLOT ########################################

u0 = np.zeros((8,1))
for i in range(200):
    print("################################### {} ################################################".format(i))
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

    lagring.append(x0) ######################### DENNE ER FOR PLOT ########################################

    if(i == 50):
        x_setp = -3
        y_setp = 0
        z_setp = 2
    if(i == 100):
        phi_setp = -3.14
        theta_setp = 1
        psi_setp = 0
    if(i == 150):
        x_setp = 0
        y_setp = 3
        z_setp = 3
        phi_setp = 1.8
        theta_setp = -1
        psi_setp = 0

######################### DETTE ER FOR PLOT ########################################
for i in range(len(test3)):
    lagring[i] = [float(test3[i][j]) for j in range(len(lagring[i]))]
data = [list(lagring[i]) for i in range(len(lagring))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data.csv', index=False)
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
        sim_graphics.result_lines['_x', 'psi'])
ax[2].legend(lines,'φθψ',title='euler angles')
sim_graphics.plot_results()

sim_graphics.reset_axes()

plt.show()
