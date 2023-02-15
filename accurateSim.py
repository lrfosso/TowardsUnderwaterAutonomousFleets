import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *

import do_mpc

model_type = 'continuous'
model = do_mpc.model.Model(model_type)

m = 11.5                    # Kg
W = 112.8                   # Newton
B = 114.8                   # Newton
r_b = np.array([[0],        # m     
                [0],
                [0]])
r_g = np.array([[0],        # m
                [0],
                [0.02]])
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
u_dot = model.set_variable('_x', 'u_dot')
v_dot = model.set_variable('_x', 'v_dot')
w_dot = model.set_variable('_x', 'w_dot')
#ang vel
p = model.set_variable('_x', 'p')
q = model.set_variable('_x', 'q')
r = model.set_variable('_x', 'r')
#ang acc
p_dot = model.set_variable('_x', 'p_dot')
q_dot = model.set_variable('_x', 'q_dot')
r_dot = model.set_variable('_x', 'r_dot')

model.set_rhs('x', u)
model.set_rhs('y', v)
model.set_rhs('z', w)

model.set_rhs('phi', p)
model.set_rhs('theta', q)
model.set_rhs('psi', r)

M_rb = np.array([[m, 0, 0, 0, m*z_g, 0],
		 [0, m, 0, -m*z_g, 0 ,0],
		 [0, 0, m, 0, 0 ,0],
		 [0, -m*z_g, 0, I_x, 0 ,0],
		 [0, 0, 0, I_x, 0 ,0],
		 [0, 0, 0, 0, I_y ,0],
		 [0, 0, 0, 0, 0, I_z]])


M_a = -np.diag(X_udot,Y_vdot,Z_wdot,K_pdot,M_pdot,N_rdot)
M = M_rb+M_a



