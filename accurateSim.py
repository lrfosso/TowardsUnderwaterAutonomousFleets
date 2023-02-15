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

z_g = r_g[2]
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


#Building the equations of motion for the ROV

M_rb = np.array([[m, 0, 0, 0, m*z_g, 0],
		 [0, m, 0, -m*z_g, 0 ,0],
		 [0, 0, m, 0, 0, 0],
		 [0, -m*z_g, 0, I_x, 0 ,0],
		 [m*z_g, 0, 0, I_y, 0 ,0],
		 [0, 0, 0, 0, 0, I_z]], dtype=object)
print(np.size(M_rb))
v = np.transpose(np.array([u,v,w,p,q,r]))
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


print(model.x)
C = C_a + C_rb

l = C@v
print(l[2])


#d = np.array([1,1,absv],dtype=object)
D = -np.array([X_u+X_u_abs*(u**2)/u, Y_v+Y_v_abs*(v**2)/v, Z_w+Z_w_abs*(v**2)/v, K_p+K_p_abs*(p**2)/2, M_q+M_q_abs*q**2/q, N_r+N_r_abs*(r**2)/r],dtype=object)













