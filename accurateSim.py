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

u_vec = model.set_variable('_u', 'uvec', shape=(8,1))

T_mat = np.array([[0.707,0.707,-0.707,-0.707, 0, 0, 0, 0],
		    [-0.707,0.707,-0.707,0.707, 0, 0, 0, 0 ],
		    [0 , 0, 0, 0, -1, 1, 1, -1],
		    [0.06, -0.06, 0.06, -0.06, -0.218, -0.218,0.218, 0.218],
		    [0.06, 0.06, -0.06, -0.06, 0.120, -0.120, 0.120, -0.120],
		    [-0.1888, 0.1888, 0.1888, -0.1888, 0, 0, 0, 0]],dtype=object)

tau_vec = T_mat@u_vec







model.set_rhs('x', u)
model.set_rhs('y', v)
model.set_rhs('z', w)

model.set_rhs('phi', p)
model.set_rhs('theta', q)
model.set_rhs('psi', r)

model.set_rhs('p', p_dot)
model.set_rhs('q', q_dot)
model.set_rhs('r', r_dot)

model.set_rhs('u', u_dot)
model.set_rhs('v', v_dot)
model.set_rhs('w', w_dot)




#Building the equations of motion for the ROV

M_rb = SX([[m, 0, 0, 0, m*z_g, 0],
		 [0, m, 0, -m*z_g, 0 ,0],
		 [0, 0, m, 0, 0, 0],
		 [0, -m*z_g, 0, I_x, 0 ,0],
		 [m*z_g, 0, 0, 0, I_y, 0],
		 [0, 0, 0, 0, 0, I_z]])
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


print(model.x)
C = C_a + C_rb


#D matrix written as a column vector, to avoid problesm with nmpy and casadi, possible since the D vector is a diagonal matrix
D = -np.transpose(np.array([X_u+X_u_abs*u, Y_v+Y_v_abs*v, Z_w+Z_w_abs*v, K_p+K_p_abs*p, M_q+M_q_abs*q, N_r+N_r_abs*r],dtype=object))








v_dot_vec = np.transpose(np.array([u_dot,v_dot,w_dot,p_dot,q_dot,r_dot]))

v_vec = np.transpose(np.array([u,v,w,p,q,r],dtype=object))
print(tau_vec[0].size)
#dynamics = vertcat(
#	M[0]@v_dot_vec + C[0]@v_vec + D[0]*v_vec[0] - tau_vec[0],
#	M[1]@v_dot_vec + C[1]@v_vec + D[1]*v_vec[1] - tau_vec[1],
#	M[2]@v_dot_vec + C[2]@v_vec + D[2]*v_vec[2] - tau_vec[2],
#	M[3]@v_dot_vec + C[3]@v_vec + D[3]*v_vec[3] - tau_vec[3],
#	M[4]@v_dot_vec + C[4]@v_vec + D[4]*v_vec[4] - tau_vec[4],
#	M[5]@v_dot_vec + C[5]@v_vec + D[5]*v_vec[5] - tau_vec[5])

#DAE written out
#	M				C		D			tau
f_1 = (m-X_udot)*u_dot + m*z_g*q_dot + (Z_wdot+m)*w*q - (X_u+X_u_abs*fabs(u))*u - (0.707*u_vec[0] + 0.707*u_vec[1]-0.707*u_vec[2]-0.707*u_vec[3])
#	M				C			   D				tau
f_2  = (m-Y_vdot)*v_dot -m*z_g*p_dot - (Z_wdot+m)*w*p - X_udot*u - (Y_v + Y_v_abs*fabs(v))*v - (-0.707*u_vec[0] + 0.707*u_vec[1]-0.707*u_vec[2] + 0.707*u_vec[3])
#	M		   C					     D				tau
f_3 = (m-Z_wdot)*w_dot + (m*v*p - m*u*q -Y_vdot*v*p + X_udot*u*r) - (Z_w+Z_w_abs*fabs(v))*w - ( -u_vec[4] + u_vec[5] + u_vec[6] - u_vec[7])
#	M				   C
f_4 = (-m*z_g*v_dot + (I_x-K_pdot)*p_dot) + (-Z_wdot*w+m*w)*v + (-m*v+Y_vdot*v)*w + (I_z*r-N_rdot*r)*q + (-I_y*q+M_qdot*q)*r + (K_p+K_p_abs*fabs(p))*p - (0.06*u_vec[0] - 0.06*u_vec[1] + 0.06*u_vec[2] - 0.06*u_vec[3] -0.218*u_vec[4] - 0.218*u_vec[5] + 0.218*u_vec[6] + 0.218*u_vec[7])

f_5 =




print("u")
print(u)
print("v")
print(v)



dynamics = vertcat(f_1,f_2,f_3,f_4)

model.set_alg('dynamics',dynamics)

model.setup()




