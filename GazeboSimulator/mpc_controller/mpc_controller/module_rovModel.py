######## File that contains the implementation of the model in do-mpc ########

import numpy as np
from casadi import *

import do_mpc

model_type = 'continuous'
class MyROVModel():
    def __init__(self):
        self.model = do_mpc.model.Model(model_type)

        ########## PARAMETERS #############      

        m = 11.5                    # Kg
        W = m*9.81                   # m*g, Newton
        B = 114.8                   # buoyancy, Newton
        r_b = np.array([[0],        # m centre of buoyancy 
                        [0],
                        [0]])
        r_g = np.array([[0],        # m centre of gravity
                        [0],
                        [0.02]])
        
        z_g = r_g[2]
        y_g = 0
        x_g = 0
        #inertia
        I_x = 0.16                  # Kg m**2
        I_y = 0.16                  # Kg m**2
        I_z = 0.16                  # Kg m**2
        
        # Linear damping
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
        
        
        
        #STATES

        #position
        x = self.model.set_variable('_x', 'x')
        y = self.model.set_variable('_x', 'y')
        z = self.model.set_variable('_x', 'z')

        #attitude quaternions
        q_0 = self.model.set_variable('_x','q_0')
        e_1 = self.model.set_variable('_x','e_1')
        e_2 = self.model.set_variable('_x','e_2')
        e_3 = self.model.set_variable('_x','e_3')

        #linear velocity
        u = self.model.set_variable('_x', 'u')
        v = self.model.set_variable('_x', 'v')
        w = self.model.set_variable('_x', 'w')
        
        #linear accelerations
        u_dot = self.model.set_variable('_z', 'u_dot')
        v_dot = self.model.set_variable('_z', 'v_dot')
        w_dot = self.model.set_variable('_z', 'w_dot')
    
        #angular velocity
        p = self.model.set_variable('_x', 'p')
        q = self.model.set_variable('_x', 'q')
        r = self.model.set_variable('_x', 'r')
        
        #angular acceleration
        p_dot = self.model.set_variable('_z', 'p_dot')
        q_dot = self.model.set_variable('_z', 'q_dot')
        r_dot = self.model.set_variable('_z', 'r_dot')

        #input to thrusters
        u_1 = self.model.set_variable('_u', 'u_1')
        u_2 = self.model.set_variable('_u', 'u_2')
        u_3 = -self.model.set_variable('_u', 'u_3')
        u_4 = -self.model.set_variable('_u', 'u_4')
        u_5 = self.model.set_variable('_u', 'u_5')
        u_6 = self.model.set_variable('_u', 'u_6')
        u_7 = self.model.set_variable('_u', 'u_7')
        u_8 = self.model.set_variable('_u', 'u_8')

        #Time-varying parameters for setpoints
        x_sp = self.model.set_variable('_tvp', 'x_sp')
        y_sp = self.model.set_variable('_tvp', 'y_sp')
        z_sp = self.model.set_variable('_tvp', 'z_sp')
        q_0_sp = self.model.set_variable('_tvp', 'q_0_sp')
        e_1_sp = self.model.set_variable('_tvp', 'e_1_sp')
        e_2_sp = self.model.set_variable('_tvp', 'e_2_sp')        
        e_3_sp = self.model.set_variable('_tvp', 'e_3_sp')
        
        #States of other ROVs implemented as time-varying parameters
        x_2 = self.model.set_variable('_tvp', 'x_2') 
        y_2 = self.model.set_variable('_tvp', 'y_2') 
        z_2 = self.model.set_variable('_tvp', 'z_2')    
        x_3 = self.model.set_variable('_tvp', 'x_3') 
        y_3 = self.model.set_variable('_tvp', 'y_3') 
        z_3 = self.model.set_variable('_tvp', 'z_3')  
        
        #Quaternion rotation matrix translating change in body frame to NED
        self.model.set_rhs('x',(1 - 2*(e_2**2 + e_3**2))*u + 2*(e_1*e_2 - e_3*q_0)*v + 2*(e_1*e_3 + e_2*q_0)*w )
        self.model.set_rhs('y', 2*(e_1*e_2 + e_3*q_0)*u + (1 - 2*(e_1**2 + e_3**2))*v + 2*(e_2*e_3 - e_1*q_0)*w)
        self.model.set_rhs('z', 2*(e_1*e_3 - e_2*q_0)*u + 2*(e_2*e_3 + e_1*q_0)*v + (1 - 2*(e_1**2 + e_2**2))*w)
        
        # Translating change in attitude from body to NED
        self.model.set_rhs('q_0', 0.5*(-e_1*p - e_2*q - e_3*r))
        self.model.set_rhs('e_1', 0.5*(q_0*p - e_3*q + e_2*r))
        self.model.set_rhs('e_2', 0.5*(e_3*p + q_0*q - e_1*r))
        self.model.set_rhs('e_3', 0.5*(-e_2*p + e_1*q + q_0*r))

        # Change in rotational velocity
        self.model.set_rhs('p', p_dot)
        self.model.set_rhs('q', q_dot)
        self.model.set_rhs('r', r_dot)

        # Change in linear velocity
        self.model.set_rhs('u', u_dot)
        self.model.set_rhs('v', v_dot)
        self.model.set_rhs('w', w_dot)
        
        #### MATRICES ############### (multiplied out)
        
        #hydrostatics
        g_1 = (B-W)*(2*e_1*e_3 - 2*e_2*q_0)
        g_2 = (B-W)*(2*e_2*e_3 + 2*e_1*q_0)
        g_3 = -(B-W)*(2*e_1**2 + 2*e_2**2 -1)
        g_4 = W*z_g*(2*e_2*e_3 + 2*e_1*q_0)
        g_5 = -W*z_g*(2*e_1*e_3 - 2*e_2*q_0)
        g_6 = 0
        
        #Tau matrix, inputs
        tau_1 = (0.707*u_1 + 0.707*u_2-0.707*u_3-0.707*u_4) #x
        tau_2 =(-0.707*u_1 + 0.707*u_2-0.707*u_3 + 0.707*u_4) #y
        tau_3 =( -u_5 + u_6 + u_7 - u_8)                        #z
        tau_4 =(0.06*u_1 - 0.06*u_2 + 0.06*u_3 - 0.06*u_4 -0.218*u_5 - 0.218*u_6 + 0.218*u_7 + 0.218*u_8) #p
        tau_5 =(0.06*u_1 + 0.06*u_2 - 0.06*u_3 - 0.06*u_4 + 0.120*u_5 - 0.120*u_6 + 0.120*u_7 - 0.120*u_8) #q
        tau_6 =(-0.1888*u_1 + 0.1888*u_2 + 0.1888*u_3 - 0.1888*u_4) #r

        #M_rb, Rigid-body mass matrix 
        M_rb_1 = m*(u_dot + z_g*q_dot)
        M_rb_2 = m*(v_dot - z_g*p_dot)
        M_rb_3 = m*w_dot
        M_rb_4 = -m*z_g*v_dot + I_x*p_dot
        M_rb_5 = m*z_g*u_dot + I_y*q_dot
        M_rb_6 = I_z*r_dot
        
        #C_rb Rigid-body coriolis and centripital matrix
        C_rb_1 = m*w*q + (m*p*z_g-m*v)*r
        C_rb_2 = -m*w*p + (m*u + m*q*z_g)*r
        C_rb_3 =  m*((v - p*z_g)*p + (-u - q*z_g)*q)
        C_rb_4 =  m*(w*v + (p*z_g - v)*w) +I_z*r*q  + (- I_y*q - m*u*z_g)*r
        C_rb_5 = m*(-w*u + (u + q*z_g)*w) - I_z*r*p + (I_x*p - m*v*z_g)*r
        C_rb_6 = m*((v - p*z_g)*u + (-u -q*z_g)*v) + (I_y*q + m*u*z_g)*p  + (m*v*z_g - I_x*p)*q
        
        
        # M_a Added mass matrix
        M_a_1 = -X_udot*u_dot
        M_a_2 = -Y_vdot*v_dot
        M_a_3 = -Z_wdot*w_dot
        M_a_4 = -K_pdot*p_dot
        M_a_5 = -M_qdot*q_dot
        M_a_6 = -N_rdot*r_dot
        
        #C_a Coriolis and centripital matrix
        C_a_1 = -Z_wdot*w*q + Y_vdot*v*r
        C_a_2 = Z_wdot*w*p - X_udot*u*r
        C_a_3 = -Y_vdot*v*p + X_udot*u*q
        C_a_4 = -Z_wdot*w*v + Y_vdot*v*w - N_rdot*r*q + M_qdot*q*r
        C_a_5 = Z_wdot*w*u - X_udot*u*w + N_rdot*r*p - K_pdot*p*r
        C_a_6 = - Y_vdot*v*u + X_udot*u*v - M_qdot*q*p + K_pdot*p*q
        

        #D_l Linear damping matrix, skin friction
        D_l_1 = -X_u*u 
        D_l_2 = -Y_v*v
        D_l_3 = -Z_w*w
        D_l_4 = -K_p*p
        D_l_5 = -M_q*q
        D_l_6 = -N_r*r
        
        #D_nl non-linear damping matrix
        D_nl_1 = -(X_u_abs*fabs(u))*u
        D_nl_2 = -(Y_v_abs*fabs(v))*v 
        D_nl_3 = -(Z_w_abs*fabs(w))*w
        D_nl_4 = -(K_p_abs*fabs(p))*p
        D_nl_5 = -(M_q_abs*fabs(q))*q
        D_nl_6 = -(N_r_abs*fabs(r))*r
        
        
        ################# EQUATIONS OF MOTION ##################
        f_1 = M_rb_1 + M_a_1 + C_rb_1 + C_a_1 + D_l_1 + D_nl_1 + g_1 - tau_1
        f_2 = M_rb_2 + M_a_2 + C_rb_2 + C_a_2 + D_l_2 + D_nl_2 + g_2 - tau_2
        f_3 = M_rb_3 + M_a_3 + C_rb_3 + C_a_3 + D_l_3 + D_nl_3 + g_3 - tau_3
        f_4 = M_rb_4 + M_a_4 + C_rb_4 + C_a_4 + D_l_4 + D_nl_4 + g_4 - tau_4
        f_5 = M_rb_5 + M_a_5 + C_rb_5 + C_a_5 + D_l_5 + D_nl_5 + g_5 - tau_5
        f_6 = M_rb_6 + M_a_6 + C_rb_6 + C_a_6 + D_l_6 + D_nl_6 + g_6 - tau_6
        
        
        
        dynamics = vertcat(f_1,f_2,f_3,f_4,f_5,f_6)
        
        self.model.set_alg('dynamics',dynamics)
        
        self.model.setup()
