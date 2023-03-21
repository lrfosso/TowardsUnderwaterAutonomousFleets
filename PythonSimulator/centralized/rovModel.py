import numpy as np
from casadi import *

import do_mpc

model_type = 'continuous'
class MyROVModel():
    def __init__(self):
        self.model = do_mpc.model.Model(model_type)
        
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
        
        
        
        #states ROV1
        #pos
        x = self.model.set_variable('_x', 'x')
        y = self.model.set_variable('_x', 'y')
        z = self.model.set_variable('_x', 'z')
        #euler angles
        phi = self.model.set_variable('_x','phi')
        theta = self.model.set_variable('_x','theta')
        psi = self.model.set_variable('_x','psi')
        #lin vel
        u = self.model.set_variable('_x', 'u')
        v = self.model.set_variable('_x', 'v')
        w = self.model.set_variable('_x', 'w')
        #lin acc
        u_dot = self.model.set_variable('_z', 'u_dot')
        v_dot = self.model.set_variable('_z', 'v_dot')
        w_dot = self.model.set_variable('_z', 'w_dot')
        #ang vel
        p = self.model.set_variable('_x', 'p')
        q = self.model.set_variable('_x', 'q')
        r = self.model.set_variable('_x', 'r')
        #ang acc
        p_dot = self.model.set_variable('_z', 'p_dot')
        q_dot = self.model.set_variable('_z', 'q_dot')
        r_dot = self.model.set_variable('_z', 'r_dot')
        #input
        u_1_1 = self.model.set_variable('_u', 'u_1_1')
        u_2_1 = self.model.set_variable('_u', 'u_2_1')
        u_3_1 = self.model.set_variable('_u', 'u_3_1')
        u_4_1 = self.model.set_variable('_u', 'u_4_1')
        u_5_1 = self.model.set_variable('_u', 'u_5_1')
        u_6_1 = self.model.set_variable('_u', 'u_6_1')
        u_7_1 = self.model.set_variable('_u', 'u_7_1')
        u_8_1 = self.model.set_variable('_u', 'u_8_1')
        
        
        x_sp = self.model.set_variable('_tvp', 'x_sp')
        y_sp = self.model.set_variable('_tvp', 'y_sp')
        z_sp = self.model.set_variable('_tvp', 'z_sp')
        phi_sp = self.model.set_variable('_tvp', 'phi_sp')
        theta_sp = self.model.set_variable('_tvp', 'theta_sp')
        psi_sp = self.model.set_variable('_tvp', 'psi_sp')        
        
         #states ROV2
        #pos
        x_2 = self.model.set_variable('_x', 'x_2')
        y_2 = self.model.set_variable('_x', 'y_2')
        z_2 = self.model.set_variable('_x', 'z_2')
        #euler angles
        phi_2 = self.model.set_variable('_x','phi_2')
        theta_2 = self.model.set_variable('_x','theta_2')
        psi_2 = self.model.set_variable('_x','psi_2')
        #lin vel
        u_2 = self.model.set_variable('_x', 'u_2')
        v_2 = self.model.set_variable('_x', 'v_2')
        w_2 = self.model.set_variable('_x', 'w_2')
        #lin acc
        u_dot_2 = self.model.set_variable('_z', 'u_dot_2')
        v_dot_2 = self.model.set_variable('_z', 'v_dot_2')
        w_dot_2 = self.model.set_variable('_z', 'w_dot_2')
        #ang vel
        p_2 = self.model.set_variable('_x', 'p_2')
        q_2 = self.model.set_variable('_x', 'q_2')
        r_2 = self.model.set_variable('_x', 'r_2')
        #ang acc
        p_dot_2 = self.model.set_variable('_z', 'p_dot_2')
        q_dot_2 = self.model.set_variable('_z', 'q_dot_2')
        r_dot_2 = self.model.set_variable('_z', 'r_dot_2')
        #input
        u_1_2 = self.model.set_variable('_u', 'u_1_2')
        u_2_2 = self.model.set_variable('_u', 'u_2_2')
        u_3_2 = self.model.set_variable('_u', 'u_3_2')
        u_4_2 = self.model.set_variable('_u', 'u_4_2')
        u_5_2 = self.model.set_variable('_u', 'u_5_2')
        u_6_2 = self.model.set_variable('_u', 'u_6_2')
        u_7_2 = self.model.set_variable('_u', 'u_7_2')
        u_8_2 = self.model.set_variable('_u', 'u_8_2')
        
        
        x_sp_2 = self.model.set_variable('_tvp', 'x_sp_2')
        y_sp_2 = self.model.set_variable('_tvp', 'y_sp_2')
        z_sp_2 = self.model.set_variable('_tvp', 'z_sp_2')
        phi_sp_2 = self.model.set_variable('_tvp', 'phi_sp_2')
        theta_sp_2 = self.model.set_variable('_tvp', 'theta_sp_2')
        psi_sp_2 = self.model.set_variable('_tvp', 'psi_sp_2')        
        
        
#######################DYNAMICS ROV1 #######################################################





        
        self.model.set_rhs('x', cos(psi)*cos(theta)*u + (-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi))*v + (sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*w)
        self.model.set_rhs('y', sin(psi)*cos(theta)*u + (cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi))*v + (-cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi))*w)
        self.model.set_rhs('z', -sin(theta)*u + cos(theta)*sin(phi)*v + cos(theta)*cos(phi)*w)
        
        self.model.set_rhs('phi', p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r)
        self.model.set_rhs('theta',  cos(phi)*q - sin(phi)*r)
        self.model.set_rhs('psi', (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r)

        self.model.set_rhs('p', p_dot)
        self.model.set_rhs('q', q_dot)
        self.model.set_rhs('r', r_dot)

        self.model.set_rhs('u', u_dot)
        self.model.set_rhs('v', v_dot)
        self.model.set_rhs('w', w_dot)
        
        
        
        
        
        
        #hydrostatics
        g_1 = (W-B)*sin(theta)
        g_2 = -(W-B)*cos(theta)*sin(phi)
        g_3 = -(W-B)*cos(theta)*cos(phi)
        g_4 = -z_g*W*cos(theta)*sin(phi)
        g_5 = z_g*W*sin(theta)
        g_6 = 0
        
        #tau forces
        tau_1 = (0.707*u_1_1 + 0.707*u_2_1-0.707*u_3_1-0.707*u_4_1)
        tau_2 =(-0.707*u_1_1 + 0.707*u_2_1-0.707*u_3_1 + 0.707*u_4_1)
        tau_3 =( -u_5_1 + u_6_1 + u_7_1 - u_8_1)
        tau_4 =(0.06*u_1_1 - 0.06*u_2_1 + 0.06*u_3_1 - 0.06*u_4_1 -0.218*u_5_1 - 0.218*u_6_1 + 0.218*u_7_1 + 0.218*u_8_1)
        tau_5 =(0.06*u_1_1 + 0.06*u_2_1 - 0.06*u_3_1 - 0.06*u_4_1 + 0.120*u_5_1 - 0.120*u_6_1 + 0.120*u_7_1 - 0.120*u_8_1)
        tau_6 =(-0.1888*u_1_1 + 0.1888*u_2_1 + 0.1888*u_3_1 - 0.1888*u_4_1)
        
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
        

################### DYNAMICS ROV2 #######################################################################################

        self.model.set_rhs('x_2', cos(psi_2)*cos(theta_2)*u_2 + (-sin(psi_2)*cos(phi_2) + cos(psi_2)*sin(theta_2)*sin(phi_2))*v_2 + (sin(psi_2)*sin(phi_2)+cos(psi_2)*cos(phi_2)*sin(theta_2))*w_2)
        self.model.set_rhs('y_2', sin(psi_2)*cos(theta_2)*u_2 + (cos(psi_2)*cos(phi_2)+sin(phi_2)*sin(theta_2)*sin(psi_2))*v_2 + (-cos(psi_2)*sin(phi_2) + sin(theta_2)*sin(psi_2)*cos(phi_2))*w_2)
        self.model.set_rhs('z_2', -sin(theta_2)*u_2 + cos(theta_2)*sin(phi_2)*v + cos(theta_2)*cos(phi_2)*w_2)
        
        self.model.set_rhs('phi_2', p_2 + sin(phi_2)*tan(theta_2)*q_2 + cos(phi_2)*tan(theta_2)*r_2)
        self.model.set_rhs('theta_2',  cos(phi_2)*q_2 - sin(phi_2)*r_2)
        self.model.set_rhs('psi_2', (sin(phi_2)/cos(theta_2))*q_2 + (cos(phi_2)/cos(theta_2))*r_2)

        self.model.set_rhs('p_2', p_dot_2)
        self.model.set_rhs('q_2', q_dot_2)
        self.model.set_rhs('r_2', r_dot_2)

        self.model.set_rhs('u_2', u_dot_2)
        self.model.set_rhs('v_2', v_dot_2)
        self.model.set_rhs('w_2', w_dot_2)
        
        
        
        
        
        
        #hydrostatics
        g_1_2 = (W-B)*sin(theta_2)
        g_2_2 = -(W-B)*cos(theta_2)*sin(phi_2)
        g_3_2 = -(W-B)*cos(theta_2)*cos(phi_2)
        g_4_2 = -z_g*W*cos(theta_2)*sin(phi_2)
        g_5_2 = z_g*W*sin(theta_2)
        g_6_2 = 0
        
        #tau forces
        tau_1_2 = (0.707*u_1_2 + 0.707*u_2_2-0.707*u_3_2-0.707*u_4_2)
        tau_2_2 =(-0.707*u_1_2 + 0.707*u_2_2-0.707*u_3_2 + 0.707*u_4_2)
        tau_3_2 =( -u_5_2 + u_6_2 + u_7_2 - u_8_2)
        tau_4_2 =(0.06*u_1_2 - 0.06*u_2_2 + 0.06*u_3_2 - 0.06*u_4_2 -0.218*u_5_2 - 0.218*u_6_2 + 0.218*u_7_2 + 0.218*u_8_2)
        tau_5_2 =(0.06*u_1_2 + 0.06*u_2_2 - 0.06*u_3_2 - 0.06*u_4_2 + 0.120*u_5_2 - 0.120*u_6_2 + 0.120*u_7_2 - 0.120*u_8_2)
        tau_6_2 =(-0.1888*u_1_2 + 0.1888*u_2_2 + 0.1888*u_3_2 - 0.1888*u_4_2)
        
        #M_rb
        M_rb_1_2 = m*(u_dot_2 + z_g*q_dot_2)
        M_rb_2_2 = m*(v_dot_2 - z_g*p_dot_2)
        M_rb_3_2 = m*w_dot_2
        M_rb_4_2 = -m*z_g*v_dot_2 + I_x*p_dot_2
        M_rb_5_2 = m*z_g*u_dot_2 + I_y*q_dot_2
        M_rb_6_2 = I_z*r_dot_2
        
        #C_rb
        C_rb_1_2 = m*w_2*q_2
        C_rb_2_2 =  -m*w_2*p_2
        C_rb_3_2 =  m*(v_2*p_2 - u_2*q_2)
        C_rb_4_2 =  m*(w_2*v_2 - v_2*w_2) +I_z*r_2*q_2 - I_y*q_2*r_2
        C_rb_5_2 = -m*(w_2*u_2 - u_2*w_2) - I_z*r_2*p_2 + I_x*p_2*r_2
        C_rb_6_2 = m*(v_2*u_2 - u_2*v_2) + I_y*q_2*p_2 -I_x*p_2*q_2
        
        # M_a
        M_a_1_2 = -X_udot*u_dot_2
        M_a_2_2 = -Y_vdot*v_dot_2
        M_a_3_2 = -Z_wdot*w_dot_2
        M_a_4_2 = -K_pdot*p_dot_2
        M_a_5_2 = -M_qdot*q_dot_2
        M_a_6_2 = -N_rdot*r_dot_2
        
        #C_a
        C_a_1_2 = Z_wdot*w_2*q_2
        C_a_2_2 = -Z_wdot*w_2*p_2 - X_udot*u_2*r_2
        C_a_3_2 = -Y_vdot*v_2*p_2 + X_udot*u_2*q_2
        C_a_4_2 = -Z_wdot*w_2*v_2 + Y_vdot*v_2*w_2 - N_rdot*r_2*q_2 + M_qdot*q_2*r_2
        C_a_5_2 = Z_wdot*w_2*u_2 - X_udot*u_2*w_2 + N_rdot*r_2*p_2 - K_pdot*p_2*r_2
        C_a_6_2 = - Y_vdot*v_2*u_2 + X_udot*u_2*v_2 - M_qdot*q_2*p_2 + K_pdot*p_2*q_2
        
        
        #D_l
        D_l_1_2 = -X_u*u_2 
        D_l_2_2 = -Y_v*v_2
        D_l_3_2 = -Z_w*w_2
        D_l_4_2 = -K_p*p_2
        D_l_5_2 = -M_q*q_2
        D_l_6_2 = -N_r*r_2
        
        #D_nl
        D_nl_1_2 = -(X_u_abs*fabs(u_2))*u_2
        D_nl_2_2 = -(Y_v_abs*fabs(v_2))*v_2 
        D_nl_3_2 = -(Z_w_abs*fabs(w_2))*w_2
        D_nl_4_2 = -(K_p_abs*fabs(p_2))*p_2
        D_nl_5_2 = -(M_q_abs*fabs(q_2))*q_2
        D_nl_6_2 = -(N_r_abs*fabs(r_2))*r_2
        
        
        #f_1 = M_rb_1 + M_a_1 + C_rb_1 + C_a_1 + D_l_1  + g_1 - tau_1
        #f_2 = M_rb_2 + M_a_2 + C_rb_2 + C_a_2 + D_l_2  + g_2 - tau_2
        #f_3 = M_rb_3 + M_a_3 + C_rb_3 + C_a_3 + D_l_3  + g_3 - tau_3
        #f_4 = M_rb_4 + M_a_4 + C_rb_4 + C_a_4 + D_l_4  + g_4 - tau_4
        #f_5 = M_rb_5 + M_a_5 + C_rb_5 + C_a_5 + D_l_5  + g_5 - tau_5
        #f_6 = M_rb_6 + M_a_6 + C_rb_6 + C_a_6 + D_l_6  + g_6 - tau_6

        ### ROBOT EQUATIONS ############
        f_1 = M_rb_1 + M_a_1 + C_rb_1 + C_a_1 + D_l_1 + D_nl_1 + g_1 - tau_1
        f_2 = M_rb_2 + M_a_2 + C_rb_2 + C_a_2 + D_l_2 + D_nl_2 + g_2 - tau_2
        f_3 = M_rb_3 + M_a_3 + C_rb_3 + C_a_3 + D_l_3 + D_nl_3 + g_3 - tau_3
        f_4 = M_rb_4 + M_a_4 + C_rb_4 + C_a_4 + D_l_4 + D_nl_4 + g_4 - tau_4
        f_5 = M_rb_5 + M_a_5 + C_rb_5 + C_a_5 + D_l_5 + D_nl_5 + g_5 - tau_5
        f_6 = M_rb_6 + M_a_6 + C_rb_6 + C_a_6 + D_l_6 + D_nl_6 + g_6 - tau_6

        f_1_2 = M_rb_1_2 + M_a_1_2 + C_rb_1_2 + C_a_1_2 + D_l_1_2 + D_nl_1_2 + g_1_2 - tau_1_2
        f_2_2 = M_rb_2_2 + M_a_2_2 + C_rb_2_2 + C_a_2_2 + D_l_2_2 + D_nl_2_2 + g_2_2 - tau_2_2
        f_3_2 = M_rb_3_2 + M_a_3_2 + C_rb_3_2 + C_a_3_2 + D_l_3_2 + D_nl_3_2 + g_3_2 - tau_3_2
        f_4_2 = M_rb_4_2 + M_a_4_2 + C_rb_4_2 + C_a_4_2 + D_l_4_2 + D_nl_4_2 + g_4_2 - tau_4_2
        f_5_2 = M_rb_5_2 + M_a_5_2 + C_rb_5_2 + C_a_5_2 + D_l_5_2 + D_nl_5_2 + g_5_2 - tau_5_2
        f_6_2 = M_rb_6_2 + M_a_6_2 + C_rb_6_2 + C_a_6_2 + D_l_6_2 + D_nl_6_2 + g_6_2 - tau_6_2


        
        dynamics = vertcat(f_1,f_2,f_3,f_4,f_5,f_6,f_1_2,f_2_2,f_3_2,f_4_2,f_5_2,f_6_2)
        
        self.model.set_alg('dynamics',dynamics)
        
        self.model.setup()

