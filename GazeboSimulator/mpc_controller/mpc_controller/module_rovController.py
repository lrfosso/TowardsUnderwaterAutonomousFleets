#### File for the controller implementation in do-mpc ####
### This files contains only the controller class, and is imported to another file, that contains the node ###
import do_mpc
import numpy as np

class MyController():
    def __init__(self, rovModel, n_multi_agent = 0, radius_setp = 2, distance_rovs = 3.5, FOV_range_deg = 90, FOV_range_soft_deg = 45, FOV_constraint = False):
        #### Setting up the MPC-controller
        self.mpc = do_mpc.controller.MPC(rovModel.model)
        setup_mpc = { #MPC-controller parameters
                'n_horizon':20,
                't_step':0.05,
                'n_robust':2,
                'nlpsol_opts': {'ipopt.max_iter': 25},
                }
        self.mpc.set_param(**setup_mpc)
        _x_rov1 = rovModel.model.x
        _u_rov1  = rovModel.model.u
        _tvp_rov1 = rovModel.model.tvp
        ### Initialize Time varying parameters
        self.x_setp = 0
        self.y_setp = 0
        self.z_setp = 0
        self.q_0_setp = 0 
        self.e_1_setp = 0
        self.e_2_setp = 0 
        self.e_3_setp = 0
        self.x_2 = 0 
        self.y_2 = 0
        self.z_2 = 0
        self.x_3 = 0 
        self.y_3 = 0
        self.z_3 = 0
#        self.x_4 = 0 
#        self.y_4 = 0
#        self.z_4 = 0
#        self.x_5 = 0 
#        self.y_5 = 0
#        self.z_5 = 0
#        self.x_6 = 0 
#        self.y_6 = 0
#        self.z_6 = 0
        self.mpc.set_tvp_fun(self.tvp_fun)
        tvp_template = self.mpc.get_tvp_template()
        ### Define the cost function
        match n_multi_agent:
            case 1: #Single agent
                mterm = ((3*(_x_rov1['x']-_tvp_rov1['x_sp'])**2
                + (_x_rov1['z']-_tvp_rov1['y_sp'])**2
                +20*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
                + 30*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
                ))
                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
            case 2: #Two agents
                mterm = ((35*(_x_rov1['x']-_tvp_rov1['x_sp'])**2
                + 35*(_x_rov1['y']-_tvp_rov1['y_sp'])**2
                + 60*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
                + 20*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
                ))
                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
            case 3: #Three agents
                mterm = (
                ( 40*(_x_rov1['x']-_tvp_rov1['x_sp'])**2
                + 40*(_x_rov1['y']-_tvp_rov1['y_sp'])**2
                + 70*(_x_rov1['z']-_tvp_rov1['z_sp'])**2
                )
                + 12*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
                ))
                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
#            case 4: #Four agents
#                mterm = (25*(((1*(_tvp_rov1['x_sp']-_x_rov1['x'])**2+ 1*(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-radius_setp**2)**2 +
#                12*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
#                + 50*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
#                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
#                ))
#                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
#                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
#                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
#            case 5: #Five agents
#                mterm = (25*(((1*(_tvp_rov1['x_sp']-_x_rov1['x'])**2+ 1*(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-radius_setp**2)**2 +
#                12*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
#                + 50*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
#                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
#                ))
#                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
#                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
#                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
#            case 6: #Six agents
#                mterm = (25*(((1*(_tvp_rov1['x_sp']-_x_rov1['x'])**2+ 1*(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-radius_setp**2)**2 +
#                12*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
#                + 50*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
#                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
#                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
#                ))
#                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
#                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
#                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1
        ## Pentalty term for the thrusters
        self.mpc.set_rterm(
                u_1 = 0.1,
                u_2 = 0.1,
                u_3 = 0.1,
                u_4 = 0.1,
                u_5 = 0.1,
                u_6 = 0.1,
                u_7 = 0.1,
                u_8 = 0.1
                )
        self.mpc.set_objective(mterm=mterm,lterm=lterm) #Sets the cost function
        #Penalty terms for the distance and FOV
        #                        2   3  
        penalty_term_distance = [70, 500]
        penalty_term_FOV =      [70, 220]
        # Match non-linear constraints to number of ROVs 
        if(n_multi_agent>1):
            # Distance between ROVs, soft constraint
            self.mpc.set_nl_cons("Distance2", 
                (distance_rovs**2-((_tvp_rov1['x_2']-_x_rov1['x'])**2+(_tvp_rov1['y_2']-_x_rov1['y'])**2+(_tvp_rov1['z_2']-_x_rov1['z'])**2)), 
                ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2], maximum_violation=((distance_rovs*0.9)**2))
            if(FOV_constraint):
                #Soft constraint FOV
                self.mpc.set_nl_cons("FOV1", 
                    (np.cos((FOV_range_soft_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_2']-_x_rov1['x'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_2']-_x_rov1['y'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_2']-_x_rov1['z'])))
                    , ub=0, soft_constraint=True, penalty_term_cons=penalty_term_FOV[n_multi_agent-2]
                    )
                #FOV hard constraint
                self.mpc.set_nl_cons("FOV1_hard", 
                    (np.cos((FOV_range_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_2']-_x_rov1['x'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_2']-_x_rov1['y'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_2']-_x_rov1['z'])))
                    , ub=0, soft_constraint=False
                    )
        if(n_multi_agent>2): # more than two ROVs
            #Distance contraint
            self.mpc.set_nl_cons("Distance3", 
                (distance_rovs**2-((_tvp_rov1['x_3']-_x_rov1['x'])**2+(_tvp_rov1['y_3']-_x_rov1['y'])**2+(_tvp_rov1['z_3']-_x_rov1['z'])**2)), 
                ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2])
            if(FOV_constraint):
                #Soft FOV constraint
                self.mpc.set_nl_cons("FOV2",
                    (np.cos((FOV_range_soft_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_3']-_x_rov1['x'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_3']-_x_rov1['y'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_3']-_x_rov1['z'])))
                    , ub=0, soft_constraint=True, penalty_term_cons=penalty_term_FOV[n_multi_agent-2]
                    )
                #Hard FOV constraint
                self.mpc.set_nl_cons("FOV2_hard",
                    (np.cos((FOV_range_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_3']-_x_rov1['x'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_3']-_x_rov1['y'])
                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_3']-_x_rov1['z'])))
                    , ub=0, soft_constraint=False
                    )
#        if(n_multi_agent>3):
#            self.mpc.set_nl_cons("Distance4", 
#                (distance_rovs**2-((_tvp_rov1['x_4']-_x_rov1['x'])**2+(_tvp_rov1['y_4']-_x_rov1['y'])**2+(_tvp_rov1['z_4']-_x_rov1['z'])**2)), 
#                ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2])
#            if(FOV_constraint):
#                self.mpc.set_nl_cons("FOV3",
#                    2*(np.cos((FOV_range_soft_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_4']-_x_rov1['x'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_4']-_x_rov1['y'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_4']-_x_rov1['z'])))
#                    , ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2]
#                    )
#        if(n_multi_agent>4):
#            self.mpc.set_nl_cons("Distance5", 
#                (distance_rovs**2-((_tvp_rov1['x_5']-_x_rov1['x'])**2+(_tvp_rov1['y_5']-_x_rov1['y'])**2+(_tvp_rov1['z_5']-_x_rov1['z'])**2)), 
#                ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2])
#            if(FOV_constraint):
#                self.mpc.set_nl_cons("FOV4",
#                    2*(np.cos((FOV_range_soft_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_5']-_x_rov1['x'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_5']-_x_rov1['y'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_5']-_x_rov1['z'])))
#                    , ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2]
#                    )
#        if(n_multi_agent>5):
#            self.mpc.set_nl_cons("Distance6", 
#                (distance_rovs**2-((_tvp_rov1['x_6']-_x_rov1['x'])**2+(_tvp_rov1['y_6']-_x_rov1['y'])**2+(_tvp_rov1['z_6']-_x_rov1['z'])**2)), 
#                ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2])
#            if(FOV_constraint):
#                self.mpc.set_nl_cons("FOV5",
#                    2*(np.cos((FOV_range_soft_deg/180)*3.14)*distance_rovs-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_6']-_x_rov1['x'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_6']-_x_rov1['y'])
#                    +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_6']-_x_rov1['z'])))
#                    , ub=0, soft_constraint=True, penalty_term_cons=penalty_term_distance[n_multi_agent-2]
#                    )
  
        # Boundary constraints, set to -10 and + 10
        self.mpc.bounds['lower', '_u', 'u_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_3'] = - 10
        self.mpc.bounds['lower', '_u', 'u_4'] = - 10
        self.mpc.bounds['lower', '_u', 'u_5'] = - 10
        self.mpc.bounds['lower', '_u', 'u_6'] = - 10
        self.mpc.bounds['lower', '_u', 'u_7'] = - 10
        self.mpc.bounds['lower', '_u', 'u_8'] = - 10
        
        
        self.mpc.bounds['upper', '_u', 'u_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_3'] =  10
        self.mpc.bounds['upper', '_u', 'u_4'] =  10
        self.mpc.bounds['upper', '_u', 'u_5'] =  10
        self.mpc.bounds['upper', '_u', 'u_6'] =  10
        self.mpc.bounds['upper', '_u', 'u_7'] =  10
        self.mpc.bounds['upper', '_u', 'u_8'] =  10
        self.mpc.setup()

        

    def tvp_fun(self, t_now):
        """Function for setting the time varying parameters"""
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp',k,'x_sp'] =  self.x_setp
            tvp_template['_tvp',k,'y_sp'] =  self.y_setp
            tvp_template['_tvp',k,'z_sp'] =  self.z_setp
            tvp_template['_tvp',k,'q_0_sp'] =  self.q_0_setp
            tvp_template['_tvp',k,'e_1_sp'] =  self.e_1_setp
            tvp_template['_tvp',k,'e_2_sp'] =  self.e_2_setp
            tvp_template['_tvp',k,'e_3_sp'] =  self.e_3_setp
            tvp_template['_tvp',k,'x_2'] =  self.x_2
            tvp_template['_tvp',k,'y_2'] =  self.y_2
            tvp_template['_tvp',k,'z_2'] =  self.z_2
            tvp_template['_tvp',k,'x_3'] =  self.x_3
            tvp_template['_tvp',k,'y_3'] =  self.y_3
            tvp_template['_tvp',k,'z_3'] =  self.z_3
#            tvp_template['_tvp',k,'x_4'] =  self.x_4
#            tvp_template['_tvp',k,'y_4'] =  self.y_4
#            tvp_template['_tvp',k,'z_4'] =  self.z_4
#            tvp_template['_tvp',k,'x_5'] =  self.x_5
#            tvp_template['_tvp',k,'y_5'] =  self.y_5
#            tvp_template['_tvp',k,'z_5'] =  self.z_5
#            tvp_template['_tvp',k,'x_6'] =  self.x_6
#            tvp_template['_tvp',k,'y_6'] =  self.y_6
#            tvp_template['_tvp',k,'z_6'] =  self.z_6


            
        return tvp_template
