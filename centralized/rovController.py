import do_mpc
class MyController():


    def __init__(self, rovModel, trackMode,  setPoints = [0,0,0,0,0,0,0,0,0,0,0,0]):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.z_setp = setPoints[2]
        self.phi_setp = setPoints[3]
        self.theta_setp = setPoints[4]
        self.psi_setp = setPoints[5]

        self.mpc = do_mpc.controller.MPC(rovModel.model)
        
        setup_mpc = {
                'n_horizon':20,
                't_step':0.1,
                'n_robust':2,
                'store_full_solution':False,
        
                }
        
        self.mpc.set_param(**setup_mpc)
        _x_rov1 = rovModel.model.x
        _u_rov1  = rovModel.model.u
        _tvp_rov1 = rovModel.model.tvp
        #mterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01)
        #lterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01
        #       + (u_1**2+u_2**2+u_3**2+u_4**2+u_5**2 + u_6**2+u_7**2+u_8**2)*0.001)
        #mterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        #lterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        match trackMode:
            case 0:
                mterm =   _x_rov1['z']**2 + _x_rov1['y']**2 +  _x_rov1['phi']**2 + (_x_rov1['theta'])**2 + _x_rov1['psi']**2 + _x_rov1['x']**2 + _x_rov1['z_2']**2 + _x_rov1['y_2']**2 +  _x_rov1['phi_2']**2 + (_x_rov1['theta_2'])**2 + _x_rov1['psi_2']**2 + _x_rov1['x_2']**2
                lterm =   ((_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['y']-_tvp_rov1['y_sp'])**2 +  (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + 
                            (_x_rov1['theta']- _tvp_rov1['theta_sp'] )**2 + (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + (_x_rov1['x']- _tvp_rov1['x_sp'])**2 + _x_rov1['z_2']**2 + (_x_rov1['y_2']-5)**2 +  _x_rov1['phi_2']**2 + (_x_rov1['theta_2'])**2 + _x_rov1['psi_2']**2 + _x_rov1['x_2']**2 +
                            (_u_rov1['u_1_2']**2+_u_rov1['u_2_2']**2+_u_rov1['u_3_2']**2+_u_rov1['u_4_2']**2+_u_rov1['u_5_2']**2 + _u_rov1['u_6_2']**2+_u_rov1['u_7_2']**2+_u_rov1['u_8_2']**2)*0.01)

            case 1:
                mterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 + (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta'] - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2  
                lterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 +(_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta']  - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2
        

        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        tvp_template = self.mpc.get_tvp_template()
        
        self.mpc.set_tvp_fun(self.tvp_fun)
        self.mpc.set_rterm(
                u_1_1 = 1,
                u_2_1 = 1,
                u_3_1 = 1,
                u_4_1 = 1,
                u_5_1 = 1,
                u_6_1 = 1,
                u_7_1 = 1,
                u_8_1 = 1,
		u_1_2 = 1,
                u_2_2 = 1,
                u_3_2 = 1,
                u_4_2 = 1,
                u_5_2 = 1,
                u_6_2 = 1,
                u_7_2 = 1,
                u_8_2 = 1
                )
        
        
        
        self.mpc.set_objective(mterm=mterm,lterm=lterm)
        
        self.mpc.bounds['lower', '_u', 'u_1_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_2_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_3_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_4_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_5_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_6_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_7_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_8_1'] = - 10

        self.mpc.bounds['lower', '_u', 'u_1_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_2_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_3_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_4_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_5_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_6_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_7_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_8_2'] = - 10
        
        
        self.mpc.bounds['upper', '_u', 'u_1_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_2_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_3_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_4_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_5_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_6_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_7_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_8_1'] =  10
        
        self.mpc.bounds['upper', '_u', 'u_1_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_2_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_3_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_4_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_5_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_6_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_7_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_8_2'] =  10
        
        self.mpc.setup()

    def tvp_fun(self, t_now):
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp',k,'x_sp'] =  self.x_setp
            tvp_template['_tvp',k,'y_sp'] =  self.y_setp
            tvp_template['_tvp',k,'z_sp'] =  self.z_setp
            tvp_template['_tvp',k,'phi_sp'] =  self.phi_setp
            tvp_template['_tvp',k,'theta_sp'] =  self.theta_setp
            tvp_template['_tvp',k,'psi_sp'] =  self.psi_setp
    
        return tvp_template

