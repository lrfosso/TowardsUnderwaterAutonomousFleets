import do_mpc
class MyController():


    def __init__(self, rovModel1, rovModel2, trackMode,  setPoints = [0,0,0,0,0,0,0,0,0,0,0,0], state_2 = [0,0]):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.z_setp = setPoints[2]
        self.phi_setp = setPoints[3]
        self.theta_setp = setPoints[4]
        self.psi_setp = setPoints[5]

        self.x_2 = state_2[0]
        self.y_2 = state_2[1]

        self.mpc = do_mpc.controller.MPC(rovModel1.model)
        
        setup_mpc = {
                'n_horizon':20,
                't_step':0.1,
                'n_robust':2,
                'store_full_solution':True,
        
                }
        
        self.mpc.set_param(**setup_mpc)
        _x_rov1 = rovModel1.model.x
        _u_rov1  = rovModel1.model.u
        _tvp_rov1 = rovModel1.model.tvp
        _tvp_rov2 = rovModel2.model.tvp
        #mterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01)
        #lterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01
        #       + (u_1**2+u_2**2+u_3**2+u_4**2+u_5**2 + u_6**2+u_7**2+u_8**2)*0.001)
        #mterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        #lterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        radius = 2
        lengde = 5
        match trackMode:
            case 0:
                mterm =   _x_rov1['z']**2 + _x_rov1['y']**2 +  _x_rov1['phi']**2 + (_x_rov1['theta'])**2 + _x_rov1['psi']**2 + _x_rov1['x']**2
                lterm =   ((_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['y']-_tvp_rov1['y_sp'])**2 +  (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + 
                            (_x_rov1['theta']- _tvp_rov1['theta_sp'] )**2 + (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + (_x_rov1['x']- _tvp_rov1['x_sp'])**2 + 
                            (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2+_u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+_u_rov1['u_7']**2+_u_rov1['u_8']**2)*0.01)

            case 1:
                mterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 + (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta'] - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2  
                lterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 +(_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta']  - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2
            case 2:
                mterm =   (_x_rov1['theta']- _tvp_rov1['theta_sp'] )**2 + (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 +(_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['y']-_tvp_rov1['y_sp'])**2 + (_x_rov1['x']-_tvp_rov1['x_sp'])**2
                lterm = ((((_tvp_rov1['x_sp']-_x_rov1['x'])**2+(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-radius**2)**2 +
                        ((((_tvp_rov1['x_2']-_x_rov1['x'])**2+(_tvp_rov1['y_2']-_x_rov1['y'])**2)-lengde**2)**2)*0.1 +
                        (+ (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + ((_x_rov1['theta']) - _tvp_rov1['theta_sp'])**2 ) +
                        (_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['phi'])**2 +
                        (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2+_u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+_u_rov1['u_7']**2+_u_rov1['u_8']**2)*0.01
                        )

        tvp_template = self.mpc.get_tvp_template()
        
        self.mpc.set_tvp_fun(self.tvp_fun)
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
        
        
        #self.mpc.set_nl_cons("FOV", -((cos(_x_rov1['psi'])*cos(_x_rov1['theta'])*(_tvp_rov1['x_2']-_x_rov1['x'])+sin(_x_rov1['psi'])*cos(_x_rov1['theta'])*(_tvp_rov1['y_2']-_x_rov1['y'])+sin(_x_rov1['theta']*(_tvp_rov1['z_2']-_x_rov1['z']))) / (((_tvp_rov1['x_2']-_x_rov1['x'])**2+(_tvp_rov1['y_2']-_x_rov1['x'])**2+(_tvp_rov1['z_2']-_x_rov1['z'])**2)**0.5)), ub=1, soft_constraint=False)
        self.mpc.set_objective(mterm=mterm,lterm=lterm)
        
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
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp',k,'x_sp'] =  self.x_setp
            tvp_template['_tvp',k,'y_sp'] =  self.y_setp
            tvp_template['_tvp',k,'z_sp'] =  self.z_setp
            tvp_template['_tvp',k,'phi_sp'] =  self.phi_setp
            tvp_template['_tvp',k,'theta_sp'] =  self.theta_setp
            tvp_template['_tvp',k,'psi_sp'] =  self.psi_setp

            tvp_template['_tvp',k,'x_2'] =  self.x_2
            tvp_template['_tvp',k,'y_2'] =  self.y_2
    
        return tvp_template

