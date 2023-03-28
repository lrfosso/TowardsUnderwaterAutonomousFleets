import do_mpc
from numpy import cos
class MyController():


    def __init__(self, rovModel, trackMode,  setPoints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.z_setp = setPoints[2]
        self.q_0_setp = setPoints[3]
        self.e_1_setp = setPoints[4]
        self.e_2_setp = setPoints[5]
        self.e_3_setp = setPoints[6]
        self.x_setp_2 = setPoints[7]
        self.y_setp_2 = setPoints[8]
        self.z_setp_2 = setPoints[9]
        self.q_0_setp_2 = setPoints[10]
        self.e_1_setp_2 = setPoints[11]
        self.e_2_setp_2 = setPoints[12]
        self.e_3_setp_2 = setPoints[13]

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

        radius = 2
        length = 2
        FOV_range_deg = 45
        match trackMode:
            case 0:
                mterm =   _x_rov1['z']**2 + _x_rov1['y']**2 +  _x_rov1['phi']**2 + (_x_rov1['theta'])**2 + _x_rov1['psi']**2 + _x_rov1['x']**2 + _x_rov1['z_2']**2 + _x_rov1['y_2']**2 +  _x_rov1['phi_2']**2 + (_x_rov1['theta_2'])**2 + _x_rov1['psi_2']**2 + _x_rov1['x_2']**2
                lterm =   ((_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['y']-_tvp_rov1['y_sp'])**2 +  (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + 
                            (_x_rov1['theta']- _tvp_rov1['theta_sp'] )**2 + (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + (_x_rov1['x']- _tvp_rov1['x_sp'])**2 + _x_rov1['z_2']**2 + (_x_rov1['y_2']-5)**2 +  _x_rov1['phi_2']**2 + (_x_rov1['theta_2'])**2 + _x_rov1['psi_2']**2 + _x_rov1['x_2']**2 +
                            (_u_rov1['u_1_2']**2+_u_rov1['u_2_2']**2+_u_rov1['u_3_2']**2+_u_rov1['u_4_2']**2+_u_rov1['u_5_2']**2 + _u_rov1['u_6_2']**2+_u_rov1['u_7_2']**2+_u_rov1['u_8_2']**2)*0.01)

            case 1:
                mterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 + (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta'] - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2  
                lterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 +(_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta']  - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2
            case 3:
                mterm_1 = 1*(((1*(_tvp_rov1['x_sp']-_x_rov1['x'])**2+ 1*(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-radius**2)**2 +
                ((((_x_rov1['x_2']-_x_rov1['x'])**2+(_x_rov1['y_2']-_x_rov1['y'])**2)-length**2)**2) +
                2*(_x_rov1['z']-_tvp_rov1['z_sp'])**2) + 100*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2)

                mterm_2 = 1*(((1*(_tvp_rov1['x_sp_2']-_x_rov1['x_2'])**2+ 1*(_tvp_rov1['y_sp_2']-_x_rov1['y_2'])**2)-radius**2)**2 +
                ((((_x_rov1['x']-_x_rov1['x_2'])**2+(_x_rov1['y']-_x_rov1['y_2'])**2)-length**2)**2) +
                20*(_x_rov1['z_2']-_tvp_rov1['z_sp_2'])**2) + 10*((((_x_rov1['q_0_2']*_tvp_rov1['q_0_sp_2']+_x_rov1['e_1_2'] * _tvp_rov1['e_1_sp_2']+_x_rov1['e_2_2']* _tvp_rov1['e_2_sp_2']+_x_rov1['e_3_2']* _tvp_rov1['e_3_sp_2'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp_2']*_x_rov1['q_0_2']+_tvp_rov1['q_0_sp_2']*_x_rov1['e_1_2']-_tvp_rov1['e_3_sp_2']*_x_rov1['e_2_2']+_tvp_rov1['e_2_sp_2']*_x_rov1['e_3_2'])**2
                +(-_tvp_rov1['e_2_sp_2']*_x_rov1['q_0_2']+_tvp_rov1['e_3_sp_2']*_x_rov1['e_1_2']+_tvp_rov1['q_0_sp_2']*_x_rov1['e_2_2']-_tvp_rov1['e_1_sp_2']*_x_rov1['e_3_2'])**2
                +(-_tvp_rov1['e_3_sp_2']*_x_rov1['q_0_2']-_tvp_rov1['e_2_sp_2']*_x_rov1['e_1_2']+_tvp_rov1['e_1_sp_2']*_x_rov1['e_2_2']+_tvp_rov1['q_0_sp_2']*_x_rov1['e_3_2'])**2)


                lterm = mterm_1 + mterm_2 + (_u_rov1['u_1_1']**2+_u_rov1['u_2_1']**2+_u_rov1['u_3_1']**2 +
                                 _u_rov1['u_4_1']**2+_u_rov1['u_5_1']**2 + _u_rov1['u_6_1']**2+
                                 _u_rov1['u_7_1']**2+_u_rov1['u_8_1']**2 +
                                 _u_rov1['u_1_2']**2+_u_rov1['u_2_2']**2+_u_rov1['u_3_2']**2 +
                                 _u_rov1['u_4_2']**2+_u_rov1['u_5_2']**2 + _u_rov1['u_6_2']**2+
                                 _u_rov1['u_7_2']**2+_u_rov1['u_8_2']**2)*1


        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        tvp_template = self.mpc.get_tvp_template()

        self.mpc.set_nl_cons("FOV1", 
            2*(cos((FOV_range_deg/180)*3.14)*length-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_x_rov1['x_2']-_x_rov1['x'])
            +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_x_rov1['y_2']-_x_rov1['y'])
            +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_x_rov1['z_2']-_x_rov1['z'])))
            , 0)

        self.mpc.set_nl_cons("FOV2", 
            2*(cos((FOV_range_deg/180)*3.14)*length-((1-(2*_x_rov1['e_2_2']**2+2*_x_rov1['e_3_2']**2))*(_x_rov1['x']-_x_rov1['x_2'])
            +(2*_x_rov1['e_1_2']*_x_rov1['e_2_2']+2*_x_rov1['e_3_2']*_x_rov1['q_0_2'])*(_x_rov1['y']-_x_rov1['y_2'])
            +(2*_x_rov1['e_1_2']*_x_rov1['e_3_2']-2*_x_rov1['e_2_2']*_x_rov1['q_0_2'])*(_x_rov1['z']-_x_rov1['z_2'])))
            , 0)
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
        
        
        
        self.mpc.set_objective(mterm=(mterm_1+mterm_2),lterm=lterm)
        
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
            tvp_template['_tvp',k,'q_0_sp'] = self.q_0_setp 
            tvp_template['_tvp',k,'e_1_sp'] = self.e_1_setp 
            tvp_template['_tvp',k,'e_2_sp'] = self.e_2_setp 
            tvp_template['_tvp',k,'e_3_sp'] = self.e_3_setp 
            tvp_template['_tvp',k,'x_sp_2'] = self.x_setp_2 
            tvp_template['_tvp',k,'y_sp_2'] = self.y_setp_2 
            tvp_template['_tvp',k,'z_sp_2'] = self.z_setp_2 
            tvp_template['_tvp',k,'q_0_sp_2'] = self.q_0_setp_2 
            tvp_template['_tvp',k,'e_1_sp_2'] = self.e_1_setp_2 
            tvp_template['_tvp',k,'e_2_sp_2'] = self.e_2_setp_2 
            tvp_template['_tvp',k,'e_3_sp_2'] = self.e_3_setp_2 
    
        return tvp_template

