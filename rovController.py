import do_mpc
class MyController():
    def __init__(self, rovModel1, rovModel2, setPoints = [0,0,0,0,0,0,0,0,0,0,0,0]):

        self.mpc = do_mpc.controller.MPC(rovModel1.model)
        
        setup_mpc = {
                'n_horizon':40,
                't_step':0.1,
                'n_robust':2,
                'store_full_solution':True,
        
                }
        
        self.mpc.set_param(**setup_mpc)
        _x = rovModel1.model.x
        _u = rovModel1.model.u
        #mterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01)
        #lterm = ((_x['x']-5)**2+ (_x['y']-5)**2+(_x['z']-5)**2 + (_x['u']**2  + _x['v']**2+ _x['w']**2)*0.01
        #       + (u_1**2+u_2**2+u_3**2+u_4**2+u_5**2 + u_6**2+u_7**2+u_8**2)*0.001)
        #mterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        #lterm = _x['x']**2 + _x['y']**2 + _x['z']**2 + (_x['phi']**2 + _x['theta']**2 + _x['psi']**2)*0.1
        mterm =   _x['z']**2 + _x['y']**2 +  _x['phi']**2 + (_x['theta'])**2 + _x['psi']**2 + _x['x']**2
        lterm =   (_x['z']-setPoints[2])**2 + (_x['y']-setPoints[1])**2 +  _x['phi']**2 + (_x['theta'] )**2 + _x['psi']**2 + (_x['x']-[setPoints[0]])**2 + (_u['u_1']**2+_u['u_2']**2+_u['u_3']**2+_u['u_4']**2+_u['u_5']**2 + _u['u_6']**2+_u['u_7']**2+_u['u_8']**2)*0.01
        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
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
        
        
        
        self.mpc.set_objective(mterm=mterm,lterm=lterm)
        
        self.mpc.bounds['lower', '_u', 'u_1'] = - 40
        self.mpc.bounds['lower', '_u', 'u_2'] = - 40
        self.mpc.bounds['lower', '_u', 'u_3'] = - 40
        self.mpc.bounds['lower', '_u', 'u_4'] = - 40
        self.mpc.bounds['lower', '_u', 'u_5'] = - 40
        self.mpc.bounds['lower', '_u', 'u_6'] = - 40
        self.mpc.bounds['lower', '_u', 'u_7'] = - 40
        self.mpc.bounds['lower', '_u', 'u_8'] = - 40
        
        
        self.mpc.bounds['upper', '_u', 'u_1'] =  40
        self.mpc.bounds['upper', '_u', 'u_2'] =  40
        self.mpc.bounds['upper', '_u', 'u_3'] =  40
        self.mpc.bounds['upper', '_u', 'u_4'] =  40
        self.mpc.bounds['upper', '_u', 'u_5'] =  40
        self.mpc.bounds['upper', '_u', 'u_6'] =  40
        self.mpc.bounds['upper', '_u', 'u_7'] =  40
        self.mpc.bounds['upper', '_u', 'u_8'] =  40
        
        
        self.mpc.setup()

