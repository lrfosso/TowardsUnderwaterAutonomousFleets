"""
fasit:
    mterm = (1.8*(_x_rov1['x'] + 4)**2 + 3*(_x_rov1['y'] - 7)**2 +  2*(_x_rov1['z'] - _tvp_rov1['z_sp'])**2 
    +2*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1']* _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
    +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
    +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
    +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2))
    lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2+_u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+_u_rov1['u_7']**2+_u_rov1['u_8']**2)*0.03
"""

### New variables

x = _x_rov1['x']
y = _x_rov1['y']
z = _x_rov1['z']
q0 = _x_rov1['q_0']
e_1 = _x_rov1['e_1']
e_2 = _x_rov1['e_2']
e_3 = _x_rov1['e_3']
x_sp = _tvp_rov1['x_sp']
y_sp = _tvp_rov1['y_sp']
z_sp = _tvp_rov1['z_sp']
q0_sp = _tvp_rov1['q_0_sp']
e_1_sp = _tvp_rov1['e_1_sp']
e_2_sp = _tvp_rov1['e_2_sp']
e_3_sp = _tvp_rov1['e_3_sp']


q0ref = 0.707
e1ref = 0.0
e2ref = 0.0
e3ref = 0.707

q0 = 0.707
e1 = 0.0
e2 = 0.0
e3 = 0.707


svaret = ((((q0*q0ref+e1*e1ref+e2*e2ref+e3* e3ref)**2-1)**2 )
    +(-e1ref*q0+q0ref*e1-e3ref*e2+e2ref*e3)**2
    +(-e2ref*q0+e3ref*e1+q0ref*e2-e1ref*e3)**2
    +(-e3ref*q0-e2ref*e1+e1ref*e2+q0ref*e3)**2)

print(svaret)