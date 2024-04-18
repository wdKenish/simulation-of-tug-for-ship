from params import params
def hull_force(state,ship,v1 = 0, r1 = 0):
    x, y, psi, u, v, r = state
    m,I_zz,m_x,m_y,J_zz,X_u_,X_rr,X_vr,X_vv,Y_v,Y_r,Y_v1r,Y_v1v,Y_r1r,Y_r,N_r,N_v,N_r1r,N_vvr,N_vrr = params(state,ship)
    X_H = X_u_ + X_vv * v ** 2 + X_vr * v * r + X_rr * r ** 2  # 纵向水动力，沿船中方向
    Y_H = Y_v * v + Y_r * r + Y_v1v * v1 * v + Y_r1r * r1 * r + Y_v1r * v1 * r 
    N_H = N_v * v + N_r * r + N_vvr * v * v * r + N_vrr * v * r * r + N_r1r * r1 * r
    return X_H,Y_H,N_H