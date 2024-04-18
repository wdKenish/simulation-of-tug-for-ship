import math
def params(state,ship): 
    x, y, psi, u, v, r = state
    rho = 1025
    # L为船长，B为船宽，d为船舶吃水， C_b为船舶的方形系数
    L = ship['Lpp']
    B = ship['width']
    d = ship['draft']
    stern_draft = ship['stern_draft']
    stem_draft = ship['stem_draft']
    m = ship['disp']
    C_b = ship['C_b']
    I_zz = m * (0.26 * L) ** 2
    L_w = m / (C_b * d * rho) / (0.85 * B)  # 0.85是人为添加的参数，因为不知道水线面的宽度
    V_ship = m / rho  # 船舶排水体积
    tao = (stern_draft - stem_draft ) / d  # 白军计算水动力X_vr时所用的无量纲吃水差
    d_m = d # 都是吃水
    
    # 需要计算雷诺数
    V = (u ** 2 + v ** 2) ** 0.5

    m_x = (1 / 100 * (0.398 + 11.97 * C_b * (1 + 3.73 * d / B) 
                    - 2.89 * C_b * L / B * (1 + 1.13 * d / B)
                    + 0.175 * C_b * (L / B) ** 2 
                    * (1 + 0.541 * d / B) 
                    - 1.107 * L / B * d / B)) * m
    m_y = ((0.882 - 0.54 * C_b * (1 - 1.6 * d / B) 
            - 0.156 * L / B * (1 - 0.673 * C_b)
            + 0.826 * d / B * L / B *(1 - 0.678 * d / B) 
            - 0.638 * C_b * d / B * L / B* (1 - 0.669 * d / B))) * m 
    J_zz = ((m / 100 * L * (33 - 76.85 * C_b * (1 - 0.784 * C_b) 
                            + 3.43 * L / B * (1 - 0.63 * C_b)))) ** 2 * m 
    

    S = (1.54 * d_m + 0.45 * B + 0.904 * B * C_b + 0.026 * C_b * B * d_m) * L_w # S是船体湿面积
    miu = 1.05372 * 10 ** (-6) # m^2/s  海水在20摄氏度时，roh = 1024.73 kg/m^3,miu=
    R_e = V * L / miu # 雷诺数
    C_r = 0.01 * (35 * V_ship / L ** 3) ** 0.5  #剩余阻力
    #C_f = 0.083 / (math.log(R_e) - 1.65) ** 2 # 摩擦阻力系数R_e可能为0
    if R_e > 0:
        C_f = 0.083 / (math.log(R_e) - 1.65) ** 2
    else:
        C_f = 0  
    if L <= 150:
        # 粗糙度补贴系数的选取
        delta_C_AR = 0.00035 # 应该插值
    elif L <= 210:
        delta_C_AR = 0.0002
    elif L <= 260:
        delta_C_AR = 0.0001
    elif L <= 300:
        delta_C_AR = 0
    elif L <= 350:
        -0.0001
    else:
        delta_C_AR = -0.00025
    C_t = C_f + C_r + delta_C_AR # 总阻力系数
    X_u_ = - S / L / d_m * C_t * u ** 2 # X_u_是船舶直行阻力
    #X_vr = (1.6757 * C_b - 1.5054 ) * lambda22   # 没找到相关文献
    X_vr = (1.11 * C_b - 1.07 ) * m_y + (1.11 * C_b - 0.07) * m_y * (0.208 * tao)
    X_vv = 0.4 * B / L - 0.006 * L / d_m 
    X_rr = 0.0003 * L / d_m 
    # X_H是船舶的纵向力
    # 2.横向水动力及船艏摇力矩 沿船舷方向
    Y_v = -5 * (d_m / L) ** 2
    Y_r = 1.02 * (d_m / L) ** 2
    N_v = -1.94 * (d_m / L) ** 2
    N_r = -0.65 * (d_m / L) ** 2
    Y_v1v = 0.07 - 6.48 * (1 - C_b) * d_m / B
    Y_r1r = 0.0045 - 0.445 * (1 - C_b) * d_m / B
    Y_v1r = -0.3791 + 1.28 * (1 - C_b) * d_m / B
    N_r1r = -0.0805 + 8.6092 * (C_b * B / L) - 36.9816 * (C_b * B / L) ** 2
    N_vvr = -6.0856 + 137.4736 * (C_b * B / L) - 1029.514 * (C_b * B / L) ** 2 + 2480.6082 * (C_b * B / L) ** 3
    N_vrr = -0.0635 + 0.04414 * (C_b * d_m / B)
    return m,I_zz,m_x,m_y,J_zz,X_u_,X_rr,X_vr,X_vv,Y_v,Y_r,Y_v1r,Y_v1v,Y_r1r,Y_r,N_r,N_v,N_r1r,N_vvr,N_vrr