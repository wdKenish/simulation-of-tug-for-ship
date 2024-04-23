import math
import numpy as np

# Safe operations to prevent overflow and zero division issues
def safe_sqrt(x):
    """ Prevents overflow by capping the value at a large number before taking the square root. """
    return np.sqrt(np.clip(x, a_min=None, a_max=1e+150))

def safe_divide(numerator, denominator):
    """ Prevents division by zero by adding a small constant to the denominator. """
    return numerator / (denominator if denominator != 0 else 1e-10)

def params(state, ship):
    x, y, psi, u, v, r = state
    rho = 1025
    L = ship['Lpp']
    B = ship['width']
    d = ship['draft']
    stern_draft = ship['stern_draft']
    stem_draft = ship['stem_draft']
    m = ship['disp']
    C_b = ship['C_b']
    I_zz = m * (0.26 * L) ** 2
    L_w = m / (C_b * d * rho) / (0.85 * B)
    V_ship = m / rho
    tao = (stern_draft - stem_draft) / d
    d_m = d

    # Corrected calculations
    V = safe_sqrt(u**2 + v**2)
    miu = 1.05372e-6
    R_e = safe_divide(V * L, miu)
    C_r = 0.01 * (35 * V_ship / L ** 3) ** 0.5  #剩余阻力
    # Check to ensure Reynolds number is not zero for logarithmic operation
    if R_e > 0:
        C_f = 0.083 / (math.log(R_e) - 1.65) ** 2
    else:
        C_f = 0

    # Handling of C_t based on L with correct assignments
    if L <= 150:
        delta_C_AR = 0.00035
    elif L <= 210:
        delta_C_AR = 0.0002
    elif L <= 260:
        delta_C_AR = 0.0001
    elif L <= 300:
        delta_C_AR = 0
    elif L <= 350:
        delta_C_AR = -0.0001
    else:
        delta_C_AR = -0.00025

    C_t = C_f + C_r + delta_C_AR

    def complex_calculation_for_m_x(L, B, C_b, d, m):
        m_x = (1 / 100 * (0.398 + 11.97 * C_b * (1 + 3.73 * d / B) 
                    - 2.89 * C_b * L / B * (1 + 1.13 * d / B)
                    + 0.175 * C_b * (L / B) ** 2 
                    * (1 + 0.541 * d / B) 
                    - 1.107 * L / B * d / B)) * m
        return m_x
    
    def complex_calculation_for_m_y(L, B, C_b, d, m):
        m_y = ((0.882 - 0.54 * C_b * (1 - 1.6 * d / B) 
            - 0.156 * L / B * (1 - 0.673 * C_b)
            + 0.826 * d / B * L / B *(1 - 0.678 * d / B) 
            - 0.638 * C_b * d / B * L / B* (1 - 0.669 * d / B))) * m 
        return m_y

    def complex_calculation_for_J_zz(L, B, C_b, d, m):
        J_zz = ((m / 100 * L * (33 - 76.85 * C_b * (1 - 0.784 * C_b) 
                            + 3.43 * L / B * (1 - 0.63 * C_b)))) ** 2 * m 
        return J_zz

    def complex_calculation_for_S(L, B, C_b, d, m):
        S = (1.54 * d_m + 0.45 * B + 0.904 * B * C_b + 0.026 * C_b * B * d_m) * L_w # S是船体湿面积
        return S 
    def complex_ship_parameters(L, B, C_b, d, m, V, V_ship):
        m_x = complex_calculation_for_m_x(L, B, C_b, d, m)
        m_y = complex_calculation_for_m_y(L, B, C_b, d, m)
        J_zz = complex_calculation_for_J_zz(L, B, C_b, d, m)
        S = complex_calculation_for_S(L, B, C_b, d, m)
        C_r = 0.01 * (35 * V_ship / L ** 3) ** 0.5
        return m_x, m_y, J_zz, S, C_r
    
    m_x, m_y, J_zz, S, C_r = complex_ship_parameters(L, B, C_b, d, m, V, V_ship)
    
    C_t = C_f + C_r + delta_C_AR # 总阻力系数
    X_u_ = - S / L / d_m * C_t * np.square(u) # X_u_是船舶直行阻力
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

    return {
        'm': m,
        'I_zz': I_zz,
        'm_x': m_x,
        'm_y': m_y,
        'J_zz': J_zz,
        'X_u_': X_u_,
        'X_rr': X_rr,
        'X_vr': X_vr,
        'X_vv': X_vv,
        'Y_v': Y_v,
        'Y_r': Y_r,
        'Y_v1r': Y_v1r,
        'Y_v1v': Y_v1v,
        'Y_r1r': Y_r1r,
        'Y_r': Y_r,  
        'N_r': N_r,
        'N_v': N_v,
        'N_r1r': N_r1r,
        'N_vvr': N_vvr,
        'N_vrr': N_vrr
        }

def normalization(a, b ):  # 进行无量纲化
    return a/b







