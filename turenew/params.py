

import math
def calculate(ship):
    # 计算船舶的参数
    # 主要是船舶的水动力导数

    L = ship['Lpp']
    B = ship['breadth']
    d = ship['draft']
    Cb = ship['C_b']
    m = ship['disp']     # 这里的单位是t
    rho = 1025           # 水的密度
    derivatives = {}

    nondim_M = 0.5 * rho * L**2 * d # 无量纲质量
    nondim_N = 0.5 * rho * L**4 * d # 无量纲惯量
    # m = m * rho  # Displacement * water density
    derivatives['m'] = m
    derivatives['Lpp'] = L
    derivatives['rho'] = rho 
    derivatives['d'] = d 
    derivatives['I_zz'] = m * (0.25 * L) ** 2
    derivatives['m_x_dash'] = 0.05 * m / nondim_M
    derivatives['m_y_dash'] = (0.882 - 0.54 * Cb * (1 - 1.6 * d / B) - 0.156 * (1 - 0.673 * Cb) * L / B +
                               0.826 * ((d * L) / (B ** 2)) * (1 - 0.678 * d / B) - 0.638 * Cb * ((d * L) / (
                                           B ** 2)) * (1 - 0.669 * d / B)) * m / nondim_M
    derivatives['J_z_dash'] = (L / 100 * (33 - 76.85 * Cb * (1 - 0.784 * Cb) + 3.43 * L / B * (1 - 0.63 * Cb))) ** 2 * m / nondim_N
    
    # Surge
    m_y_dash = derivatives['m_y_dash']
    derivatives['X_vv_dash'] = 0.0014 - 0.1975 * d * (1 - Cb) / B * L / d  # Lee et. al. (1998)
    derivatives['X_vvvv_dash'] = -6.68 * (Cb / (L / B)) + 1.1  # Yoshimura and Masumoto (2012)
    derivatives['X_rr_dash'] = (-0.0027 + 0.0076 * Cb * d / B) * L / d  # Lee et al. (1998)
    derivatives['X_vr_dash'] = (m + 0.1176 * m_y_dash * (0.5 + Cb)) * L / d  # Lee et al. (1998)

    # Sway
    derivatives['Y_v_dash'] = -(0.5 * math.pi * 2 * d / L + 1.4 * Cb * B / L)  # Kijima et. al. (1990)
    derivatives['Y_vvv_dash'] = (-0.6469 * (1 - Cb) * d / B + 0.0027) * L / d  # Lee et al. (1998)
    derivatives['Y_r_dash'] = (-0.115 * Cb * B / L + 0.0024) * L / d  # Lee et al. (1998)
    derivatives['Y_rrr_dash'] = (-0.233 * Cb * d / B + 0.0063) * L / d  # Lee et al. (1998)
    derivatives['Y_vrr_dash'] = -(5.95 * d * (1 - Cb) / d)  # Kijima et. al. (1990)
    derivatives['Y_vvr_dash'] = 1.5 * d * Cb / B - 0.65  # Kijima et. al. (1990)

    # Yaw
    derivatives['N_v_dash'] = -2 * d / L  # Yoshimura and Masumoto (2012)
    derivatives['N_vvv_dash'] = (0.0348 - 0.5283 * (1 - Cb) * d / B) * L / d  # Lee et al. (1998)
    derivatives['N_r_dash'] = -0.54 * 2 * d / L + (2 * d / L) ** 2  # Kijima et. al. (1990)
    derivatives['N_rrr_dash'] = (-0.0572 + 0.03 * Cb * d / L) * L / d  # Lee et al. (1998)
    derivatives['N_vrr_dash'] = (0.5 * d * Cb / B) - 0.05  # Kijima et. al. (1990)
    derivatives['N_vvr_dash'] = -(57.5 * (Cb * B / L) ** 2 - 18.4 * (Cb * B / L) + 1.6)  # Kijima et. al. (1990)


    derivatives['R_0_dash'] = 0.025
    ship.update(derivatives)
    return ship

# test
ship = {
    'disp': 55126000.0,  # kg
    'Lpp': 181.6,  # Length between perpendiculars (meters)
    'width': 30.5,  # Width of the ship (meters)
    'breadth': 30.5,  # Breadth of the ship (meters)
    'stem_draft': 11.82,  # Draft at the stem (meters)
    'stern_draft': 11.82,  # Draft at the stern (meters)
    'draft': 11.82,  # General draft (meters)
    'Draft': 22.7,  # Assumed from calculation
    'x_0': 0,  # Initial x position
    'C_b': 0.821,  # Block coefficient
    'A_x': 1735.0,  # Frontal area (square meters)
    'A_y': 394.0  # Lateral area (square meters)
}
print(calculate(ship))
