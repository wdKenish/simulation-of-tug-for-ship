import math
from math import sin,cos
# def env_forces(windparams,waveparams,state,ship):
# 可以做一个总的函数来返回力
def calculate_wind_force(U_T,wind_angle,state,ship):
    # 风速，风的攻角（风舷角）,船舶参数
    # from Cruising Performance of a Large Passager Ship in Heavey Sea
    x, y, psi, u, v, phi = state
    Psi = abs (wind_angle - psi)
    rho = 1025 # density of wahter
    A_F = ship['A_x']
    A_L = ship['A_y']
    L_OA = ship['Lpp']
    B = ship['breadth']
    H_BR = ship['Draft']  # 总的高度，对于滚装船就是型深
    H_C = 0.5 * (ship['Draft'] - ship['draft'])  # 船舶水面以上的体积在横向投影的中心高度，可以假设为型深减去吃水的0.5倍
    C = 0 # 中心和重心重合 = 0
    A_OD = A_L # 这里指上层建筑的侧向投影面积，但是对于滚装船来说就是Ay
    u_x = U_T * math.cos(Psi) + u  # u 是船舶x方向速度 = U * cos(belta)
    u_y = U_T * math.sin(Psi) - v  # 同理
    U_A= (u_x**2 + u_y**2) **0.5
    q_A = 0.5 * rho * U_A ** 2
    Psi_A = math.atan(u_x/u_y)
    C_H = 0.355 * phi + 1.0
    alpha = [0.404, 0.368, 0.902]
    beta1 = [-0.922, 0.507, 1.162]
    beta2 = [0.018, -5.091, 10.367, -3.011, -0.341]
    gamma1 = [0.116, 3.345]
    gamma2 = [0.446, 2.192]
    delta1 = [0.458, 3.245, -2.313]
    delta2 = [-1.901, 12.727,24.407, -40.310, -5.481]
    epsilon1 = [-0.585, -0.906, 3.239]
    epsilon2 = [-0.314, -1.117]
    if Psi <= 90 and Psi >= 0:
        C_LF = beta1[0] + beta1[1] * A_L / L_OA / B + beta1[2] * C / L_OA
        C_YM = gamma1[0] + gamma1[1] * A_F / L_OA / B
        C_XLI = delta1[0] + delta1[1] * A_L / L_OA / H_BR + delta1[2] * A_F / B / H_BR
        C_ALF = epsilon1[0] + epsilon1[1] * A_OD / A_L + epsilon1[2] * B / L_OA
    else:
        C_LF = beta2[0] + beta2[1] * B / L_OA  + beta2[2] * H_C / L_OA + beta2[3] * A_OD / L_OA ** 2 + beta2[4] * A_F / B **2
        C_YM = gamma2[0] + gamma2[1] * A_OD / L_OA **2
        C_XLI = delta2[0] + delta2[1] * A_L / L_OA / H_BR + delta2[2] * A_F / A_L + delta2[3] * B / L_OA + delta2[4] * A_F / B / H_BR
        C_ALF = epsilon2[0] + epsilon2[1] * A_OD / A_L
    C_CF = alpha[0] + alpha[1] * A_F / B /H_BR + alpha[2] * H_BR / L_OA
    C_YLI = math.pi * A_L / L_OA **2 + C_YM
    C_AX = (C_LF * math.cos(Psi_A) 
            + C_XLI * (math.sin(Psi_A)-0.5 * math.sin(Psi_A) * math.cos(Psi_A) **2) * math.sin(Psi_A) * math.cos(Psi_A)
            + C_ALF * math.sin(Psi_A) * math.cos(Psi_A) **3)
    C_AY = (C_CF * math.sin(Psi_A)**2
            + C_YLI * (math.cos(Psi_A) + 0.5 * math.sin(Psi_A) **2 * math.cos(Psi_A))* math.sin(Psi_A) * math.cos(Psi_A))
    C_AN = C_AY * (0.927 * C / L_OA - 0.149 * (Psi_A - 0.5 * math.pi))
    X_wind = C_AX * q_A * A_F
    Y_wind = C_H * C_AY * q_A * A_L
    N_wind = C_H * C_AN * q_A * A_L * L_OA
    return X_wind,Y_wind,N_wind

def calculate_wave_force(Lambda,theta,a,state,ship):
    """
    波浪力的计算, 只考虑规则波浪
    Lambda 是波长
    theta 是波浪的方向
    phi 是船舶艏向
    L 是船长
    rho 是水密度
    a 是平均波浪的幅值
    """
    phi = state[2]
    L = ship['Lpp']
    rho = 1025
    C_Xw = 0.05 - 0.2 * (Lambda / L) + 0.75 * (Lambda / L) ** 2 - 0.51 * (Lambda / L) ** 3
    C_Yw = 0.46 + 6.83 * (Lambda / L) - 15.65 * (Lambda / L) ** 2 + 8.44 * (Lambda / L) ** 3
    C_Nw = -0.11 + 0.68 * (Lambda / L) - 0.79 * (Lambda / L) ** 2 + 0.21 * (Lambda / L) ** 3
    X_wave = 0.5 * rho * L * a ** 2 * cos(theta + phi) * C_Xw
    Y_wave = 0.5 * rho * L * a ** 2 * sin(theta + phi) * C_Yw
    N_wave = 0.5 * rho * L ** 2 * a ** 2 * sin(theta + phi) * C_Nw
    return X_wave, Y_wave, N_wave
