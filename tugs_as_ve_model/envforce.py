import math
import numpy as np
from math import sin,cos
from params import params
def safe_sqrt(x):
    return np.sqrt(np.clip(x, a_min=None, a_max=1e+150))

def safe_divide(numerator, denominator):
    return numerator / (denominator if denominator != 0 else 1e-10)



def all_env_force(state, ship, env_params):
    wind = env_params['wind_params']
    wave = env_params['wave_params']
    h = hull_force(state, ship)
    cvf = calculate_wave_force(wave[0], wave[1], wave[2], state, ship)
    cwf = calculate_wind_force(wind[0], wind[1], state, ship)
    X = h[0] + cvf[0] + cwf[0]
    Y = h[1] + cvf[1] + cwf[1]
    N = h[2] + cvf[2] + cwf[2]
    return X, Y, N


def hull_force(state, ship, v1=0, r1=0):
    x, y, psi, u, v, r = state
    # Ensure that speed (v) and rotation rate (r) are within reasonable bounds
    v = max(min(v, 30), -30)  # Clamp the velocity to prevent extreme values
    r = max(min(r, 1), -1)    # Clamp the rotation rate similarly
    p = params(state, ship)
    X_u_ = p['X_u_']
    X_rr = p['X_rr']
    X_vr = p['X_vr']
    X_vv = p['X_vv']
    Y_v = p['Y_v']
    Y_r = p['Y_r']
    Y_v1r = p['Y_v1r']
    Y_v1v = p['Y_v1v']
    Y_r1r = p['Y_r1r']
    N_r = p['N_r']
    N_v = p['N_v']
    N_r1r = p['N_r1r']
    N_vvr = p['N_vvr']
    N_vrr = p['N_vrr']
    X_H = X_u_ + X_vv * v ** 2 + X_vr * v * r + X_rr * r ** 2  # Longitudinal hydrodynamic force along the ship centerline
    Y_H = Y_v * v + Y_r * r + Y_v1v * v1 * v + Y_r1r * r1 * r + Y_v1r * v1 * r
    N_H = N_v * v + N_r * r + N_vvr * v * v * r + N_vrr * v * r * r + N_r1r * r1 * r
    X_H = round(X_H,4)
    Y_H = round(Y_H,4)
    N_H = round(N_H,4)
    print('X_H, Y_H, N_H:', X_H, Y_H, N_H)
    return X_H, Y_H, N_H

def calculate_wind_force(U_T, wind_angle, state, ship):
    x, y, psi, u, v, phi = state
    Psi_normalized = psi % 360
    Psi = abs(wind_angle - Psi_normalized)
    Psi_rad = math.radians(Psi)

    rho = 1025
    A_F = ship['A_x']
    A_L = ship['A_y']
    L_OA = ship['Lpp']
    B = ship['breadth']
    H_BR = ship['Draft']
    H_C = 0.5 * (ship['Draft'] - ship['draft'])
    C = 0 # 中心和重心重合 = 0
    A_OD = A_L # 这里指上层建筑的侧向投影面积，但是对于滚装船来说就是Ay
    u_x = U_T * math.cos(Psi_rad) + u
    u_y = U_T * math.sin(Psi_rad) - v
    U_A = safe_sqrt(u_x**2 + u_y**2)
    q_A = 0.5 * rho * np.square(U_A)

    if u_y != 0:
        Psi_A = math.atan(safe_divide(u_x, u_y))
    else:
        Psi_A = math.pi / 2 if u_x >= 0 else -math.pi / 2

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
    X_wind = round(X_wind,4)
    Y_wind = round(Y_wind,4)
    N_wind = round(N_wind,4)
    print('X_wind,Y_wind,N_wind:',X_wind,Y_wind,N_wind)
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
    phi_normalized = phi % 360
    phi = phi_normalized
    L = ship['Lpp']
    rho = 1025
    C_Xw = 0.05 - 0.2 * (Lambda / L) + 0.75 * (Lambda / L) ** 2 - 0.51 * (Lambda / L) ** 3
    C_Yw = 0.46 + 6.83 * (Lambda / L) - 15.65 * (Lambda / L) ** 2 + 8.44 * (Lambda / L) ** 3
    C_Nw = -0.11 + 0.68 * (Lambda / L) - 0.79 * (Lambda / L) ** 2 + 0.21 * (Lambda / L) ** 3
    X_wave = 0.5 * rho * L * a ** 2 * cos(theta + phi) * C_Xw
    Y_wave = 0.5 * rho * L * a ** 2 * sin(theta + phi) * C_Yw
    N_wave = 0.5 * rho * L ** 2 * a ** 2 * sin(theta + phi) * C_Nw
    X_wave = round(X_wave, 4)
    Y_wave = round(Y_wave, 4)
    N_wave = round(N_wave, 4)
    print('X_wave, Y_wave, N_wave:',X_wave, Y_wave, N_wave)
    return X_wave, Y_wave, N_wave
