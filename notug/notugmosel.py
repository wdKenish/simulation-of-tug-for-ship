import numpy as np
import math


# 船舶的数据集合

PCC = {# 不是滚装船，是BELNOR，集装箱
    'disp': 55126000, # kg
    'Lpp': 181.6,
    'width': 30.5,
    'stem_draft': 11.82,
    'stern_draft': 11.82,
    'draft': 11.82,
    'Draft': 22.7,  #这里是假设的 D = 0.02 * L * B ** 0.5 * (0.66 + 0.22 * L / B)，使用书上的母型估算 D = D0 * (Lpp/Lpp0) 
    'x_0': 0,
    'C_b': 0.821,
    'A_x':1735,
    'A_y':394

    }
# Define the ship parameters
def params(state): 
    x, y, psi, u, v, r = state
    rho = 1025
    # L为船长，B为船宽，d为船舶吃水， C_b为船舶的方形系数
    L = PCC['Lpp']
    B = PCC['width']
    d = PCC['draft']
    stern_draft = PCC['stern_draft']
    stem_draft = PCC['stem_draft']
    m = PCC['disp']
    C_b = PCC['C_b']
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
    '''
    return {
        'mass': m,
        'added_mass_x': m_x,
        'added_mass_y': m_y,
        'inertia_z': I_zz,
        'length': L,
        'beam': B,
        'draft': d,
        'volume': V_ship
    }
    '''

# 计算船体水动力
def hull_force(u,v,r,v1,r1):
    m,I_zz,m_x,m_y,J_zz,X_u_,X_rr,X_vr,X_vv,Y_v,Y_r,Y_v1r,Y_v1v,Y_r1r,Y_r,N_r,N_v,N_r1r,N_vvr,N_vrr = params()
    X_H = X_u_ + X_vv * v ** 2 + X_vr * v * r + X_rr * r ** 2  # 纵向水动力，沿船中方向
    Y_H = Y_v * v + Y_r * r + Y_v1v * v1 * v + Y_r1r * r1 * r + Y_v1r * v1 * r 
    N_H = N_v * v + N_r * r + N_vvr * v * v * r + N_vrr * v * r * r + N_r1r * r1 * r
    return X_H,Y_H,N_H


ship = PCC

def calculate_wind_force(U_T,Psi):
    # 风速，风的攻角（风舷角）
    # from Cruising Performance of a Large Passager Ship in Heavey Sea
    u = ship['u']
    v = ship['v'] # 要给出，这个是循环量
    phi = ship['r']
    rho = 1025 # density of wahter
    A_F = ship['A_x']
    A_L = ship['A_y']
    L_OA = ship['Loa']
    B = ship['breadth']
    H_BR = ship['Draft']  # 总的高度，对于滚装船就是型深
    H_C = 0.5 * (ship['Draft'] - ship['draft'])  # 船舶水面以上的体积在横向投影的中心高度，可以假设为型深减去吃水的0.5倍
    C = 0 # 中心和重心重合 = 0
    A_OD = A_L # 这里指上层建筑的侧向投影面积，但是对于滚装船来说就是Ay
    u_x = U_T * math.cos(Psi) + u  # u 是船舶x方向速度 = U * cos(belta)
    u_y = U_T * math.sin(Psi) - v  # 同理
    U_A= (u_x**2 + u_y**2) **0.5
    q_A = 0.5 * rho * U_A ** 2
    Psi_A = math.arctan(u_x/u_y)
    C_H = 0.355 * phi + 1.0
    alpha = [0.404, 0.368, 0.902]
    beta1 = [-0.922, 0.507, 1.162]
    beta2 = [0.018, -5.091, 10.367, -3.011, -0.341]
    gamma1 = [0.116, 3.345]
    gamma2 = [0.446, 2.192]
    delta1 = [0.458, 3.245, -2.313, 24.407, -5.481]
    delta2 = [-1.901, 12.727]
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


def calculate_wave_force(H, w_L, T, angle_wave,angle_ship, t): # 输入波高、波长、周期、遭遇角、对应时刻t
    w_a = 0.5 * H
    w_k = 2 * math.pi / w_L
    w_omega = math.pi * 2 / T
    alpha_w = angle_ship - angle_wave # 遭遇角要进行计算
    """
    计算船舶上的波浪诱导力和力矩。

    参数:
    - w_a: 波浪振幅
    - w_k: 波数
    - alpha_w: 与波浪的相遇角度,###需要计算###
    - g: 重力加速度
    - w_omega: 相遇频率
    - t: 时间
    - B: 船舶的宽度（船宽）
    - d: 吃水深度
    - L: 船长
    - Cb: 方形系数
    - rho_water: 水的密度
    - Zb: 船舶浮力中心的垂直坐标
    """
    g = 9.8
    rho_water = 1025
    B = ship['dreadth']
    d = ship['draft']
    L = ship['Lpp']
    X_wave = (4 * rho_water * g * w_a / w_k**2 / math.sin(alpha_w) * (1 - math.exp(-w_k * d))
              * math.sin(w_k * L * math.cos(alpha_w) / 2) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.sin(w_omega * t))
    Y_wave = (4 * rho_water * g * w_a / w_k**2 / math.cos(alpha_w) * (1 - math.exp(-w_k * d))
              * math.sin(w_k * L * math.cos(alpha_w) / 2) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.sin(w_omega * t))
    N_wave = (-2 * rho_water * g * w_a / w_k * (1 - math.exp(-w_k * d)) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.cos(w_omega * t))

    return X_wave, Y_wave, N_wave







def ship_dynamics(state, t, params, e): # 环境参数就固定吧
    """
    Calculate the derivatives of the state vector for the ship's motion.
    state: [x, y, psi, u, v, r] - position, heading, and velocities in ship coordinates
    t: time (not used directly in equations, but useful for time-dependent forces)
    params: ship parameters
    env_forces: environmental forces and moments functions
    """
    # Unpack state
    x, y, psi, u, v, r = state
    
    # Ship parameters
    m,I_zz,m_x,m_y,J_zz = params[0],params[1],params[2],params[3],params[4]
    
    # Environmental forces at current state
    # wind = env_forces['wind_force']
    # wave = env_forces['wave_force']
    
    # Calculate wind and wave forces at current state
    # 数据修改！！！！！！
    wind_forces = calculate_wind_force(200, 50, 15, np.pi/4, psi)  # A simplistic static call, needs dynamic update
    wave_forces = calculate_wave_force(2, 200, 10, np.pi/2, psi)  # A simplistic static call, needs dynamic update
    
    # Decompose wind forces
    X_wind = wind_forces[0]
    Y_wind = wind_forces[1]
    N_wind = wind_forces[2]
    
    # Decompose wave forces
    X_wave = wave_forces[0]
    Y_wave = wave_forces[1]  # Simplified, usually different calculations
    N_wave = wave_forces[2]
    
    # Hydrodynamic forces (simplified, placeholders for actual models)
    X_hydro,Y_hydro,N_hydro = hull_force()

    
    # Sum forces
    X_total = X_hydro + X_wind + X_wave
    Y_total = Y_hydro + Y_wind + Y_wave
    N_total = N_hydro + N_wind + N_wave
    
    u_dot = ((m + m_y) * u * r + X_total) / (m + m_x)
    v_dot = (Y_total - (m + m_x)* u * r) / (m + m_y)
    r_dot = N_total / (I_zz + J_zz)
    
    # Kinematic equations
    x_dot = u * math.cos(psi) - v * math.sin(psi)
    y_dot = u * math.sin(psi) + v * math.cos(psi)
    psi_dot = r
    
    return [x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot]

# Placeholder for RK4 integration (to be implemented)
def rk4_integration(initial_state, params, env_forces, dt, T):
    """
    Perform RK4 numerical integration of the ship dynamics.
    initial_state: initial state vector
    params: ship parameters
    env_forces: environmental forces and moments functions
    dt: time step
    T: total simulation time
    """
    time = np.arange(0, T, dt)
    states = [initial_state]
    for t in time:
        current_state = states[-1]
        k1 = np.array(ship_dynamics(current_state, t, params, env_forces))
        k2 = np.array(ship_dynamics(current_state + 0.5 * dt * k1, t + 0.5 * dt, params, env_forces))
        k3 = np.array(ship_dynamics(current_state + 0.5 * dt * k2, t + 0.5 * dt, params, env_forces))
        k4 = np.array(ship_dynamics(current_state + dt * k3, t + dt, params, env_forces))
        
        next_state = current_state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
        states.append(next_state)
    
    return np.array(states)

# Test integration setup (not executing yet)
initial_state = [0, 0, 60, 0, 0, 0]  # Initial position and velocities
params = params(initial_state)
dt = 0.1  # Time step in seconds
T = 100  # Total simulation time in seconds
ship_params = params(initial_state)  # Get ship parameters
# env_forces = {'wind_force': calculate_wind_force, 'wave_force': calculate_wave_force}  # Environmental force functions
env_forces = 0
# The actual call to rk4_integration will happen once everything is confirmed and ready to simulate
# simulated_states = rk4_integration(initial_state, ship_params, env_forces, dt, T)
# simulated_states

# Execute the simulation using the RK4 integration method
simulated_states = rk4_integration(initial_state, ship_params, env_forces, dt, T)

# Extract data for plotting
times = np.arange(0, T + dt, dt)
x_positions = simulated_states[:, 0]
y_positions = simulated_states[:, 1]
psi_headings = simulated_states[:, 2]

# Import plotting library
import matplotlib.pyplot as plt

# Plot the trajectory of the ship
plt.figure(figsize=(10, 5))
plt.plot(x_positions, y_positions, label='Trajectory')
plt.title('Ship Trajectory Under Environmental Forces')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.legend()
plt.grid(True)
plt.show()

# Plot the heading over time
plt.figure(figsize=(10, 5))
plt.plot(times, np.degrees(psi_headings), label='Heading')
plt.title('Ship Heading Over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Heading (degrees)')
plt.legend()
plt.grid(True)
plt.show()

