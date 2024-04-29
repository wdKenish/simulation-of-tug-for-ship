import numpy as np
from params import calculate
# from envforce import all_env_force
from tool import safe_divide


# 输入船舶参数
ship_params = {
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


p = calculate(ship_params)

m_x = p['m_x_dash'] * (0.5 * p['rho'] * (p['Lpp']**2) * p['d'])
m_y = p['m_y_dash'] * (0.5 * p['rho'] * (p['Lpp']**2) * p['d'])
J_zz = p['J_z_dash'] * (0.5 * p['rho'] * (p['Lpp']**4) * p['d'])
m = p['m']
I_zz = m * (0.25 * p['Lpp'])**2

def dynamics(state):
    # mmg方程

    psi, u, v ,r = state[2],state[3],state[4],state[5]

    # 计算环境总力和总力矩
    # X_env, Y_env, N_env = all_env_force(state, p, env_params) # 值需要重复计算放入函数当中
    # 根据之前的想法可以不计算环境力的作用效果，只需在最后输出拖轮力的的时候加上即可
    X_env, Y_env, N_env = 0 ,0 ,0
    # 通过PID控制拖轮的力
    xt, yt, nt = None

    X = X_env + xt
    Y = Y_env + yt
    N = N_env + nt
    u_dot = safe_divide((m + m_y) * u * r + X, m + m_x)
    v_dot = safe_divide(Y - (m + m_x) * u * r, m + m_y)
    r_dot = safe_divide(N, I_zz + J_zz)

    # 将航向角进行
    psi_normalized = psi % 360
    psi_rad = np.radians(psi_normalized)

    x_dot = u * np.cos(psi_rad) - v * np.sin(psi_rad)
    y_dot = u * np.sin(psi_rad) + v * np.cos(psi_rad)
    psi_dot = r

    '''还需要对返回的角度值进行归一化，以便于可视化
        但是也应该角度不会超pi'''
    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])


def rk4_step(state, dt):
    # 使用龙格库塔法进行迭代求解
    k1 = dynamics(state)
    k2 = dynamics(state + 0.5 * k1 * dt)
    k3 = dynamics(state + 0.5 * k2 * dt)
    k4 = dynamics(state + k3 * dt)
    
    new_state = state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
    return new_state