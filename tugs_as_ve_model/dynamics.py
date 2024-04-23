import numpy as np
import math
from params import params
from envforce import all_env_force
from tugforces import out_Tugs           # 计算拖轮的分力
from judgement import turn_position
def safe_divide(numerator, denominator):
    # Prevents division by zero by adding a small constant to the denominator.
    return numerator / (denominator if denominator != 0 else 1e-10)

def ship_dynamics(state, t, ship, env_params, target_position, a_params, dt):
    x, y, psi, u, v, r = state
    p = params(state, ship)

    # Unpack parameters with safety
    m = p.get('m', 0)
    I_zz = p.get('I_zz', 0)
    m_x = p.get('m_x', 0)
    m_y = p.get('m_y', 0)
    J_zz = p.get('J_zz', 0)
    # turn_p = turn_position(target_position,ship )
    turn_p = [300,300]
    Tug = out_Tugs(state, turn_p, target_position, ship, env_params, a_params)
    '''
    if Tug == None: # 问题！
        Tug = [0,0,0]
    '''
    if Tug is None:
        Tug = [0, 0, 0]
    env = all_env_force(state, ship, env_params)
    
    env = all_env_force(state, ship, env_params)
    if env is None:
        env = [0, 0, 0]
    print('tug:',Tug)
    print('env:',env)

    X_total, Y_total, N_total = env[0] + Tug[0], env[1] + Tug[1], env[2] + Tug[2]
    u_dot = safe_divide((m + m_y) * u * r + X_total, m + m_x)
    v_dot = safe_divide(Y_total - (m + m_x) * u * r, m + m_y)
    r_dot = safe_divide(N_total, I_zz + J_zz)

    psi_normalized = psi % 360
    psi_rad = math.radians(psi_normalized)

    x_dot = u * math.cos(psi_rad) - v * math.sin(psi_rad)
    y_dot = u * math.sin(psi_rad) + v * math.cos(psi_rad)
    psi_dot = r

    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])

def rk4_integration(initial_state, ship, env_params, dt, target_position, a_params):
    states = [initial_state]
    t = 0  # initial time
    while True:
        t += dt
        if t == dt:
            current_state = ship_dynamics(states[0], t, ship, env_params, target_position, a_params, dt)
            next_state = current_state 
            states.append(next_state)
            continue
        current_state = states[-1]
        x, y, _, _, _, _ = current_state
        # Calculate the distance to the target position
        distance_to_target = np.sqrt((x - target_position[0])**2 + (y - target_position[1])**2)
        if distance_to_target <= 0.1:
            break

        k1 = ship_dynamics(current_state, t, ship, env_params, target_position, a_params, dt)
        k2 = ship_dynamics(current_state + 0.5 * dt * k1, t + 0.5 * dt, ship, env_params, target_position, a_params, dt)
        k3 = ship_dynamics(current_state + 0.5 * dt * k2, t + 0.5 * dt, ship, env_params, target_position, a_params, dt)
        k4 = ship_dynamics(current_state + dt * k3, t + dt, ship, env_params, target_position, a_params, dt)
        
        next_state = current_state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        states.append(next_state)
        
        if t >= 100:break
    return t, np.array(states)
