import math
import numpy as np
from params import params
from hydro import hull_force
from envforce import calculate_wave_force,calculate_wind_force
def ship_dynamics(state, t, ship,env_params): # 环境参数字典中有风和浪的参数
    """
    Calculate the derivatives of the state vector for the ship's motion.
    state: [x, y, psi, u, v, r] - position, heading, and velocities in ship coordinates
    t: time (not used directly in equations, but useful for time-dependent forces)
    params: ship parameters
    env_forces: environmental forces and moments functions
    """
    # Unpack state
    x, y, psi, u, v, r = state
    p = params(state,ship)
    # Ship parameters
    m,I_zz,m_x,m_y,J_zz = p[0],p[1],p[2],p[3],p[4]
    
    # Environmental forces at current state

    wind = env_params['wind_params']
    wave = env_params['wave_params']
    
    # Calculate wind and wave forces at current state
    # 直接填入了，检查是不是使用字典的原因
    wind_forces = calculate_wind_force(wind[0],wind[1],state,ship)  # A simplistic static call, needs dynamic update
    wave_forces = calculate_wave_force(wave[0],wave[1],wave[2],state,ship)  # A simplistic static call, needs dynamic update
    
    # Decompose wind forces
    X_wind = wind_forces[0]
    Y_wind = wind_forces[1]
    N_wind = wind_forces[2]
    
    # Decompose wave forces
    X_wave = wave_forces[0]
    Y_wave = wave_forces[1]  # Simplified, usually different calculations
    N_wave = wave_forces[2]
    
    # Hydrodynamic forces (simplified, placeholders for actual models)
    X_hydro,Y_hydro,N_hydro = hull_force(state,ship)

    
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
def rk4_integration(initial_state, ship, env_forces, dt, T):
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
    #a = 0  # 用来跳过第一次的u1,v1,r1
    for t in time:

        current_state = states[-1]
        k1 = np.array(ship_dynamics(current_state, t, ship, env_forces))
        k2 = np.array(ship_dynamics(current_state + 0.5 * dt * k1, t + 0.5 * dt, ship, env_forces))
        k3 = np.array(ship_dynamics(current_state + 0.5 * dt * k2, t + 0.5 * dt, ship, env_forces))
        k4 = np.array(ship_dynamics(current_state + dt * k3, t + dt, ship, env_forces))
        
        next_state = current_state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
        states.append(next_state)
    print(states)
    return np.array(states)
