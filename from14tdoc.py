import numpy as np
import math
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# 定义船舶和拖轮参数
ship_params = {
    'L_pp': 180.0,    # Length between perpendiculars (m)
    'B': 30.0,        # Beam (m)
    'd': 0.1367,      # Draft (m)
    'D_p': 0.095,     # Propeller diameter (m)
    'C_b': 0.611,     # Block coefficient
    'x_g': -0.0422,   # Longitudinal center of gravity (-)
    'A_R': 0.0333,    # Rudder area ratio (A_R / L_pp * d)
    'rho': 1000       # Water density (kg/m^3)
}

# Hull derivatives for PCC
hull_derivatives = {
    'X_vv': -0.00454, 'X_vr': -0.1232, 'X_rr': 0.5917,
    'Y_v': 0.2629, 'Y_r': -0.566, 'Y_vvvv': 1.5504,
    'Y_vrr': 0.7384, 'Y_rrr': -0.033,
    'N_v': 0.0977, 'N_r': -0.457, 'N_vvv': -0.6273, 'N_vvr': 0.0954, 'N_rrr': -0.0533
}

# Function to calculate hull forces
def calculate_hull_forces(V, delta_r, params, derivatives):
    L_pp, d, x_g, A_R, rho = params['L_pp'], params['d'], params['x_g'], params['A_R'], params['rho']
    r = V / L_pp  # Non-dimensional velocity
    beta = np.arcsin(V / (V**2 + r**2)**0.5)
    beta_r = delta_r - beta  # Rudder angle adjustment
    v = V * np.sin(beta)
    u = V * np.cos(beta)
    X = (derivatives['X_vv']*v**2 + derivatives['X_vr']*v*r + derivatives['X_rr']*r**2) * 0.5 * rho * (L_pp * d)**2
    Y = (derivatives['Y_v']*v + derivatives['Y_r']*r + derivatives['Y_vvvv']*v**4 + derivatives['Y_vrr']*v*r**2 + derivatives['Y_rrr']*r**3) * 0.5 * rho * (L_pp * d)**2 * V
    N = (derivatives['N_v']*v + derivatives['N_r']*r + derivatives['N_vvv']*v**3 + derivatives['N_vvr']*v**2*r + derivatives['N_rrr']*r**3) * 0.5 * rho * (L_pp * d)**2 * L_pp * V
    return X, Y, N

# Initialize ship conditions
V = 10  # m/s
delta_r = 0.1  # radians
X_hull, Y_hull, N_hull = calculate_hull_forces(V, delta_r, ship_params, hull_derivatives)

def calculate_wind_force(U_w, angle_w, params):
    rho_air = 1.225  # air density (kg/m^3)
    A_frontal = params['A_frontal']  # Frontal area for wind resistance
    A_lateral = params['A_lateral']  # Lateral area for wind resistance
    Cx = 0.6  # Drag coefficient in the direction of the wind
    Cy = 0.8  # Drag coefficient perpendicular to the wind

    # Wind force components
    X_wind = 0.5 * rho_air * A_frontal * Cx * U_w**2 * np.cos(angle_w)
    Y_wind = 0.5 * rho_air * A_lateral * Cy * U_w**2 * np.sin(angle_w)
    N_wind = Y_wind * params['L_pp'] / 2  # Simplified moment calculation
    return X_wind, Y_wind, N_wind

# Assuming some values for wind speed and angle
U_w = 15  # wind speed in m/s
angle_w = np.deg2rad(45)  # wind angle in radians
wind_forces = calculate_wind_force(U_w, angle_w, {'A_frontal': 450, 'A_lateral': 900, 'L_pp': ship_params['L_pp']})


def calculate_wave_force(params):
    # Simplified wave force model
    amplitude = 2  # wave amplitude (meters)
    wavelength = 200  # wavelength (meters)
    wave_direction = np.deg2rad(30)  # wave direction in radians relative to ship heading
    L_pp = params['L_pp']

    # Simplified calculation
    X_wave = 0.1 * amplitude * wavelength * np.cos(wave_direction)
    Y_wave = 0.1 * amplitude * wavelength * np.sin(wave_direction)
    N_wave = Y_wave * L_pp / 4  # Simplified moment calculation
    return X_wave, Y_wave, N_wave

wave_forces = calculate_wave_force({'L_pp': ship_params['L_pp']})

def calculate_current_effect(U_c, angle_c, params):
    # Current speed and direction
    rho_water = 1025  # sea water density (kg/m^3)
    C_current = 1.0  # Current drag coefficient
    A_submerged = params['A_submerged']  # Submerged lateral area

    # Current force components
    X_current = 0.5 * rho_water * A_submerged * C_current * U_c**2 * np.cos(angle_c)
    Y_current = 0.5 * rho_water * A_submerged * C_current * U_c**2 * np.sin(angle_c)
    N_current = Y_current * params['L_pp'] / 2  # Simplified moment calculation
    return X_current, Y_current, N_current

# Assuming some values for current speed and angle
U_c = 2  # current speed in m/s
angle_c = np.deg2rad(60)  # current angle in radians
current_effects = calculate_current_effect(U_c, angle_c, {'A_submerged': 800, 'L_pp': ship_params['L_pp']})


# Setting up fuzzy logic control for tug operations
distance = ctrl.Antecedent(np.arange(0, 500, 1), 'distance')
velocity = ctrl.Antecedent(np.arange(0, 10,1), ‘velocity’)
tug_force = ctrl.Consequent(np.arange(0, 100, 1), ‘tug_force’)

Membership functions

distance.automf(3)
velocity.automf(3)

tug_force[‘low’] = fuzz.trimf(tug_force.universe, [0, 20, 40])
tug_force[‘medium’] = fuzz.trimf(tug_force.universe, [30, 50, 70])
tug_force[‘high’] = fuzz.trimf(tug_force.universe, [60, 80, 100])

Rules

rule1 = ctrl.Rule(distance[‘poor’] & velocity[‘good’], tug_force[‘high’])
rule2 = ctrl.Rule(distance[‘average’] & velocity[‘average’], tug_force[‘medium’])
rule3 = ctrl.Rule(distance[‘good’] & velocity[‘poor’], tug_force[‘low’])

tug_control_system = ctrl.ControlSystem([rule1, rule2, rule3])
tug_simulator = ctrl.ControlSystemSimulation(tug_control_system)


# Time settings
total_simulation_time = 3600  # Total simulation time in seconds
dt = 1  # Time step in seconds

# Ship initial state
ship_state = {
    'position': np.array([0.0, 0.0]),  # Initial position (x, y)
    'velocity': np.array([0.0, 0.0]),  # Initial velocity (u, v)
    'heading': np.deg2rad(0),  # Initial heading angle in radians
    'rudder_angle': 0,  # Initial rudder angle in radians
    'propeller_speed': 0  # Initial propeller speed
}

# Target position for berthing
target_position = np.array([200, 50])  # Target position (x, y)

# Environmental conditions
wind_speed = 15  # m/s
wind_angle = np.deg2rad(45)  # radians
wave_amplitude = 2  # meters
wave_direction = np.deg2rad(30)  # radians
current_speed = 2  # m/s
current_angle = np.deg2rad(60)  # radians

# Initialize forces
total_force = np.array([0.0, 0.0])
total_moment = 0.0

# Simulate over the time period
for t in range(0, total_simulation_time, dt):
    # Calculate environmental forces
    wind_forces = calculate_wind_force(wind_speed, wind_angle, {'A_frontal': 450, 'A_lateral': 900, 'L_pp': ship_params['L_pp']})
    wave_forces = calculate_wave_force({'L_pp': ship_params['L_pp'], 'amplitude': wave_amplitude, 'wave_direction': wave_direction})
    current_effects = calculate_current_effect(current_speed, current_angle, {'A_submerged': 800, 'L_pp': ship_params['L_pp']})

    # Update forces
    X_env = wind_forces[0] + wave_forces[0] + current_effects[0]
    Y_env = wind_forces[1] + wave_forces[1] + current_effects[1]
    N_env = wind_forces[2] + wave_forces[2] + current_effects[2]

    # Fuzzy control for tugs based on the distance to target
    distance_to_target = np.linalg.norm(target_position - ship_state['position'])
    speed = np.linalg.norm(ship_state['velocity'])
    tug_simulator.input['distance'] = distance_to_target
    tug_simulator.input['velocity'] = speed
    tug_simulator.compute()
    tug_force = tug_simulator.output['tug_force']

    # Assume tug force acts directly against environmental forces for simplicity
    X_tug = tug_force - X_env
    Y_tug = tug_force - Y_env
    N_tug = tug_force - N_env  # Simplification: tug's moment counters environmental moments

    # Total forces and moments
    total_force = np.array([X_tug, Y_tug])
    total_moment = N_tug

    # Update ship state based on forces and moments
    acceleration = total_force / ship_params['mass']
    angular_acceleration = total_moment / ship_params['moment_of_inertia']

    # Euler integration for new state
    ship_state['velocity'] += acceleration * dt
    ship_state['position'] += ship_state['velocity'] * dt
    ship_state['heading'] += angular_acceleration * dt

    # Check if target is reached
    if distance_to_target < 1.0:
        print(f"Berthing completed at time {t} seconds.")
        break

    # Print state every 100 seconds as an example
    if t % 100 == 0:
        print(f"Time {t}: Position = {ship_state['position']}, Velocity = {ship_state['velocity']}")
        

