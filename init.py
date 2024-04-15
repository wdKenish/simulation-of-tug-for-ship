import numpy as np

def initialize_simulation():
    # Simulation time settings
    total_simulation_time = 3600  # seconds
    dt = 1  # seconds

    # Ship initial state
    ship_state = {
        'position': np.array([0.0, 0.0]),
        'velocity': np.array([0.0, 0.0]),
        'heading': np.deg2rad(0),
        'rudder_angle': 0,
        'propeller_speed': 0
    }

    # Target position
    target_position = np.array([200, 50])

    # Environmental conditions
    env_conditions = {
        'wind_speed': 15,
        'wind_angle': np.deg2rad(45),
        'wave_amplitude': 2,
        'wave_direction': np.deg2rad(30),
        'current_speed': 2,
        'current_angle': np.deg2rad(60)
    }

    return total_simulation_time, dt, ship_state, target_position, env_conditions