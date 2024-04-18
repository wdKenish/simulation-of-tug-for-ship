import numpy as np
import matplotlib.pyplot as plt
from dynamics import rk4_integration as rk4
initial_state = [0, 0, 60, 0, 0, 0]  # Initial position and velocities
dt = 1  # Time step in seconds
T = 10  # Total simulation time in seconds

'''
ship_params = {# 不是滚装船，是BELNOR，集装箱
    'disp': 55126000, # kg
    'Lpp': 181.6,
    'width': 30.5,
    'breadth':30.5,
    'stem_draft': 11.82,
    'stern_draft': 11.82,
    'draft': 11.82,
    'Draft': 22.7,  #这里是假设的 D = 0.02 * L * B ** 0.5 * (0.66 + 0.22 * L / B)，使用书上的母型估算 D = D0 * (Lpp/Lpp0) 
    'x_0': 0,
    'C_b': 0.821,
    'A_x':1735,
    'A_y':394

    }
'''
# 无量纲的尺度 from ship to ship
ship_params = {# 不是滚装船，是BELNOR，集装箱
    'disp': 0.1236, # kg
    'Lpp': 3.6,
    'width': 0.4265,
    'breadth':0.4265,
    'stem_draft': 0.1430, # 假设
    'stern_draft': 0.1430, # 假设
    'draft': 0.1430,
    'Draft': 0.18,  #这里是假设的 D = 0.02 * L * B ** 0.5 * (0.66 + 0.22 * L / B)，使用书上的母型估算 D = D0 * (Lpp/Lpp0) 
    'x_0': 0,
    'C_b': 0.651,
    'A_x':0.1735,
    'A_y':0.394
    }
# env_forces = {'wind_force': calculate_wind_force, 'wave_force': calculate_wave_force}  # Environmental force functions
env_forces = {'wind_params':eval('10,30'),'wave_params':eval('50,-40,1')}

# Execute the simulation using the RK4 integration method

simulated_states = rk4(initial_state, ship_params, env_forces, dt, T)

# Extract data for plotting
times = np.arange(0, T + dt, dt)
x_positions = simulated_states[:, 0]
y_positions = simulated_states[:, 1]
psi_headings = simulated_states[:, 2]



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

