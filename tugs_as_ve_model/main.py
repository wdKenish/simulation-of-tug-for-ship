import numpy as np
import matplotlib.pyplot as plt
from dynamics import rk4_integration as rk4
from PIDcontrol import out_a


ship_params = {    # 不是滚装船，是BELNOR，集装箱
    'disp':          55126000, # kg
    'Lpp':           181.6,
    'width':         30.5,
    'breadth':       30.5,
    'stem_draft':    11.82,
    'stern_draft':   11.82,
    'draft':         11.82,
    'Draft':         22.7,  #这里是假设的 D = 0.02 * L * B ** 0.5 * (0.66 + 0.22 * L / B)，使用书上的母型估算 D = D0 * (Lpp/Lpp0) 
    'x_0':           0,
    'C_b':           0.821,
    'A_x':           1735,
    'A_y':           394

    }
'''
# 无量纲的尺度 from ship to ship
ship_params = {# 不是滚装船，是BELNOR，集装箱
    'disp':             0.1236, # kg
    'Lpp':              3.6,
    'width':            0.4265,
    'breadth':          0.4265,
    'stem_draft':       0.1430, # 假设
    'stern_draft':      0.1430, # 假设
    'draft':            0.1430,
    'Draft':            0.18,  #这里是假设的 D = 0.02 * L * B ** 0.5 * (0.66 + 0.22 * L / B)，使用书上的母型估算 D = D0 * (Lpp/Lpp0) 
    'x_0':              0,
    'C_b':              0.651,
    'A_x':              0.1735,
    'A_y':              0.394
    }
'''
# env_forces = {'wind_force': calculate_wind_force, 'wave_force': calculate_wave_force}  # Environmental force functions
env_forces = {'wind_params':eval('10,30'),'wave_params':eval('50,-40,1')}
a_params ={'a_xline':1,'v_xline':5,'a_turn':0.5,'v_turn':2,'a_yline':0.2,'v_yline':1}# 平行靠泊的速度不是这个
# Execute the simulation using the RK4 integration method
target_position = [500,500]
initial_state = [0, 0, 45, 0, 0, 0]  # Initial position and velocities
# [0, 0, 0, 0, 0, 0] 曲线看上去比较好，但是为什么好小
dt = 0.1  # Time step in seconds


# timesfromPID, positions,velocities,accelerations, all_accel = out_a(target_position,ship_params,a_params['a'])

simulated_states = rk4(initial_state, ship_params, env_forces, dt, target_position, a_params)

# Extract data for plotting
times = np.arange(0, simulated_states[0], dt)
x_positions = simulated_states[1][:, 0]
y_positions = simulated_states[1][:, 1]
psi_headings = simulated_states[1][:, 2]

# Plot the trajectory of the ship
plt.figure(figsize=(10, 5))
plt.plot(x_positions, y_positions, label='Trajectory')
plt.title('Ship Trajectory')
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

'''
results = timesfromPID, positions,velocities,accelerations, all_accel

# 提取位置、速度和加速度数据
timesfromPID = [result[0] for result in results]
positions = [result[1] for result in results]
velocities = [result[2] for result in results]
accelerations = [result[3] for result in results]

# 绘制位置图
plt.figure(figsize=(12, 10))

plt.subplot(3, 1, 1)
plt.plot(times, [pos[0] for pos in positions], label='X Position')
plt.plot(times, [pos[1] for pos in positions], label='Y Position')
plt.title('Position vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()

# 绘制速度图
plt.subplot(3, 1, 2)
plt.plot(times, [vel[0] for vel in velocities], label='X Velocity')
plt.plot(times, [vel[1] for vel in velocities], label='Y Velocity')
plt.title('Velocity vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.legend()

# 绘制加速度图
plt.subplot(3, 1, 3)
plt.plot(times, [acc[0] for acc in accelerations], label='X Acceleration')
plt.plot(times, [acc[1] for acc in accelerations], label='Y Acceleration')
plt.title('Acceleration vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration')
plt.legend()

plt.tight_layout()
plt.show()


'''
