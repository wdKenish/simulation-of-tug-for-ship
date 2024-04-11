import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import math
# 风速及方向
wind_speed = 10  # m/s
wind_direction = 60  # degrees (with respect to positive x-axis)

# 船舶参数
ship_length = 80  # meters
ship_width = 12  # meters
ship_height = 8  # meters
ship_mass = 5000000  # kg

# 初始位置及速度
initial_position = np.array([0, 0])  # meters
initial_velocity = np.array([0.2, 0.1])  # m/s
initial_heading = 30  # degrees

# 航线拟合点
waypoints = np.array([[0, 0], [40, 60], [120, 200], [250, 300], [400, 400], [600, 480]])
# 对航线进行样条曲线拟合
cs = CubicSpline(waypoints[:, 0], waypoints[:, 1])
# 定义一些辅助函数
def calculate_angle(x, y):
    return np.arctan2(y, x) * 180 / np.pi

def calculate_force(speed, angle):
    rho = 1.225  # 空气密度 kg/m^3
    area_front = ship_width * ship_height
    area_side = ship_length * ship_height
    return 0.5 * rho * (area_front * wind_speed**2 * np.sin(np.deg2rad(angle - wind_direction)) +
                        area_side * wind_speed**2 * np.sin(np.deg2rad(angle - (wind_direction - 90))))

# 模拟运动过程
dt = 0.1  # 时间间隔
total_time = 0
current_position = initial_position
current_velocity = initial_velocity
current_heading = initial_heading
forces = []
accelerations = []

while np.linalg.norm(current_position - waypoints[-1]) > 1:
    # 计算当前船舶位置和航向
    current_heading = calculate_angle(cs(current_position[0]), cs(current_position[1]))
    
    # 计算当前船舶与风的遭遇角
    angle_to_wind = current_heading - wind_direction
    
    # 计算当前时刻的阻力
    force = calculate_force(wind_speed, angle_to_wind)
    forces.append(force)
    
    # 计算加速度
    acceleration = force / ship_mass
    accelerations.append(acceleration)
    
    # 更新速度和位置
    current_velocity += acceleration * dt
    current_position += current_velocity * dt
    total_time += dt

# 输出总时间
print("Total Time:", total_time)
# 绘制轨迹图
plt.figure(figsize=(10, 6))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')
plt.plot(current_position[0], current_position[1], 'bo', label='Current Position')
plt.title('Ship Trajectory')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid(True)
plt.show()

# 绘制力的大小和方向随时间的变化图
forces = np.array(forces)
angles = np.array([calculate_angle(current_velocity[0], current_velocity[1]) for _ in range(len(forces))])

plt.figure(figsize=(10, 6))
plt.plot(np.arange(0, total_time, dt), forces, label='Force')
plt.plot(np.arange(0, total_time, dt), angles, label='Angle')
plt.title('Force and Angle over Time')
plt.xlabel('Time (s)')
plt.ylabel('Force (N) / Angle (degrees)')
plt.legend()
plt.grid(True)
plt.show()

# 绘制航向随时间的变化图
plt.figure(figsize=(10, 6))
plt.plot(np.arange(0, total_time, dt), [current_heading for _ in range(len(forces))])
plt.title('Heading over Time')
plt.xlabel('Time (s)')
plt.ylabel('Heading (degrees)')
plt.grid(True)
plt.show()







