import numpy as np
import matplotlib.pyplot as plt

class RigidBody:
    def __init__(self, position, velocity, angle):
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.angle = angle
        self.angular_velocity = 0

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def control_loop(rigid_body, target_position, angle_pid, position_pid, dt, max_accel, max_velocity, data):
    # 角度控制
    current_angle_to_target = get_angle_to_target(rigid_body.position, target_position)
    angle_error = current_angle_to_target - rigid_body.angle
    external_torque = 10 * np.cos(rigid_body.angle * rigid_body.angular_velocity)
    total_angle_error = angle_error + external_torque
    angular_command = angle_pid.update(total_angle_error, dt)
    rigid_body.angular_velocity = angular_command
    rigid_body.angle += rigid_body.angular_velocity * dt
    
    # 位置控制
    direction = np.array([np.cos(rigid_body.angle), np.sin(rigid_body.angle)])
    distance_to_target = np.linalg.norm(target_position - rigid_body.position)
    velocity_command = position_pid.update(distance_to_target, dt)
    force_x = 30 * np.cos(rigid_body.velocity[0])
    force_y = 40 * np.cos(rigid_body.velocity[1])
    acceleration = np.array([force_x, force_y]) - velocity_command * direction
    new_velocity = rigid_body.velocity + np.clip(acceleration, -max_accel, max_accel) * dt
    rigid_body.velocity = np.clip(new_velocity, -max_velocity, max_velocity)  # 限制速度
    rigid_body.position += rigid_body.velocity * dt
    
    # 收集数据
    data['positions'].append(rigid_body.position.copy())
    data['velocities'].append(rigid_body.velocity.copy())
    data['forces'].append(np.array([force_x, force_y]))
    data['torques'].append(external_torque)

    # 检查是否应该停止仿真
    if distance_to_target < 0.1 and np.linalg.norm(rigid_body.velocity) < 0.1:
        return False  # 返回 False 表示停止仿真
    return True  # 返回 True 表示继续仿真

def get_angle_to_target(current_position, target_position):
    vector_to_target = target_position - current_position
    angle_to_target = np.arctan2(vector_to_target[1], vector_to_target[0])
    return angle_to_target

# 初始设置和参数
rigid_body = RigidBody(position=[0, 0], velocity=[0, 0], angle=0)
target_position = np.array([10, 10])
angle_pid = PIDController(kp=0.1, ki=0.01, kd=0.05)
position_pid = PIDController(kp=0.2, ki=0.05, kd=0.1)
max_accel = 5  # 最大加速度
max_velocity = 3  # 最大速度
data = {'positions': [], 'velocities': [], 'forces': [], 'torques': []}

# 模拟控制过程
dt = 0.1  # 时间步长
continue_simulation = True
while continue_simulation:
    continue_simulation = control_loop(rigid_body, target_position, angle_pid, position_pid, dt, max_accel, max_velocity, data)
    if not continue_simulation:
        print("Simulation stopped: Target reached with low velocity.")

# 可视化
time_steps = np.arange(len(data['positions'])) * dt
positions = np.array(data['positions'])
velocities = np.array(data['velocities'])
forces = np.array(data['forces'])
torques = np.array(data['torques'])

plt.figure(figsize=(12, 8))
plt.subplot(221)
plt.plot(time_steps, positions[:, 0], label='X Position')
plt.plot(time_steps, positions[:, 1], label='Y Position')
plt.title('Position Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()

plt.subplot(222)
plt.plot(time_steps, velocities[:, 0], label='X Velocity')
plt.plot(time_steps, velocities[:, 1], label='Y Velocity')
plt.title('Velocity Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.legend()

plt.subplot(223)
plt.plot(time_steps, forces[:, 0], label='Force X')
plt.plot(time_steps, forces[:, 1], label='Force Y')
plt.title('Forces Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Force')
plt.legend()

plt.subplot(224)
plt.plot(time_steps, torques, label='Torque')
plt.title('Torque Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Torque')
plt.legend()

plt.tight_layout()
plt.show()
