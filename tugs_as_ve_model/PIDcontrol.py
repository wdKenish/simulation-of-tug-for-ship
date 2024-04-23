# PID算法初步实现
import numpy as np
import matplotlib.pyplot as plt
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error

        return output
    
class TwoDPIDController:
    def __init__(self, kp, ki, kd):
        self.pid_x = PIDController(kp, ki, kd)
        self.pid_y = PIDController(kp, ki, kd)

    def compute(self, setpoint_x, setpoint_y, measured_x, measured_y, dt):
        accel_x = self.pid_x.compute(setpoint_x, measured_x, dt)
        accel_y = self.pid_y.compute(setpoint_y, measured_y, dt)
        return accel_x, accel_y



class TwoDMotionSimulator:
    '''
    实现二维的速度控制，加速控制和位置输出
    '''
    def __init__(self, mass, max_accel, max_velocity, controller, initial_pos=(0, 0)):
        self.mass = mass
        self.max_accel = max_accel
        self.max_velocity = max_velocity
        self.controller = controller
        self.position = np.array(initial_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.time = 0

    def update(self, target_position, dt):
        # Calculate desired acceleration from the PID controllers
        accel_x, accel_y = self.controller.compute(target_position[0], target_position[1], 
                                                   self.position[0], self.position[1], dt)
        
        # Limit acceleration
        accel = np.array([accel_x, accel_y])
        norm_accel = np.linalg.norm(accel)
        if norm_accel > self.max_accel:
            accel = (accel / norm_accel) * self.max_accel

        # Update velocity
        self.velocity += accel * dt
        norm_velocity = np.linalg.norm(self.velocity)
        if norm_velocity > self.max_velocity:
            self.velocity = (self.velocity / norm_velocity) * self.max_velocity
        
        # Update position
        self.position += self.velocity * dt
        self.time += dt

        return self.position, self.velocity, accel



def out_a(target_p, ship, max_accel, max_velocity, dt = 0.1):
    mass = ship['disp']
    controller = TwoDPIDController(kp=0.2, ki=0.01, kd=0.1)
    simulator = TwoDMotionSimulator(mass, max_accel, max_velocity, controller=controller, initial_pos=np.array([0.0, 0.0], dtype=float))
    target_position = target_p  # 目标位置需要输入 

    position_tolerance = 0.1  # 到达目标的位置容差
    velocity_tolerance = 0.01  # 速度容差

    results = []
    t = 0
    while (np.linalg.norm(target_position - simulator.position) > position_tolerance or
        np.linalg.norm(simulator.velocity) > velocity_tolerance):
        pos, vel, accel = simulator.update(target_position, dt)
        results.append((simulator.time, pos.copy(), vel.copy(), accel.copy()))
        t += 1
        if t>=24: # 本来可以删掉的垃圾代码，但是这里是情怀
            break

    # 总加速度计算转动惯量和角加速度的乘积
    '''
    all_accel = []
    for _, _, _, accel in results:
        al = np.sqrt(np.square(accel[0]) + np.square(accel))
        all_accel.append(al)
    '''
        # 提取位置、速度和加速度数据
    times = [result[0] for result in results]
    positions = [result[1] for result in results]
    velocities = [result[2] for result in results]
    accelerations = [result[3] for result in results]
    # return times, positions,velocities,accelerations, all_accel
    values = accelerations[0]

# 计算平方和的开方
    result = np.linalg.norm(values)
    print('PID加速度：',result)
    return times, positions,velocities,accelerations,result


'''
state = [0, 0, 30, 0, 0, 0]
turn_p = [500, 800]
targ_p = [800, 800]
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

env_forces = {'wind_params':eval('10,30'),'wave_params':eval('50,-40,1')}
a_params ={'a_xline':1,'v_xline':5,'a_turn':0.5,'v_turn':2,'a_yline':0.2,'v_yline':1}

print(out_a(targ_p,ship_params, a_params['a_xline'],a_params['v_xline']))
'''

# 定义模拟参数
'''
controller = TwoDPIDController(kp=0.2, ki=0.01, kd=0.1)
simulator = TwoDMotionSimulator(mass=10, max_accel=2.0, max_velocity=5.0, controller=controller, initial_pos=np.array([0.0, 0.0], dtype=float))
target_position = np.array([100.0, 100.0])  # 目标位置需要输入
dt = 0.1  # 时间间隔也需要输入
position_tolerance = 0.1  # 到达目标的位置容差
velocity_tolerance = 0.01  # 速度容差

results = []
while (np.linalg.norm(target_position - simulator.position) > position_tolerance or
       np.linalg.norm(simulator.velocity) > velocity_tolerance):
    pos, vel, accel = simulator.update(target_position, dt)
    results.append((simulator.time, pos.copy(), vel.copy(), accel.copy()))

print("Simulation complete. Final position and velocity:", pos, vel)
'''

'''
# 提取位置、速度和加速度数据
times = [result[0] for result in results]
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



'''
import time

class PIDController:
    def __init__(self, c_x, b_x, i_x, t_int, x_wf, c_y, b_y, i_y, y_wf, c_phi, b_phi, i_phi, phi_wf):
        # X 方向的 PID 常数
        self.c_x = c_x  # X 方向的比例增益
        self.b_x = b_x  # X 方向的微分增益
        self.i_x = i_x  # X 方向的积分增益
        self.x_wf = x_wf  # X 方向的前馈项（偏置/常数项）

        # Y 方向的 PID 常数
        self.c_y = c_y  # Y 方向的比例增益
        self.b_y = b_y  # Y 方向的微分增益
        self.i_y = i_y  # Y 方向的积分增益
        self.y_wf = y_wf  # Y 方向的前馈项（偏置/常数项）

        # 旋转（Phi）方向的 PID 常数
        self.c_phi = c_phi  # Phi 方向的比例增益
        self.b_phi = b_phi  # Phi 方向的微分增益
        self.i_phi = i_phi  # Phi 方向的积分增益
        self.phi_wf = phi_wf  # Phi 方向的前馈项（偏置/常数项）

        self.t_int = t_int  # 积分时间
        
        # 初始化状态变量，用于积分和上一次误差的微分计算
        self.integral_x = self.integral_y = self.integral_phi = 0
        self.prev_error_x = self.prev_error_y = self.prev_error_phi = 0
        self.prev_time = time.time()  # 存储前一次的时间，用于计算 dt

    def update(self, x_setpoint, y_setpoint, phi_setpoint, x, y, phi):
        # 计算自上次更新以来的时间，用于积分和微分计算
        current_time = time.time()
        dt = current_time - self.prev_time

        # 计算设定点与当前位置之间的误差
        error_x = x_setpoint - x
        error_y = y_setpoint - y
        error_phi = phi_setpoint - phi

        # 计算误差的变化，用于微分项
        d_error_x = (error_x - self.prev_error_x) / dt
        d_error_y = (error_y - self.prev_error_y) / dt
        d_error_phi = (error_phi - self.prev_error_phi) / dt

        # 更新积分项中累积的误差
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        self.integral_phi += error_phi * dt

        # 使用 PID 公式计算所需的控制输出
        x_req = -error_x + self.c_x * error_x + self.b_x * d_error_x + (self.i_x / self.t_int) * self.integral_x + self.x_wf
        y_req = -error_y + self.c_y * error_y + self.b_y * d_error_y + (self.i_y / self.t_int) * self.integral_y + self.y_wf
        m_req = -error_phi + self.c_phi * error_phi + self.b_phi * d_error_phi + (self.i_phi / self.t_int) * self.integral_phi + self.phi_wf

        # 为下一次迭代存储当前误差和时间
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_phi = error_phi
        self.prev_time = current_time

        # 返回计算出的控制力
        return x_req, y_req, m_req
'''
# 使用适当的常数创建 PIDController 的实例
# 将这些常数替换为您实际应用中的具体值
'''
pid = PIDController(c_x=1.0, b_x=1.0, i_x=0.1, t_int=1.0, x_wf=0.0,
                    c_y=1.0, b_y=1.0, i_y=0.1, t_int=1.0, y_wf=0.0,
                    c_phi=1.0, b_phi=1.0, i_phi=0.1, phi_wf=0.0)

# 示例设定点和当前状态
# 将这些替换为您系统中的真实测量值和目标值
x_setpoint, y_setpoint, phi_setpoint = 0.0, 0.0, 0.0
x, y, phi = 0.0, 0.0, 0.0

# 更新 PID 控制器并获取控制力
x_req, y_req, m_req = pid.update(x_setpoint, y_setpoint, phi_setpoint, x, y, phi)

# 输出控制力（这些将被应用到您系统的执行机构）
print(f"控制力: X_req={x_req}, Y_req={y_req}, M_req={m_req}")
'''