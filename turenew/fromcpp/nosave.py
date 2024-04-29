import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoints):
        # 控制增益
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # 设定目标值
        self.setpoints = np.array(setpoints)
        # 累积误差
        self.integral = np.zeros_like(setpoints)
        # 上一次的误差
        self.previous_error = np.zeros_like(setpoints)
        # 控制输出
        self.output = np.zeros_like(setpoints)
        # 时间步长
        self.timestep = 0.05

    def update(self, measured_values):
        # 计算误差
        error = self.setpoints - np.array(measured_values)
        # 计算误差的积分
        self.integral += error * self.timestep
        # 计算误差的微分
        derivative = (error - self.previous_error) / self.timestep
        # 更新控制输出
        self.output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        # 更新上一次的误差
        self.previous_error = error
        # 限制输出，模拟您的Xmax, Ymax, Nmax等
        self.output = np.clip(self.output, [-2500000, -2000000, -100000000], [2500000, 2000000, 100000000])
        return self.output

# 示例：初始化PID控制器
pid_controller = PIDController(
    Kp=[0.001, 0.001, 0.001],
    Ki=[0.0, 0.0, 0.0],
    Kd=[0.0, 0.0, 0.0],
    setpoints=[0, 0, 0]  # 目标位置和姿态
)

# 更新控制输出
output = pid_controller.update([current_x_position, current_y_position, current_yaw_angle])
print("Control Outputs:", output)




import numpy as np

class Tugboat:
    def __init__(self, position, max_force, max_moment):
        self.position = position  # 拖轮相对于船舶质心的位置
        self.max_force = max_force
        self.max_moment = max_moment

    def calculate_force(self, force_demand, moment_demand):
        # 假设简单线性分配力和力瘫
        force = np.clip(force_demand, -self.max_force, self.max_force)
        moment = np.clip(moment_demand, -self.max_moment, self.max_moment)
        return force, moment

# 假设有两个拖轮
tug1 = Tugboat(position=(-50, 0), max_force=1000, max_moment=500)  # 船尾
tug2 = Tugboat(position=(50, 0), max_force=1000, max_moment=500)   # 船头

# 从PID控制器获取力和力瘫需求
force_demand = 500  # 需要500N推力
moment_demand = 200  # 需要200Nm力瘫

# 计算每个拖轮需要提供的力和力瘫
force1, moment1 = tug1.calculate_force(force_demand / 2, moment_demand / 2)
force2, moment2 = tug2.calculate_force(force_demand / 2, moment_demand / 2)

print("Tug 1 Force: {} N, Moment: {} Nm".format(force1, moment1))
print("Tug 2 Force: {} N, Moment: {} Nm".format(force2, moment2))



class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0

    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

# Initialize PID controllers for surge, sway, and yaw
pid_surge = PID(Kp=0.2, Ki=0.05, Kd=0.01)
pid_sway = PID(Kp=0.2, Ki=0.05, Kd=0.01)
pid_yaw = PID(Kp=0.1, Ki=0.02, Kd=0.005)

# Simulation step
dt = 0.1  # time step in seconds

# Assume these are measured or estimated from sensors
current_position_x = 100  # meters from target
current_position_y = 50   # meters from target
current_yaw = np.deg2rad(10)  # radians from target orientation

# PID updates
force_x = pid_surge.update(current_position_x, dt)
force_y = pid_sway.update(current_position_y, dt)
moment_n = pid_yaw.update(current_yaw, dt)

print("Total tug forces and moment to apply:")
print("Force X (Surge):", force_x, "N")
print("Force Y (Sway):", force_y, "N")
print("Moment N (Yaw):", moment_n, "Nm")




def ship_dynamics(state, t, ship, env_params, target_position, a_params, dt):
    x, y, psi, u, v, r = state
    p = params(state, ship)

    m = p.get('m', 0)
    I_zz = p.get('I_zz', 0)
    m_x = p.get('m_x', 0)
    m_y = p.get('m_y', 0)
    J_zz = p.get('J_zz', 0)

    turn_p = [300, 300]
    Tug = out_Tugs(state, turn_p, target_position, ship, env_params, a_params)
    if Tug is None:
        Tug = [0, 0, 0]

    # 避免重复计算环境力
    env = all_env_force(state, ship, env_params)
    if env is None:
        env = [0, 0, 0]

    print('tug:', Tug)
    print('env:', env)

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




import matplotlib.pyplot as plt
import numpy as np
def plot_simulation_results(timesteps, states, forces):
    x, y, psi, u, v, r = states.T  # 转置并解包状态数据
    Fx, Fy, N = forces.T  # 转置并解包力和力瘫数据

    fig, ax = plt.subplots(3, 2, figsize=(14, 10))  # 创建一个3x2的图表布局
    ax[0, 0].plot(timesteps, x, label='X Position')
    ax[0, 0].plot(timesteps, y, label='Y Position')
    ax[0, 0].set_title('Ship Position Over Time')
    ax[0, 0].set_xlabel('Time (s)')
    ax[0, 0].set_ylabel('Position (meters)')
    ax[0, 0].legend()

    ax[0, 1].plot(timesteps, psi, label='Heading')
    ax[0, 1].set_title('Ship Heading Over Time')
    ax[0, 1].set_xlabel('Time (s)')
    ax[0, 1].set_ylabel('Heading (radians)')

    ax[1, 0].plot(timesteps, u, label='Surge Speed')
    ax[1, 0].plot(timesteps, v, label='Sway Speed')
    ax[1, 0].set_title('Ship Speed Over Time')
    ax[1, 0].set_xlabel('Time (s)')
    ax[1, 0].set_ylabel('Speed (m/s)')
    ax[1, 0].legend()

    ax[1, 1].plot(timesteps, r, label='Yaw Rate')
    ax[1, 1].set_title('Ship Yaw Rate Over Time')
    ax[1, 1].set_xlabel('Time (s)')
    ax[1, 1].set_ylabel('Yaw Rate (rad/s)')

    ax[2, 0].plot(timesteps, Fx, label='Tug Force X')
    ax[2, 0].plot(timesteps, Fy, label='Tug Force Y')
    ax[2, 0].set_title('Tug Forces Over Time')
    ax[2, 0].set_xlabel('Time (s)')
    ax[2, 0].set_ylabel('Force (N)')
    ax[2, 0].legend()

    ax[2, 1].plot(timesteps, N, label='Tug Moment N')
    ax[2, 1].set_title('Tug Moment Over Time')
    ax[2, 1].set_xlabel('Time (s)')
    ax[2, 1].set_ylabel('Moment (Nm)')

    plt.tight_layout()
    plt.show()

# 假设以下数据从仿真脚本中获取
timesteps = np.linspace(0, 100, 1000)  # 100秒内1000个时间点
states = np.random.random((1000, 6))  # 假设的状态数据
forces = np.random.random((1000, 3))  # 假设的力和力瘫数据

plot_simulation_results(timesteps, states, forces)




import numpy as np
from scipy.optimize import minimize

class NMPCController:
    def __init__(self, T=9.0, lambda_weights=[20.0, 20.0, 1e-8], control_horizon=10):
        self.T = T
        self.lambda_weights = lambda_weights
        self.control_horizon = control_horizon
        self.state_dim = 3  # Assuming a 3D state: x, y, psi
        self.control_dim = 3  # Assuming 3 control inputs: force_x, force_y, moment_n

    def model_predictive_control(self, current_state, target_state):
        # Define the cost function
        def cost_function(U):
            U = np.reshape(U, (self.control_horizon, self.control_dim))
            state = current_state.copy()
            cost = 0
            for i in range(self.control_horizon):
                # Predict next state based on current control input
                state = self.state_update(state, U[i])
                # Calculate cost
                state_error = state - target_state
                cost += self.lambda_weights[0] * (state_error**2).sum()
                cost += self.lambda_weights[1] * (U[i]**2).sum()  # Control effort
            return cost

        # Initial guess for the control inputs
        initial_guess = np.zeros(self.control_horizon * self.control_dim)
        # Constraints might be needed to ensure feasible control inputs
        bounds = [(-1000, 1000)] * (self.control_horizon * self.control_dim)

        # Solve the optimization problem
        result = minimize(cost_function, initial_guess, bounds=bounds, method='SLSQP')
        if not result.success:
            raise Exception("Optimization failed: " + result.message)

        # Reshape result to control input format
        optimal_controls = np.reshape(result.x, (self.control_horizon, self.control_dim))
        return optimal_controls[0]  # Return the first optimal control action

    def state_update(self, state, control_input):
        # Simulate the state update, this should be your actual ship dynamics
        # For example, this is a simple integrator update (Euler method)
        x, y, psi = state
        fx, fy, mn = control_input
        dt = self.T / self.control_horizon
        x += dt * fx
        y += dt * fy
        psi += dt * mn
        return np.array([x, y, psi])

# Example usage
controller = NMPCController()
current_state = np.array([0, 0, 0])
target_state = np.array([100, 100, np.pi/4])
control_input = controller.model_predictive_control(current_state, target_state)
print("Control Input:", control_input)





import numpy as np
import math

class WOPC:
    def __init__(self, x_des, y_des):
        self.x_des = x_des
        self.y_des = y_des
        self.psi_c = 0.0
        self.psi_d = 0.0
        self.pre_psi_c = 0.0
        self.m_add = 0.004
        self.m_gain = 0.05
        self.h_gain = 1.0
        self.p_gain = 1.0
        self.k_gain = 1.0
        self.up_lmt = 0.03
        self.low_lmt = 0.0
        self.rad = 60.0
        self.used_flag = False
        self.x_force = 1.0
        self.y_force = 0.0
        self.state = 6
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.accumulate = 0.0









    def center_control(self):
        self.x_center = self.x_des + self.rad * math.cos(self.psi_d)
        self.y_center = self.y_des + self.rad * math.sin(self.psi_d)

    def opt_head_cal(self):
        self.psi_c = math.atan2(self.y_center - self.y, self.x_center - self.x)
        self.psi_c = self.adjust_angle(self.psi_c)
        if not self.used_flag:
            self.psi_d = math.atan2(self.y_des - self.y, self.x_des - self.x)
            self.psi_d = self.adjust_angle(self.psi_d)
            self.used_flag = True

        psi_cd = self.psi_c - self.psi_d
        psi_cd = self.adjust_angle(psi_cd)
        self.psi_d += self.k_gain * psi_cd * self.p_gain

    def wohc(self):
        self.x_rt_des = self.x_center - self.rad * math.cos(self.psi_c)
        self.y_rt_des = self.y_center - self.rad * math.sin(self.psi_c)
        self.psi_rt_des = self.psi_c

    def adjust_angle(self, angle):
        if angle < -math.pi:
            return angle + 2 * math.pi
        elif angle > math.pi:
            return angle - 2 * math.pi
        return angle






    def calculate_control(self, eta, nu, force):
        self.set_eta(eta)
        self.set_nu(nu)
        self.set_thrust(force)
        self.center_control()
        self.wohc()
        self.opt_head_cal()

        # 输出控制和状态
        return {
            'center': (self.x_center, self.y_center),
            'rt_pos_des': (self.x_rt_des, self.y_rt_des),
            'psi_rt_des': self.psi_rt_des
        }






# 主控制脚本

from ship_dynamics import ShipDynamics
from environmental_forces import EnvironmentalForces
from tugboat_forces import TugboatForces
from control_system import ControlSystem
import numpy as np

def run_simulation():
    # 初始化模块
    ship_dynamics = ShipDynamics()
    env_forces = EnvironmentalForces()
    tug_forces = TugboatForces()
    control_system = ControlSystem()

    # 初始状态和环境参数
    initial_state = np.array([0, 0, 0, 0, 0, 0])  # x, y, psi, u, v, r
    env_params = {'wind_speed': 5, 'current_speed': 1}
    target_position = np.array([100, 100])

    # 仿真循环
    time = 0
    dt = 0.1
    while time < 100:
        env_force = env_forces.calculate_forces(env_params)
        control_input = control_system.calculate_control(initial_state, target_position)
        tug_force = tug_forces.calculate_forces(control_input)
        next_state = ship_dynamics.update_state(initial_state, env_force, tug_force, dt)

        print("Time:", time, "State:", next_state)
        initial_state = next_state
        time += dt

if __name__ == "__main__":
    run_simulation()







def calculate_control(ship_state, target_position, current_heading, pid_controller):
    x, y, psi, u, v, r = ship_state
    target_x, target_y = target_position
    heading_to_target = math.atan2(target_y - y, target_x - x)

    # 判断是否需要转向
    heading_error = heading_to_target - psi
    heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  # 规范到 [-pi, pi]

    if abs(heading_error) > math.radians(10):  # 转向阈值，可调
        mode = "Turning"
    elif math.sqrt((x - target_x)**2 + (y - target_y)**2) > 10:  # 靠近目标点之前保持直线航行
        mode = "Straight"
    else:
        mode = "Parallel Parking"

    # 根据模式调整 PID 控制器的目标
    if mode == "Turning":
        pid_controller.set_target(heading_to_target)
    elif mode == "Straight":
        pid_controller.set_target(0)  # 直线航行，不调整方向
    elif mode == "Parallel Parking":
        # 平行靠泊，需要详细设计
        pass

    control_actions = pid_controller.update(ship_state)
    return control_actions, mode




def rk4_step(state, control_input, dt):
    """ 执行一个 RK4 步骤来更新船舶状态 """
    k1 = ship_dynamics(state, control_input)
    k2 = ship_dynamics(state + 0.5 * dt * k1, control_input)
    k3 = ship_dynamics(state + 0.5 * dt * k2, control_input)
    k4 = ship_dynamics(state + dt * k3, control_input)
    new_state = state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
    return new_state





import matplotlib.pyplot as plt

def plot_results(timeline, states, controls, modes):
    plt.figure()
    plt.title("Ship Trajectory")
    plt.plot([state[0] for state in states], [state[1] for state in states], label='Path')
    plt.scatter([states[0][0]], [states[0][1]], color='green', label='Start')
    plt.scatter([states[-1][0]], [states[-1][1]], color='red', label='End')
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.title("Control Actions Over Time")
    plt.plot(timeline, [control[0] for control in controls], label='X Force')
    plt.plot(timeline, [control[1] for control in controls], label='Y Force')
    plt.xlabel("Time")
    plt.ylabel("Control Force")
    plt.legend()
    plt.grid(True)

    plt.show()




def calculate_control(ship_state, target_position, current_heading, pid_controller):
    x, y, psi, u, v, r = ship_state
    target_x, target_y = target_position

    # 计算目标航向和当前位置的角度差
    desired_heading = math.atan2(target_y - y, target_x - x)
    heading_error = normalize_angle(desired_heading - psi)

    # 根据位置差距选择控制模式
    distance_to_target = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    if distance_to_target > 50:  # 距离大于50米时，可能需要转向
        control_mode = "Turning"
    elif distance_to_target > 10:  # 距离介于10米到50米之间，执行直线航向
        control_mode = "Straight"
    else:  # 距离小于10米，执行平行靠泊
        control_mode = "Parallel Parking"

    # 根据控制模式设置PID目标
    if control_mode == "Turning":
        pid_controller.set_target(desired_heading)
    elif control_mode == "Straight":
        pid_controller.set_target(desired_heading)  # 保持当前方向
    elif control_mode == "Parallel Parking":
        # 平行靠泊可能需要特定的逻辑处理
        pid_controller.set_target(align_with_dock(ship_state, dock_info))

    # 更新PID控制器并获取控制动作
    control_actions = pid_controller.update(ship_state, heading_error)

    return control_actions, control_mode








# 故障检测伪代码
def check_sensor_data(sensor_data):
    if not sensor_data.is_valid():
        raise SensorError("Sensor data is invalid")

def main_control_loop():
    try:
        sensor_data = read_sensor_data()
        check_sensor_data(sensor_data)
        update_environment_model(sensor_data)
        control_action = control_system.decide_action()
        execute_action(control_action)
    except SensorError as e:
        handle_sensor_failure(e)
    except Exception as e:
        log_error(e)
        move_to_safe_state()




class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0

    def update(self, current_value, dt):
        error = self.set_point - current_value
        self.integral += error * dt
        self.derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
        return output

    def set_target(self, target):
        self.set_point = target



def control_ship(ship_state, target_position, pid_heading, pid_position, dt):
    x, y, psi, u, v, r = ship_state
    target_x, target_y = target_position

    # Calculate errors
    distance = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    desired_heading = math.atan2(target_y - y, target_x - x)
    heading_error = desired_heading - psi

    # Determine control mode
    if distance > 50:  # Long distance requires turning mode
        mode = "Turning"
        control_output = pid_heading.update(psi, dt)
        # Assume control_output affects rudder or thrusters for turning
        force_x = 0
        force_y = max(min(control_output, max_force), -max_force)  # Saturation limit
    elif distance > 10:
        mode = "Straight"
        control_output = pid_position.update(distance, dt)
        force_x = max(min(control_output, max_force), -max_force)  # Drive forward
        force_y = 0
    else:
        mode = "Docking"
        # Implement docking control strategy here, potentially another PID or more complex logic
        force_x = 0
        force_y = 0

    # Update ship dynamics or simulate command execution
    update_ship_dynamics(ship_state, force_x, force_y, dt)
    return ship_state, (force_x, force_y), mode



import numpy as np

# Simulation parameters
dt = 0.1
simulation_time = 100
ship_state = [0, 0, 0, 0, 0, 0]  # Initial state: x, y, psi, u, v, r

# Initialize PID controllers
pid_heading = PIDController(0.2, 0.0, 0.01)
pid_position = PIDController(0.1, 0.01, 0.005)
pid_heading.set_target(0)  # Target heading is north initially

# Run simulation
for t in np.arange(0, simulation_time, dt):
    ship_state, control_output, mode = control_ship(
        ship_state, (100, 100), pid_heading, pid_position, dt)
    print(f"Time {t}: Mode {mode}, Control Output {control_output}")




def ship_dynamics(state, controls, dt):
    x, y, psi, u, v, r = state
    fx, fy, m = controls  # 接收来自PID控制器或决策系统的力和力矩

    # 更新速度和位置
    # 注意这里需要结合船舶质量、惯性等物理参数
    u_dot = (fx - d * u) / m  # 假设d为线性阻力系数，m为船舶质量
    v_dot = (fy - d * v) / m
    r_dot = m / I_zz  # I_zz为转动惯量

    # 简单的欧拉积分更新状态
    u += u_dot * dt
    v += v_dot * dt
    r += r_dot * dt

    # 更新位置
    x += (u * math.cos(psi) - v * math.sin(psi)) * dt
    y += (u * math.sin(psi) + v * math.cos(psi)) * dt
    psi += r * dt

    return x, y, psi, u, v, r






def calculate_environmental_forces(ship_state, environmental_conditions):
    # 根据风、流和波的数据计算合力和力矩
    wind_force, current_force, wave_force = environmental_conditions
    total_force_x = wind_force[0] + current_force[0] + wave_force[0]
    total_force_y = wind_force[1] + current_force[1] + wave_force[1]
    total_moment = wind_force[2] + current_force[2] + wave_force[2]
    return total_force_x, total_force_y, total_moment

def adjust_tug_forces(ship_state, target_position, pid_controller, environmental_conditions):
    total_env_force_x, total_env_force_y, total_env_moment = calculate_environmental_forces(ship_state, environmental_conditions)
    
    # 计算由PID控制器所需的船舶调整力和力矩
    control_force_x, control_force_y, control_moment = pid_controller.update(ship_state, target_position)
    
    # 调整拖轮力以克服环境力影响
    adjusted_force_x = control_force_x - total_env_force_x
    adjusted_force_y = control_force_y - total_env_force_y
    adjusted_moment = control_moment - total_env_moment
    
    return adjusted_force_x, adjusted_force_y, adjusted_moment


