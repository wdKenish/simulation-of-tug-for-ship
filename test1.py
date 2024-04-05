import numpy as np
import matplotlib.pyplot as plt

class ShipBerthingSimulator:
    def __init__(self, ship_pos, tug_pos, target_pos, kp, ki, kd, sim_time, dt):
        self.ship_pos = np.array(ship_pos)   # 船舶初始位置
        self.tug_pos = np.array(tug_pos)     # 拖轮初始位置
        self.target_pos = np.array(target_pos)   # 目标位置
        self.kp = kp   # PID控制器参数
        self.ki = ki
        self.kd = kd
        self.sim_time = sim_time   # 仿真时间
        self.dt = dt   # 时间步长

        self.integral = 0   # PID控制器状态
        self.prev_error = 0

        self.ship_traj = [self.ship_pos.copy()]   # 存储船舶和拖轮的历史轨迹
        self.tug_traj = [self.tug_pos.copy()]

    def run_simulation(self):
        for t in np.arange(0, self.sim_time, self.dt):
            # 计算船舶和目标位置之间的距离
            error = self.target_pos - self.ship_pos

            # 计算PID控制器输出
            derivative = (error - self.prev_error) / self.dt
            self.integral += error * self.dt
            control = self.kp * error + self.ki * self.integral + self.kd * derivative

            # 更新拖轮位置
            self.tug_pos += control

            # 更新船舶位置
            self.ship_pos += control * self.dt

            # 存储历史数据
            self.ship_traj.append(self.ship_pos.copy())
            self.tug_traj.append(self.tug_pos.copy())

            # 如果船舶到达目标位置，退出循环
            if np.linalg.norm(error) < 0.1:
                break

    def plot_simulation(self):
        # 可视化结果
        ship_traj = np.array(self.ship_traj)
        tug_traj = np.array(self.tug_traj)

        plt.plot(ship_traj[:, 0], ship_traj[:, 1], label='Ship')
        plt.plot(tug_traj[:, 0], tug_traj[:, 1], label='Tug')
        plt.scatter(self.target_pos[0], self.target_pos[1], color='red', marker='x', label='Target')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Ship Berthing Simulation')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

# 定义仿真参数
ship_pos = [0, 0]
tug_pos = [10, 0]
target_pos = [20, 0]
kp = 0.5
ki = 0.1
kd = 0.2
sim_time = 30
dt = 0.1

# 创建仿真器对象并运行仿真
simulator = ShipBerthingSimulator(ship_pos, tug_pos, target_pos, kp, ki, kd, sim_time, dt)
simulator.run_simulation()

# 可视化仿真结果
simulator.plot_simulation()