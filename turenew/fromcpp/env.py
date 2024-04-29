import numpy as np

class Current:
    NUM = 37  # 定义数组大小常量

    def __init__(self):
        self.rho_w = 1025  # 水密度
        self.Lpp = 162.0  # 船体长度
        self.Tm = 7.2  # 平均吃水深度
        self.cxc = np.array([-0.034, -0.032, -0.031, -0.031, -0.030, -0.028, -0.022, -0.016, -0.007, 0.003,
                             0.016, 0.029, 0.039, 0.043, 0.042, 0.040, 0.035, 0.027, 0.019, 0.010, 0.004,
                             -0.001, -0.004, -0.006, -0.006, -0.004, 0.000, 0.006, 0.015, 0.023, 0.031,
                             0.036, 0.041, 0.043, 0.045, 0.047, 0.048])
        self.cyc = np.array([0.00, 0.04, 0.07, 0.11, 0.18, 0.21, 0.24, 0.29, 0.33, 0.37, 0.41, 
                            0.45, 0.47, 0.51, 0.54,	0.56, 0.58, 0.59, 0.60, 0.59, 0.58, 0.56, 0.55, 0.52, 
                            0.48, 0.45, 0.41, 0.38, 0.35, 0.28,	0.24, 0.20, 0.16, 0.12, 0.08, 0.04, 0.00])


        self.cnc = np.array([0, 0.0075, 0.0175, 0.0275, 0.0350, 0.0400, 0.0450,	0.0475, 0.0525, 0.0525, 
                             0.0500,	0.0475,0.0425, 0.0350,	0.0275, 0.0175, 0.0090, -0.0050, -0.0150, 
                            -0.0250, -0.0350,	-0.0475, -0.0575, -0.0650, -0.0725,-0.0775, -0.0800, 
                            -0.0825, -0.0800, -0.0775, -0.0725, -0.0650, -0.0600, -0.0475, -0.0350, -0.0200, -0.0100])
        self.Vc = 0.0
        self.dirC = 0.0
        self.psi = 0.0

    def set_parameters(self, speed, direction):
        self.Vc = speed
        self.dirC = np.radians(direction)  # 将角度转换为弧度

    def set_psi(self, psi):
        self.psi = np.radians(psi)  # 将角度转换为弧度

    def calculate_forces(self):
        beta = (self.psi - self.dirC) % (2 * np.pi) - np.pi  # 计算相对流向角
        cxc_value = self.interp(self.cxc, abs(beta))
        cyc_value = self.interp(self.cyc, abs(beta))
        cnc_value = self.interp(self.cnc, abs(beta))

        x_force = 0.5 * cxc_value * self.rho_w * self.Vc**2 * self.Lpp * self.Tm
        y_force = 0.5 * cyc_value * self.rho_w * self.Vc**2 * self.Lpp * self.Tm * np.sign(beta)
        n_moment = 0.5 * cnc_value * self.rho_w * self.Vc**2 * self.Lpp**2 * self.Tm * np.sign(beta)

        return {'surge': x_force, 'sway': y_force, 'yaw': n_moment}

    def interp(self, cc, bt):
        pi_over_num = np.pi / (self.NUM - 1)
        index = int(bt / pi_over_num)
        if index >= self.NUM - 1:  # 防止数组越界
            return cc[-1]
        fraction = (bt - index * pi_over_num) / pi_over_num
        return cc[index] + (cc[index + 1] - cc[index]) * fraction

# 示例使用
current = Current()
current.set_parameters(10, 45)  # 流速10 m/s, 流向45度
current.set_psi(90)  # 船舶朝向90度
forces = current.calculate_forces()
print("Forces:", forces)







import numpy as np

class Wind:
    def __init__(self):
        # 船舶形体参数
        self.L_oa = 175
        self.B0 = 25.4
        self.H_s = 10
        self.A_s = 1700
        self.A_f = 250
        self.A_ss = 175
        self.wind_c = 150
        self.wind_e = 80
        self.wind_M = 1
        self.rho_a = 1.224
        self.V_T = 15
        self.ang_T = 0.0
        self.psi = 0.0
        self.Cx = 0.0
        self.Cy = 0.0
        self.Cn = 0.0
        self.coefficients_a = np.array([...])  # 请在此处填充系数矩阵a的实际值
        self.coefficients_b = np.array([...])  # 请在此处填充系数矩阵b的实际值
        self.coefficients_c = np.array([...])  # 请在此处填充系数矩阵c的实际值

    def set_parameters(self, speed_true, angle_true):
        self.V_T = speed_true
        self.ang_T = np.radians(angle_true)

    def set_heading(self, head):
        self.psi = head

    def calculate_forces(self):
        # 计算遭遇角
        ang_enc = self.psi - self.ang_T - np.pi

        # 计算相对风速
        V_R = np.sqrt((self.nu.u - self.V_T * np.cos(self.ang_T - self.psi))**2 +
                      (self.nu.v - self.V_T * np.sin(self.ang_T - self.psi))**2)

        # 从矩阵中差值获取Cx, Cy, Cn
        Cx = self.interp(self.coefficients_a, ang_enc)
        Cy = self.interp(self.coefficients_b, ang_enc)
        Cn = self.interp(self.coefficients_c, ang_enc)

        # 计算风力和力矩
        x_force = 0.5 * self.rho_a * V_R**2 * Cx * self.A_f
        y_force = 0.5 * self.rho_a * V_R**2 * Cy * self.A_s
        n_moment = 0.5 * self.rho_a * V_R**2 * Cn * self.A_s * self.L_oa

        return {'surge': x_force, 'sway': y_force, 'yaw': n_moment}

    def interp(self, coefficients, angle):
        # 以10度为间隔进行插值
        index = int(np.abs(angle) // np.radians(10))
        fraction = (np.abs(angle) % np.radians(10)) / np.radians(10)
        value = coefficients[index] + (coefficients[index + 1] - coefficients[index]) * fraction
        return value

# 使用示例
wind = Wind()
wind.set_parameters(10, 45)  # 风速10m/s, 风向45度
wind.set_heading(np.radians(90))  # 船舶朝向90度
forces = wind.calculate_forces()
print("Wind Forces:", forces)






import numpy as np
import time

class Wave:
    def __init__(self, dir=120, Hs=3.0):
        self.psiMean = np.radians(dir)
        self.hs = Hs
        self.nFreq = 20
        self.nDir = 10
        self.omegaVec = np.linspace(0, 2*np.pi, int(self.nFreq*self.nDir))
        self.Zeta = np.zeros(int(self.nFreq*self.nDir))
        self.Omega = np.zeros(int(self.nFreq*self.nDir))
        self.Phase = np.random.uniform(0, 2*np.pi, int(self.nFreq*self.nDir))
        self.WaveNum = np.zeros(int(self.nFreq*self.nDir))
        self.Psi = np.zeros(int(self.nFreq*self.nDir))
        self.init()

    def init(self):
        # Initialize wave characteristics
        self.omegaPeak = 2.0 * np.pi / (4.883 + 2.68 * np.sqrt(self.hs))
        self.deltaOmega = self.omegaPeak / self.nFreq
        self.deltaPsi = 2 * np.pi / self.nDir

        # Initialize wave spectrum and directions
        for i in range(int(self.nFreq)):
            for j in range(int(self.nDir)):
                idx = i * int(self.nDir) + j
                self.Omega[idx] = i * self.deltaOmega
                self.Psi[idx] = j * self.deltaPsi
                self.WaveNum[idx] = self.Omega[idx]**2 / 9.81  # Simple deep water relation
                self.Zeta[idx] = np.sqrt(2 * self.waveSpec(self.hs, self.omegaPeak, self.Omega[idx]) * self.deltaOmega * self.deltaPsi)

    def waveSpec(self, Hs, Tp, omega):
        # ITTC-Modified Pierson-Moskowitz Spectrum
        A = 487.0 * Hs**2 / Tp**4
        B = 1949 / Tp**4
        return A / (omega**5 * np.exp(B / (omega**4)))

    def get_wave_forces(self):
        # Placeholder function to compute wave forces based on the spectrum
        Fx = np.sum(self.Zeta * np.cos(self.Phase))
        Fy = np.sum(self.Zeta * np.sin(self.Phase))
        return Fx, Fy

# Example usage
wave_model = Wave(120, 3)
Fx, Fy = wave_model.get_wave_forces()
print("Wave Forces: Fx = {}, Fy = {}".format(Fx, Fy))
