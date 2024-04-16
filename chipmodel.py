# 这里就作为船舶和拖轮的参数定义


# 计算船体水动力导数
import math
def params(disp=55126,
         Lpp=181.6,
         width=30.5,
         stem_draft=11.82,
         stern_draft=11.82,
         draft=11.82,
         rho_water=1.025,
         x_0=-0.85,
         C_b=0.821): 
    rho = rho_water
    # L为船长，B为船宽，d为船舶吃水， C_b为船舶的方形系数
    L = Lpp
    B = width
    d = draft
    d_m = d # 都是吃水
    m = disp 
    L_w = m / (C_b * d * rho) / (0.85 * B)  # 0.85是人为添加的参数，因为不知道水线面的宽度
    V_ship = m / rho  # 船舶排水体积
    tao = (stern_draft - stem_draft ) / d  # 白军计算水动力X_vr时所用的无量纲吃水差
    
    m_x = (1 / 100 * (0.398 + 11.97 * C_b * (1 + 3.73 * d / B) 
                    - 2.89 * C_b * L / B * (1 + 1.13 * d / B)
                    + 0.175 * C_b * (L / B) ** 2 
                    * (1 + 0.541 * d / B) 
                    - 1.107 * L / B * d / B)) * m
    m_y = ((0.882 - 0.54 * C_b * (1 - 1.6 * d / B) 
            - 0.156 * L / B * (1 - 0.673 * C_b)
            + 0.826 * d / B * L / B *(1 - 0.678 * d / B) 
            - 0.638 * C_b * d / B * L / B* (1 - 0.669 * d / B))) * m 
    I_zz = ((m / 100 * L * (33 - 76.85 * C_b * (1 - 0.784 * C_b) 
                            + 3.43 * L / B * (1 - 0.63 * C_b)))) ** 2 * m 
    

    S = (1.54 * d_m + 0.45 * B + 0.904 * B * C_b + 0.026 * C_b * B * d_m) * L_w # S是船体湿面积
    miu = 1.05372 * 10 ** (-6) # m^2/s  海水在20摄氏度时，roh = 1024.73 kg/m^3,miu=
    R_e = V * L / miu # 雷诺数
    C_r = 0.01 * (35 * V_ship / L ** 3) ** 0.5  #剩余阻力
    #C_f = 0.083 / (math.log(R_e) - 1.65) ** 2 # 摩擦阻力系数R_e可能为0
    if R_e > 0:
        C_f = 0.083 / (math.log(R_e) - 1.65) ** 2
    else:
        C_f = 0  
    if L <= 150:
        # 粗糙度补贴系数的选取
        delta_C_AR = 0.00035 # 应该插值
    elif L <= 210:
        delta_C_AR = 0.0002
    elif L <= 260:
        delta_C_AR = 0.0001
    elif L <= 300:
        delta_C_AR = 0
    elif L <= 350:
        -0.0001
    else:
        delta_C_AR = -0.00025
    C_t = C_f + C_r + delta_C_AR # 总阻力系数
    X_u_ = - S / L / d_m * C_t * u ** 2 # X_u_是船舶直行阻力
    #X_vr = (1.6757 * C_b - 1.5054 ) * lambda22   # 没找到相关文献
    X_vr = (1.11 * C_b - 1.07 ) * m_y + (1.11 * C_b - 0.07) * m_y * (0.208 * tao)
    X_vv = 0.4 * B / L - 0.006 * L / d_m 
    X_rr = 0.0003 * L / d_m 
    # X_H是船舶的纵向力
    # 2.横向水动力及船艏摇力矩 沿船舷方向
    Y_v = -5 * (d_m / L) ** 2
    Y_r = 1.02 * (d_m / L) ** 2
    N_v = -1.94 * (d_m / L) ** 2
    N_r = -0.65 * (d_m / L) ** 2
    Y_v1v = 0.07 - 6.48 * (1 - C_b) * d_m / B
    Y_r1r = 0.0045 - 0.445 * (1 - C_b) * d_m / B
    Y_v1r = -0.3791 + 1.28 * (1 - C_b) * d_m / B
    N_r1r = -0.0805 + 8.6092 * (C_b * B / L) - 36.9816 * (C_b * B / L) ** 2
    N_vvr = -6.0856 + 137.4736 * (C_b * B / L) - 1029.514 * (C_b * B / L) ** 2 + 2480.6082 * (C_b * B / L) ** 3
    N_vrr = -0.0635 + 0.04414 * (C_b * d_m / B)
    return m,m_x,m_y,I_zz,X_u_,X_rr,X_vr,X_vv,Y_v,Y_r,Y_v1r,Y_v1v,Y_r1r,Y_r,N_r,N_v,N_r1r,N_vvr,N_vrr

# 计算船体水动力
def hull_force(u,v,r,v1,r1):
    m,m_x,m_y,I_zz,X_u_,X_rr,X_vr,X_vv,Y_v,Y_r,Y_v1r,Y_v1v,Y_r1r,Y_r,N_r,N_v,N_r1r,N_vvr,N_vrr = ship()
    X_H = X_u_ + X_vv * v ** 2 + X_vr * v * r + X_rr * r ** 2  # 纵向水动力，沿船中方向
    Y_H = Y_v * v + Y_r * r + Y_v1v * v1 * v + Y_r1r * r1 * r + Y_v1r * v1 * r 
    N_H = N_v * v + N_r * r + N_vvr * v * v * r + N_vrr * v * r * r + N_r1r * r1 * r
    return X_H,Y_H,N_H

# 环境模块
# current
# 流的计算（from ship to ship）
def current(u_b = u,v_b = v，U_c,angle_cur,angle_ship):
    # 直接计算流对船速的影响
    angle_c_to_ship = angle_ship - angle_cur
    acts = angle_c_to_ship
    u_r = u_b -U_c * math.cos(angle_c_to_ship)
    v_r = v_b +U_c * math.sin(angle_c_to_ship)
    U_r = (u_r ** 2 + v_r ** 2) **0.5
    U = U_r 
    u = u_r
    v = v_r
    r = acts1
    # acts1是acts的一阶导数，没有解决！
    '''
    得到流作用下的船的绝对速度，x方向速度，y方向速度和艏摇角速度
    '''
    return U,u,v,r

def wind_force(U_T,Psi):
    # 风速，风的攻角（风舷角）
    # from Cruising Performance of a Large Passager Ship in Heavey Sea
    rho = 1025 # density of wahter
    A_F = ship[Ax]
    A_L = ship[Ay]
    u = ship[u]
    v = ship[v]
    phi = ship[r]
    L_OA = ship[Loa]
    B = ship[breadth]
    H_BR = ship[Draft] # 总的高度，对于滚装船就是型深
    C = 0 # 中心和重心重合 = 0
    A_OD = A_L # 这里指上层建筑的侧向投影面积，但是对于滚装船来说就是Ay
    H_C = 0.5 * (ship[Draft]-ship[draft])# 船舶水面以上的体积在横向投影的中心高度，可以假设为型深减去吃水的0.5倍
    u_x = U_T * math.cos(Psi) + u  # u 是船舶x方向速度 = U * cos(belta)
    u_y = U_T * math,sin(Psi) - v  # 同理
    U_A= (u_x**2 + u_y**2) **0.5
    q_A = 0.5 * rho * U_A ** 2
    Psi_A = math.arctan(u_x/u_y)
    C_H = 0.355 * phi + 1.0
    alpha = [0.404, 0.368, 0.902]
    beta1 = [-0.922, 0.507, 1.162]
    beta2 = [0.018, -5.091, 10.367, -3.011, -0.341]
    gamma1 = [0.116, 3.345]
    gamma2 = [0.446, 2.192]
    delta1 = [0.458, 3.245, -2.313, 24.407, -5.481]
    delta2 = [-1.901, 12.727]
    epsilon1 = [-0.585, -0.906, 3.239]
    epsilon2 = [-0.314, -1.117]
    if Psi <= 90 and Psi >= 0:
        C_LF = beta1[0] + beta1[1] * A_L / L_OA / B + beta1[2] * C / L_OA
        C_YM = gamma1[0] + gamma1[1] * A_F / L_OA / B
        C_XLI = delta1[0] + delta1[1] * A_L / L_OA / H_BR + delta1[2] * A_F / B / H_BR
        C_ALF = epsilon1[0] + epsilon1[1] * A_OD / A_L + epsilon1[2] * B / L_OA
    else:
        C_LF = beta2[0] + beta2[1] * B / L_OA  + beta2[2] * H_C / L_OA + beta2[3] * A_OD / L_OA ** 2 + beta2[4] * A_F / B **2
        C_YM = gamma2[0] + gamma2[1] * A_OD / L_OA **2
        C_XLI = delta2[0] + delta2[1] * A_L / L_OA / H_BR + delta2[2] * A_F / A_L + delta2[3] * B / L_OA + delta2[4] * A_F / B / H_BR
        C_ALF = epsilon2[0] + epsilon2[1] * A_OD / A_L
    C_CF = alpha[0] + alpha[1] * A_F / B /H_BR + alpha[2] * H_BR / L_OA
    C_YLI = math.pi * A_L / L_OA **2 + C_YM
    C_AX = (C_LF * cos(Psi_A) 
            + C_XLI * (math.sin(Psi_A)-0.5 * math.sin(Psi_A) * math.cos(Psi_A) **2) * math.sin(Psi_A) * math.cos(Psi_A)
            + C_ALF * math.sin(Psi_A) * math.cos(Psi_A) **3)
    C_AY = (C_CF * math.sin(Psi_A)**2
            + C_YLI * (math.cos(Psi_A) + 0.5 * math.sin(Psi_A) **2 * math.cos(Psi_A))* math.sin(Psi_A) * math.cos(Psi_A))
    C_AN = C_AY * (0.927 * C / L_OA - 0.149 * (Psi_A - 0.5 * math.pi))
    X_wind = C_AX * q_A * A_F
    Y_wind = C_H * C_AY * q_A * A_L
    N_wind = C_H * C_AN * q_A * A_L * L_OA
    return X_wind,Y_wind,N_wind

def wave(H, w_L, T, angle_wave,angle_ship, t): # 输入波高、波长、周期、遭遇角、对应时刻t
    w_a = 0.5 * H
    w_K = 2 * math.pi / w_L
    w_omega = math.pi * 2 / T
    alpha_w = angle_ship - angle_wave # 遭遇角要进行计算
    """
    计算船舶上的波浪诱导力和力矩。

    参数:
    - w_a: 波浪振幅
    - w_k: 波数
    - alpha_w: 与波浪的相遇角度,###需要计算###
    - g: 重力加速度
    - w_omega: 相遇频率
    - t: 时间
    - B: 船舶的宽度（船宽）
    - d: 吃水深度
    - L: 船长
    - Cb: 方形系数
    - rho_water: 水的密度
    - Zb: 船舶浮力中心的垂直坐标
    """
    g = 9.8
    rho_water = 1025
    B = ship['dreadth']
    d = ship['draft']
    L = ship['Loa']
    Cb = ship['C_b']
    X_wave = (4 * rho_water * g * w_a / w_k**2 / math.sin(alpha_w) * (1 - math.exp(-w_k * d))
              * math.sin(w_k * L * math.cos(alpha_w) / 2) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.sin(w_omega * t))
    Y_wave = (4 * rho_water * g * w_a / w_k**2 / math.cos(alpha_w) * (1 - math.exp(-w_k * d))
              * math.sin(w_k * L * math.cos(alpha_w) / 2) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.sin(w_omega * t))
    N_wave = (-2 * rho_water * g * w_a / w_k * (1 - math.exp(-w_k * d)) * math.sin(w_k * B * math.sin(alpha_w) / 2) * math.cos(w_omega * t))

    return X_wave, Y_wave, N_wave

# 拖轮控制，或者说拖力计算
# 假设拖轮固定在船舶首部两侧，且与船舶艏向一定角度
def tug_force(power_force,power_angle):
    angle_tug = 45 # 相对船艏向的角度
    pf = power_force
    pa = power_angle
    t1 = math.sin(45 - pa) * pf
    t2 = math.cos(45 - pa) * pf
    X_tug = t1 + t2
    Y_tug = 
    N_tug = 
    return X_tug,Y_tug,N_tug


In [ ]:
# 控制模块
def goalstright():
    # 刚开始可以进行速度的加速
    if ship_v =0:
        # 刚开始加速时,拖轮提供最大拖力
        pass
    if ship_v < maxv:
        a = force_tug / ship_mass
        ship_v += a * dt 
        # 简化了，应该是对应的u,v,r都有
        # 但是这里只为了让船舶前进，所以没有r！
    elif ship_v > maxv:
        a = force_tug / ship_mass
        ship_v -= a * dt 
    else:
        # 这个时候的拖轮的力就等于环境干扰力的合力可以计算出拖力
        pass
    # 我还要输出拖轮力才能计算整个模型的运动，得到速度和位置
    # 所以一开始的拖轮力的大小得知道
    # 可在这个模块中进行位置的计算
    return force_tug,u,v


def turnheading():
    # 刚开始考虑最大角速度旋转
    a = N_T / I_zz
    if a > 0.1:
        a = 0.1
        r += a * dt
        N_T = a * I_zz
    return N_T,r


def parallel_berthing(maxpv=0.1):# 最大的平行靠泊速度0.1m/s
    # 此时两个拖轮共同将船舶平行推向岸边
    # 所以两个拖轮的作用位置已知，且作用力方向垂直于岸边
    # 在控制靠泊速度的前提下，可以得到拖轮的推力大小
    pass
    
    



In [ ]:
# 算是对船舶运动的前提判断

# 先确定输入值及其意义
totargetanlge = "东"# 确定船舶停泊后的艏向，即岸的方向和船是左舷靠泊还是右舷靠泊。看来可能要改成判断左右舷靠泊
# 假设出我需要的值
angle_ship = 0
ship_breathd = 18
initial_position =[0,0]
initial_v = [0,0]
target_posiotion = [234,456]
# 并计算出转向点的位置
turn_position = [0,0]
ttt = totargetanlge
berthing_angle = 0
ba = berthing_angle
B = ship_breathd

# 规定船舶的停泊艏向，即规定船舶在靠泊时是左舷靠泊还是右舷靠泊
# 以及判断船舶的转向点坐标
if ttt == "东":
    ba = 0 # 单位都是度
    if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
        # 在第一象限中时
        target_posiotion[0] -= (4 * B + 20) # 假设的安全平行靠泊距离
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在二
        target_posiotion[0] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
        # 在三
        target_posiotion[0] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在四
        target_posiotion[0] -= (4 * B + 20)
    else:
        print("不能判断转向点的位置")
elif ttt =="西":
    ba = 180
    if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
        # 在第一象限中时
        target_posiotion[0] -= (4 * B + 20) # 假设的安全平行靠泊距离
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在二
        target_posiotion[0] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
        # 在三
        target_posiotion[0] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在四
        target_posiotion[0] -= (4 * B + 20)
    else:
        print("不能判断转向点的位置")
elif ttt =="北":
    ba = 90
    if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
        # 在第一象限中时
        target_posiotion[1] -= (4 * B + 20) # 假设的安全平行靠泊距离
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在二
        target_posiotion[1] -= (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
        # 在三
        target_posiotion[1] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在四
        target_posiotion[1] += (4 * B + 20)
    else:
        print("不能判断转向点的位置")
elif ttt =="南":
    ba = -90
    if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
        # 在第一象限中时
        target_posiotion[1] -= (4 * B + 20) # 假设的安全平行靠泊距离
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在二
        target_posiotion[1] -= (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
        # 在三
        target_posiotion[1] += (4 * B + 20)
    elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
        # 在四
        target_posiotion[1] += (4 * B + 20)
else:
    print("不能决定船舶的靠泊艏向")

# 判断的变量

a0 = initial_position[0]
b0 = initial_position[1]
a1 = turn_position[0]
b1 = turn_position[1]
a = target_posiotion[0]
b = target_posiotion[1]
# Dit : D initial to turn 
Dit = ((a0-a1)**2 + (b0-b1)**2) **0.5
# Dtt : D turn to target 
Dtt = ((a-a1)**2 + (b-b1)**2) **0.5
#
angle_to_turn = (b1 - b0)/(a1 - a)
att = angle_to_turn
update_position =[]
update_v = []
# 船舶运动从初始位置运动到转向点的判断
while True:
    # 首先判断初始位置
    if math.tan(angle_ship) - att > 0.001:
        turnheading()
    elif math.tan(angle_ship) - att <= 0.001:
        goalstright()
    # 更新船舶的坐标位置
    update_position.append(position)
    # 更新船舶的速度
    update_v.append(u,v,r)
    # 得到最后一个坐标的位置
    a3 = update_position[-1][0]
    b3 = update_position[-1][1]
    # 计算最后的坐标位置到转向点的距离
    distance = ((a3-a1)+(b3-b1))**0.5
    if distance<=0.001:
        break

# 此时的船舶艏向就是船舶前进时的艏向
# 重新赋值给船的艏向
angle_ship = math.arctan(att)
turn_angle = angle_ship - ba
if ba == 0:#逆时针旋转
    turnheading()
    pass
elif ba == 180:
    # 顺时针旋转
    turnheading()
    pass
elif ba == 90:
    # 逆时针旋转
    turnheading()
    pass
else:# 顺时针旋转
    turnheading()
    pass

# 旋转过turn_angle之后与岸平行，进行平行靠泊
parallel_berthing()


# 仿真计算部分
# 使用船舶运动的mmg模型，但是将船体视为无桨无舵的船
# 使用龙格库塔进行求解
X_hull,Y_hull,N_hull = hull_force()
X_wind,Y_wind,N_wind = wind_force()
X_wave, Y_wave, N_wave = wave_force()
 = tug_forece()
X_total = X_hull + X_wind + X_wave  + X_tug
Y_total = Y_hull + Y_wind + Y_wave  + Y_tug
N_total = N_hull + N_wind + N_wave  + N_tug


# 可视化
# 输出船舶的轨迹图和船舶航向角变化的图


