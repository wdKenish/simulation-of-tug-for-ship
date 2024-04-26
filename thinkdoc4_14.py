# 2023.4.14
# 更新一个整体实现
# 整体来说都差详细的代码
# 也有缺少验证和修改的部分，其中拖轮力的修改和计算
# mmg方程求解还是不知道如何迭代来记录每个时刻的速度和位置


# 多拖轮辅助滚装船靠泊的仿真
# 使用的滚装船船舶参数：
# Extracting ship parameters for PCC (Pure Car Carrier) from the provided data
# These values are obtained from the images provided by the user

ship_params = {
    'L_pp': 180.0,    # Length between perpendiculars (m)
    'B': 30.0,        # Beam (m)
    'd': 0.1367,      # Draft (m)
    'D_p': 0.095,     # Propeller diameter (m)
    'C_b': 0.611,     # Block coefficient
    'x_g': -0.0422,   # Longitudinal center of gravity (-)
    'A_R': 0.0333     # Rudder area ratio (A_R / L_pp * d)
}

# Hull derivatives for PCC extracted from the table
hull_derivatives = {
    'X_vv': -0.00454,
    'X_vr': -0.1232,
    'X_rr': 0.5917,
    'Y_v': 0.2629,
    'Y_r': -0.566,
    'Y_vvvv': 1.5504,
    'Y_vrr': 0.7384,
    'Y_rrr': -0.033,
    'N_v': 0.0977,
    'N_r': -0.457,
    'N_vvv': -0.6273,
    'N_vvr': 0.0954,
    'N_rrr': -0.0533
}

# Propeller and rudder coefficients
propeller_rudder_coeffs = {
    'epsilon': 1.214,  # Wake fraction coefficient
    't_R': 0.391,     # Thrust deduction fraction at the rudder
    'w_0': 0.753,     # Wake fraction at ship speed = 0
    't_P': 0.095,     # Thrust deduction fraction for propeller
    'k': 0.466        # Propeller open water efficiency factor
}

# Assume initial conditions for simulation
initial_conditions = {
    "u": 0,      # Surge velocity
    "v": 0,      # Sway velocity
    "r": 0,      # Yaw rate
    "delta": 0,  # Rudder angle
    "n": 0,      # Propeller speed
    "wind_speed": 0, # Wind speed
    "wind_angle": 0  # Wind angle relative to ship heading
}


使用的拖轮的参数：
Length (m): 32
Width (m): 29
Side height (m): 9.8
Draft (m): 4.3
Full load draft (m): 3.98
Light draft (m): 3.58
Total mass (t): 519.6
Full load displacement (m²): 81.34
Light displacement (m²): 176.5
Bollard pull (tons): 4.5
Propeller diameter (m): 2.35
Number of propellers: 2
Propeller rotational speed (revolutions): 0.65
Maximum speed (knots): 4


'''
整个船舶的运动只考虑二维坐标系中三个维度的运动即：
纵荡：surge，对应的力和力矩使用X来表示，线速度和角速度使用u来表示
横摇：sway，对应的力和力矩使用Y来表示，线速度和角速度使用v来表示
艏摇：yaw对应的力和力矩使用N来表示，线速度和角速度使用r来表示
定义坐标系：
1.固定坐标系（大地参考系XOY）：
正东为X轴，正北为Y轴，通常原点坐标O与船舶在t=0时刻的重心位置一致
2.随船坐标系（xoy）：
船的首向（沿船中指向船艏）为x轴，右舷方向（垂直于肿纵剖面指向右舷）为y轴，原点o在船舶重心处
'''
# 在此坐标系的前提下有mmg方程：
(m + m_x) * u1 - (m + m_y) * r * v = X_total
(m + m_y) * v1 + (m + m_x) * r * u = Y_total
(I_zz + J_zz) * r1 = N_total
'''
where:m是船舶的质量，m_x和m_y分别是船舶在x方向和y方向的附加质量
I_zz是船舶绕z轴的转动惯量，J_zz是船舶的附加转动惯量
u,v,r分别是船舶的x，y，z方向的线速度和角速度
u1,v1,r1分别是对应的加速度
X_total,Y_total,N_total分别是船舶在对应方向上的合力和和力矩
'''
# 而对于X_total,Y_total,N_total：
X_total = X_hull + X_rudder + X_propeller + X_wind + X_wave + X_current + X_tug
Y_total = Y_hull + Y_rudder + Y_wind + Y_wave + Y_current + Y_tug
N_total = N_hull + N_rudder + N_wind + N_wave + N_current + N_tug

X_H,Y_H,N_H = X_hull,Y_hull,N_hull
# Function to calculate hull forces based on derivatives and ship's speed and rudder angle
def calculate_hull_forces(V, delta_r, params, derivatives):
    """
    Calculate hull forces using the MMG model equations.
    
    V: Ship velocity (m/s)
    delta_r: Rudder angle (rad)
    params: Ship parameters
    derivatives: Hull derivatives
    """
    # Unpack required parameters
    L_pp = params['L_pp']
    d = params['d']
    x_g = params['x_g']
    A_R = params['A_R']

    # Calculate non-dimensional rudder angle and velocity terms
    r = V / L_pp  # Non-dimensional velocity
    beta = np.arcsin(V / (V**2 + r**2)**0.5)  # Drift angle (approximated as arcsin(v/U))
    beta_r = delta_r - beta  # Non-dimensional rudder angle

    # Non-dimensional velocity components for surge and sway
    v = V * np.sin(beta)
    u = V * np.cos(beta)

    # Using the hull derivatives to calculate forces
    X = (derivatives['X_vv']*v**2 + derivatives['X_vr']*v*r + derivatives['X_rr']*r**2) * 0.5 * params['rho'] * (L_pp * d)**2
    Y = (derivatives['Y_v']*v + derivatives['Y_r']*r + derivatives['Y_vvvv']*v**4 + derivatives['Y_vrr']*v*r**2 + derivatives['Y_rrr']*r**3) * 0.5 * params['rho'] * (L_pp * d)**2 * V
    N = (derivatives['N_v']*v + derivatives['N_r']*r + derivatives['N_vvv']*v**3 + derivatives['N_vvr']*v**2*r + derivatives['N_rrr']*r**3) * 0.5 * params['rho'] * (L_pp * d)**2 * L_pp * V

    return X, Y, N

# Sample inputs for velocity and rudder angle
V = 10  # Ship velocity in m/s
delta_r = 0.1  # Rudder angle in radians
rho_water = 1000  # Density of water in kg/m^3

# Add water density to ship parameters
ship_params['rho'] = rho_water

# Calculate the hull forces
X_hull, Y_hull, N_hull = calculate_hull_forces(V, delta_r, ship_params, hull_derivatives)
X_hull, Y_hull, N_hull

where:





X_rudder = 
Y_rudder =
N_rudder =


#虽然被拖船的螺旋桨不用考虑了，但是拖船在计算拖力和转速的时候需要进行计算
X_p = X_propeller
X_p = (1 - t) * rho * K_T * D_p ** 4 * n ** 2
'''
where:
t是螺旋桨的推理减额系数
rho是水的密度
K_T是螺旋桨的推力系数
D_p是螺旋桨的直径
n是螺旋桨的转速
'''
# 还是考虑无浆和舵的情况吧！
# 而对于X_total,Y_total,N_total：
X_total = X_hull + X_wind + X_wave + X_current + X_tug
Y_total = Y_hull + Y_wind + Y_wave + Y_current + Y_tug
N_total = N_hull + N_wind + N_wave + N_current + N_tug
'''
那现在的问题就是如何计算wind、wave、current、tug
难点是tug，要如何考虑

对于风浪流的计算，首先要确定他们在坐标系中的描述
对于风，使用风速和风向来进行描述（使用ship to ship的风力计算模型）
如果使用参考文献中的计算公式的话，挺麻烦
改用行业标准来进行计算（CB/Z815-2019）
船舶相对风速与船艏夹角为风向角angle_w，定义风向角=0度时顶风，90度时为左舷正横风，180度时为顺风
相对风速是风的绝对速度和船舶航行速度的矢量和
'''
# 滚装船Cwx=0.5，Cwy=1.0,
def wind(angle_w,rho_wind,Ax,Ay,L,U_w,Cwx,Cwy):
    # 风的干扰力模型
    x_A = 0
    # 假设重心和中心重合，所以x_A = 0
    fxw = (0.15 * math.cos(5 * angle_w) - math.cos(angle_w)) / 0.85
    fyw = (math.sin(angle_w) - 0.07 * math.sin(5 * angle_w)) / 0.93
    x_NW = x_A + 0.25 * L * (1 - angle_w / 90)
    # x_A是横向投影面积形心相对于原点的纵向坐标
    X_wind=0.5 * rho_wind * Ax * U_w ** 2 * Cwx * fxw
    Y_wind=0.5 * rho_wind * Ay * U_w **2 * Cwy * fyw
    N_wind=Y_wind * x_NW
    """
    alpha_w= #风向
    rho_wind= #空气密度
    Ax= #船舶水线面以上的正投影面积
    Ay= #……侧投影
    L= #船长
    U_w= #风速
    """
    return X_wind,Y_wind,N_wind


# 波浪的计算就使用周宏伟的二阶波浪计算公式
def wave(L,a,Lambda,angle_wave,angle_ship,rho=rho_water):
    # 船长，波的平均幅值，波浪的方向，艏向，水的密度
    Lambda = angle_wave + angle_ship
    C_xw =  0.05 - 0.2 * (Lambda / L) + 0.75 * (Lambda / L) **2 - 0.51 * (Lambda / L) ** 3
    C_yw =  0.46 + 6.83 * (Lambda / L) - 15.65 * (Lambda / L) **2 + 8.44 * (Lambda / L) ** 3
    C_nw = -0.11 + 0.68 * (Lambda / L) - 0.79 * (Lambda / L) **2 + 0.21 * (Lambda / L) ** 3
    X_wave = 0.5 * rho * L * a **2 * math.cos(angle) * C_xw 
    Y_wave = 0.5 * rho * L * a **2 * math.sin(angle) * C_yw
    N_wave = 0.5 * rho * L **2 * a **2 * math.sin(angle) * C_nw
    """
    目前该方法缺乏实验验证
    """
    return X_wave,Y_wave,N_wave

# 流的计算（from ship to ship）
def current(u_b = u,v_b = v,U_c,angle_cur,angle_ship):
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

'''
以上可以完成一个无浆舵的船的mmg模型
接下来是计算拖轮的推力
首先要确定拖轮所在的位置，假设拖轮已经就位到开始时刻
即步进行拖轮的分配和移动的考虑
所以就只考虑拖轮的的作用点和方向
但是我的拖轮的力不是直接给出的，而是通过计算所得
首先给出总的拖轮力的大小和方向
简化为两个拖轮
一个在船首，一个在船尾

再者就是要使用路径的规划来进行船舶移动
通过船舶的初始状态获取到船舶的初始位置
输入停泊目标位置
理论上两个点直线最短
但是实际状况中，不一定是走直线
第一种方案：
给出经过的路径关键点，使用拟合曲线来实现规划路径，让船舶尽可能的贴着曲线进行移动
但是靠泊过程不仅仅是船舶的移动

第二种方案（目前的非最优解，但最简单实现）：
通过限制来确定船舶的平行靠泊初始位置称为转向点，这样包括初始位置和目标位置就有三个点
而船舶的运动可以分解为平动和转动
就可以简单分解为三个过程：
首先判断航向是否指向转向点
若指向转向点，则让船舶前进
若不则进行转首运动，直到指向转向点
在这两个过程中，当船舶进行直线航行的时候，要求到达转向点的时候速度恰好等于或者约等于0
当船舶进行转首运动时，在达到指定的角度的时候旋转角速度叶恰好为零
在到达转向点后，进行船舶的航向是否平行于岸边
若平行岸边则进行靠泊操作
若不则进行船舶转首运动，直到于岸边平行后进行平行靠泊
'''
# 使用模糊控制策略来实现控制拖轮的
# 导入模糊控制库
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# 定义模糊控制的输入和输出变量
distance = ctrl.Antecedent(np.arange(0, 100, 1), 'distance')
speed = ctrl.Antecedent(np.arange(0, 10, 1), 'speed')
thrust = ctrl.Consequent(np.arange(0, 100, 1), 'thrust')

# 定义模糊集合和隶属度函数
distance['close'] = fuzz.trimf(distance.universe, [0, 0, 50])
distance['far'] = fuzz.trimf(distance.universe, [0, 50, 100])

speed['slow'] = fuzz.trimf(speed.universe, [0, 0, 5])
speed['fast'] = fuzz.trimf(speed.universe, [0, 5, 10])

thrust['low'] = fuzz.trimf(thrust.universe, [0, 0, 50])
thrust['high'] = fuzz.trimf(thrust.universe, [0, 50, 100])

# 定义模糊规则
rule1 = ctrl.Rule(distance['close'] & speed['slow'], thrust['high'])
rule2 = ctrl.Rule(distance['far'] & speed['fast'], thrust['low'])

# 创建模糊控制系统
thrust_ctrl = ctrl.ControlSystem([rule1, rule2])
thrusting = ctrl.ControlSystemSimulation(thrust_ctrl)

# 输入模糊控制系统的距离和速度
thrusting.input['distance'] = 30
thrusting.input['speed'] = 3

# 运行模糊控制系统
thrusting.compute()

# 输出控制结果
print(thrusting.output['thrust'])
'''
所以接下来需要进行计算的就是船舶的转向点的确定
首先是如何描述船舶的目标靠泊点，使用一个位置点和艏向的方向来描述
在坐标系中就可以再得到两个点，或者说是一条线，就可以描述船舶的目标靠泊状态
而转向点一般设置在距离靠泊点4倍船宽，而靠泊点和转向点所在直线垂直于靠泊直线，即岸边
因为下一步就是进行平行靠泊。

模糊控制：
定义艏向
angle_ship 与X0轴的夹角（随船坐标系x轴与固定坐标系X0轴的夹角）
定义靠泊艏向，即船靠岸后的船艏的朝向方向
正东方向即与x轴正半轴方向一致，正北方向则与y轴正半轴方向一致，其他方向以此类推
但是！我想还是固定有一个选择会更简单一些：
对于平行于x轴的：朝向x轴正半轴就是0度，朝向负半轴就是180度

对于平行于y轴的：朝向y轴正半轴就是90度，朝向负半轴时就是-90度
'''
# 定义三个关键点
inip:initial position
inip = (a0,b0)
turp:turn positoin
turp = (a1,b1)
tarp:target position
tarp = (a,b)
# 定义几个变量来描述船舶与转向点和目标位置的距离
Dit : D initial to turn 
Dit = ((a0-a1)**2 + (b0-b1)**2) **0.5
Dtt : D turn to target 
Dtt = ((a-a1)**2 + (b-b1)**2) **0.5
#第一次进行判断：
#判断艏向是否指向转向点
#即：
if math.tan(angle_ship) == (turp[1]-inip[1])/(turp[0]-inip[0]):
    # 前进
else:# 转向
   #  转向后再前进
#到达转向点后进行第二次判断：
if 




def goalstright():
    D_i_to_turn 

#在这个过程中
#需要可视化的有：船舶的位置（轨迹图）、拖力的大小和方向（在随船坐标系上）


import math
# 拖轮的拖力模块
def tug():
    X_T = F_T * math.cos(theta)
    Y_T = F_T * math.sin(theta)
    N_T = Y_T * 0.5 * L # 这是一个拖轮的时候

# 模糊控制
# 先定义前进模块
maxv = 5   # 最大前进速度
maxr = 0.1 # 最大角速度
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












'''
简化船舶在的控制点
除了船舶的中心点O(x0,y0)之外
增加船首控制点A(x0+L/2*cos(phi),y0+L/2*sin(phi))，位于船舶的首部中间
L=100
B=30
phi是船舶艏向,是与x轴（正北方向夹角）
初始速度：u,v,r = u0.v0,r0
目标位置O2(x,y),A2(x+L/2,y),x和y是靠泊位置
目标状态u,v,r=0,0,0
中间位置和状态O1(x,y-4*B-20),A1(x+L/2,y-4*B-20)
u,v,r=0,0,0
考虑看如何来实现船舶的运动控制过程



'''