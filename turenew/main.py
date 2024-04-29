import numpy as np
from params import calculate
from envforce import all_env_force
from tool import dof2p,turn_pos,kof2p,k2degrees
from PID import control_foreces


'''输入'''

# 输入船舶参数
ship_params = {
        'disp': 55126000.0,  # kg
    'Lpp': 181.6,  # Length between perpendiculars (meters)
    'width': 30.5,  # Width of the ship (meters)
    'breadth': 30.5,  # Breadth of the ship (meters)
    'stem_draft': 11.82,  # Draft at the stem (meters)
    'stern_draft': 11.82,  # Draft at the stern (meters)
    'draft': 11.82,  # General draft (meters)
    'Draft': 22.7,  # Assumed from calculation
    'x_0': 0,  # Initial x position
    'C_b': 0.821,  # Block coefficient
    'A_x': 1735.0,  # Frontal area (square meters)
    'A_y': 394.0  # Lateral area (square meters)
}

# 输入船舶初始状态
initial_state = np.array([0, 0, 0, 0, 0, 0, 0])

# 输入环境参数, 风速和风向， 波浪波长、波浪方向角和波幅， 流暂定
env_params = {'wind':[0, 0],'wave':[100, 0, 1.5],'current':[]}

target_position =(300,400)   # 使用元组


pid_parameters = [
    (0.1, 0.01, 0.05),  # PID for heading
    (0.2, 0.02, 0.1),   # PID for position
    (0.1, 0.01, 0.02)   # PID for speed
    ]

# 程序间隔
dt = 0.1
'''计算'''

# 计算船舶水动力导数并添加到船字典中
# 简化字典长度为p
p = calculate(ship_params)

# 得到靠泊前的转向点，即平行靠泊的开始点
turn_position = turn_pos(target_position, p)

# mmg方程的一部分参数
m_x = p['m_x_dash'] * (0.5 * p['rho'] * (p['Lpp']**2) * p['d'])
m_y = p['m_y_dash'] * (0.5 * p['rho'] * (p['Lpp']**2) * p['d'])
J_zz = p['J_z_dash'] * (0.5 * p['rho'] * (p['Lpp']**4) * p['d'])
m = p['m']
I_zz = m * (0.25 * p['Lpp'])**2



'''主模块'''
state = initial_state
x, y, psi = state[0], state[1], state[2]

p0 = (x, y)

update_state =[]
# 可能需要添加初始状态进入update_state
update_state.append(state)
if p0 == (0, 0):
    while True:# 初始转向循环
        # function of turning
        newphi = np.rad2deg(update_state[-1][2])
        newp0 = (update_state[-1][0], update_state[-1][1])
        angle2turn = k2degrees(kof2p(newp0, turn_position))
        initial_position = p0
        innital_velocity = (update_state[-1][3], update_state[-1][4])
        target_position = p0
        target_heading = angle2turn
        tugs = control_foreces(p0, psi, pid_parameters,)
        newstate= shipdynamic()
        
        update_state.append(newstate)

        if newp0 - angle2turn <= 0.1:
            print('初始航行已经调整完成')
            print('初始转向结束时字典的长度：',len(update_state))  # 计算每个拖轮力，判断拖轮模式
            break
    # 这里可能需要进行更新船舶的位置
    while True:# 直线航行循环
        new_state = update_state[-1]
        #functoin of go alstraight
        update_state.append()
        newp0 = (update_state[-1][0], update_state[-1][1])
        distance = dof2p(newp0, turn_position)
        if distance <= 0.1:
            print('到达平行靠泊起点')
            print('直线航行结束时字典的长度：',len(update_state))
            break

    while True:# 再转向判断循环
        new_state = update_state[-1]
        # function of turning 
        update_state.append()
        newphi = np.rad2deg(update_state[-1][2])
        if abs(newphi - np.pi/2) <=0.05:
            print('靠泊前转首完成')
            print('再转向结束时字典的长度：',len(update_state))
            break

    while True:# 平行靠泊循环
        new_state = update_state[-1]
        # function of parallel berthing 
        update_state.append()
        newp0 = (update_state[-1][0], update_state[-1][1])
        distance = dof2p(newp0, target_position)
        if distance <= 0.1:
            print('靠泊完成')
            print('靠泊结束时字典的长度：',len(update_state))
            break

else:
    print('不在假设初始点位置还没有添加，可以采用相对坐标位置')


