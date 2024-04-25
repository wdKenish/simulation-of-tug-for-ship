import math
import numpy as np
from scipy.optimize import fsolve





def normalize22pi(degrees):
    "将角度归一化到[0, 2π)区间"
    radians = np.deg2rad(degrees)  # 将角度从度转换为弧度
    normalized_radians = radians % (2 * np.pi)
    return normalized_radians


# 转首运动求解
def turn_mode(state):
    initial_guess = [1, 0, 0]  # t=1, a=0度, b=0度

    # 求解方程组
    solution = fsolve(turn_equations, initial_guess, args=state)

    t_value = solution[0]

    # 弧度转换为度
    a_degrees = np.degrees(solution[1]) % 360
    b_degrees = np.degrees(solution[2]) % 360

    # 确保角度在 [0, 360) 范围内
    a_degrees = a_degrees if a_degrees >= 0 else a_degrees + 360
    b_degrees = b_degrees if b_degrees >= 0 else b_degrees + 360

    # 转为弧度制，后面计算需要
    a_degrees = normalize22pi(a_degrees)
    b_degrees = normalize22pi(b_degrees)
    return t_value, a_degrees, b_degrees


# 直行运动求解
def gol_mode(state):

    initial_guess = [1, 1, 0]  # t1=1, t2=1, a=0度

    # 求解方程组
    solution = fsolve(goline_equations, initial_guess, args=state)

    t1_value = solution[0]
    t2_value = solution[0]

    # 弧度转换为度
    a_degrees = np.degrees(solution[2]) % 360

    # 确保角度在 [0, 360) 范围内
    a_degrees = a_degrees if a_degrees >= 0 else a_degrees + 360

    a_degrees = normalize22pi(a_degrees)
    return t1_value, t2_value,a_degrees


# 靠泊运动求解
def pb_mode(state):

    initial_guess = [1, 0, 0]  # t=1, a=0度, b=0度

    # 求解方程组
    solution = fsolve(pberthing_equations, initial_guess, args=state)

    t_value = solution[0]

    # 弧度转换为度
    a_degrees = np.degrees(solution[1]) % 360
    b_degrees = np.degrees(solution[2]) % 360

    # 确保角度在 [0, 360) 范围内
    a_degrees = a_degrees if a_degrees >= 0 else a_degrees + 360
    b_degrees = b_degrees if b_degrees >= 0 else b_degrees + 360


    a_degrees = normalize22pi(a_degrees)
    b_degrees = normalize22pi(b_degrees)
    return t_value, a_degrees, b_degrees


# test
'''
state = 20,30,50,20,8,3
print(turn_mode(state),gol_mode(state),pb_mode(state))
'''



   


# 根据模式判断出拖轮在三个自由度的总分量
from judgement import ship_motions
from envforce import all_env_force
from params import params
from PIDcontrol import   out_a #  得到加速度
def out_Tugs(ship_state, turn_p, targ_p, ship, env_params, a_params, tolerance = 1):
    L = ship['Lpp']
    B = ship['breadth']
    x, y, n = all_env_force(ship_state, ship, env_params)
    i_zz = params(ship_state, ship)['I_zz']
    # alpha =     # 如何从PID从得到加速度
    m = params(ship_state, ship)['m']
    judge = ship_motions(ship_state, turn_p, targ_p, tolerance )
    print('judge of out_Tugs:',judge)
    if judge == 'turning': # 这也越高不一样
        _,_,_,_,alpha = out_a(targ_p, ship, a_params['a_turn'], a_params['v_turn'])
        # 得到加速度，但是这里是一个数组
        z = i_zz * alpha
        state = x, y, z, n, L, B
        t, a, b = turn_mode(state)
        print("t={0},a={1},b={2}".format(t,a,b))
        X = t * (np.cos(a) + np.cos(b)) 
        Y = t * (np.sin(a) + np.sin(b)) 
        N = t * 0.5 * (L * (np.sin(a) + np.sin(b)) + B * (np.cos(a) + np.cos(b))) 
        return X,Y,N
    elif judge == 'go_in_line':
        _,_,_,_,alpha = out_a(targ_p, ship, a_params['a_xline'], a_params['v_xline'])
        z = m * alpha
        state = x, y, z, n, L, B
        t1, t2, a = gol_mode(state)
        print("t1={0},t2={1},a={2}".format(t1, t2,a))
        X = (t1 + t2) * np.cos(a)
        Y = (t1 - t2) * np.sin(a)
        N = 0.5 *(( L * np.sin(a) * (t1 - t2)) + (B * np.cos(a) * (t1 - t2)))
        return X,Y,N
    elif judge == 'berthing': # 这里应该不一样
        _,_,_,_,alpha = out_a(targ_p, ship, a_params['a_yline'], a_params['v_yline'])
        z = m * alpha
        state = x, y, z, n, L, B
        t, a, b = pb_mode(state)
        print("t1={0},t2={1},a={2}".format(t1, t2,a))

        X = t * (math.cos(a) + math.cos(b)) + X
        Y = t * (math.sin(a) + math.sin(b)) + Y 
        N  = 0.5 * t * (L * (math.sin(a) + math.sin(b)) + B * (math.cos(a) + math.cos(b)))
        return X, Y, N
    elif judge == 'berthing':
        print('berthing complete') 
    else:
        print('The judgement of mode is failure from tugforces')


# test 
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
print(out_Tugs(state, turn_p, targ_p, ship_params, env_forces, a_params))
'''