
import numpy as np


def turn_position(target_posiotion,ship,totargetanlge = "东"):
    # 确定船舶停泊后的艏向，即岸的方向和船是左舷靠泊还是右舷靠泊。看来可能要改成判断左右舷靠泊   
    # 并计算出转向点的位置
    turn_position = [0,0]
    ttt = totargetanlge    
    B = ship['breadth']
    if ttt == "东" or ttt == '西': # 考虑船舶的停泊位置分别在四个象限内的情况
        # y是东西方向，正半轴为东
        turn_position[0] = target_posiotion[0]
        if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
            # 在第一象限中时
            turn_position[1] = target_posiotion[1] - (4 * B + 20) # 假设的安全平行靠泊距离
        elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
            # 在二
            turn_position[1] = target_posiotion[1] + (4 * B + 20)
        elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
            # 在三
            turn_position[1] = target_posiotion[1] + (4 * B + 20)
        elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
            # 在四
            turn_position[1] = target_posiotion[1] - (4 * B + 20)
        else:
            print("distance 23 不能判断转向点的位置")
    elif ttt =="北" or ttt =="南":
        turn_position[1] = target_posiotion[1]
        if target_posiotion[0] >= 0 and target_posiotion[1]>= 0:
            # 在第一象限中时
            turn_position[0] =target_posiotion[0] - (4 * B + 20) # 假设的安全平行靠泊距离
        elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
            # 在二
            turn_position[0] =target_posiotion[0] - (4 * B + 20)
        elif target_posiotion[0] < 0 and target_posiotion[1]< 0:
            # 在三
            turn_position[0] =target_posiotion[0] + (4 * B + 20)
        elif target_posiotion[0] < 0 and target_posiotion[1]>= 0:
            # 在四
            turn_position[0] =target_posiotion[0] + (4 * B + 20)
        else:
            print("distance 39 不能判断转向点的位置")
    else:
        print("distance 41 不能决定船舶的靠泊艏向")
    return turn_position


def ship_motions(state, turning_pos , target_pos , tolerance=5.0):  # 注意输入的角度需要进行归一化
    """
    Parameters:
        initial_pos (tuple): Initial position of the ship (x, y).
        turning_pos (tuple): Turning position of the ship (x, y).
        target_pos (tuple): Target/berthing position of the ship (x, y).
        current_pos (tuple): Current position of the ship (x, y).
        current_heading (float): Current heading angle of the ship in degrees.
        tolerance (float): Tolerance for considering the ship at a position (in the same units as positions).

    """

    def normalize_angle(degrees):
        "将角度归一化到[-π, π]区间"
        radians = np.deg2rad(degrees)  # 将角度从度转换为弧度
        normalized_radians = (radians + np.pi) % (2 * np.pi) - np.pi
        return normalized_radians
    
    def vector_angle(vec1, vec2):
        """Calculate the angle in degrees between two vectors."""
        unit_vec1 = vec1 / np.linalg.norm(vec1)
        unit_vec2 = vec2 / np.linalg.norm(vec2)
        dot_product = np.dot(unit_vec1, unit_vec2)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0)) / np.pi * 180
        return angle
    
    initial_pos = (0,0) 
    current_pos = (state[0],state[1])
    current_heading = state[2]  # 假设 state[2] 是角度，单位为度
    current_heading = normalize_angle(current_heading)
    print('船舶当前航向角:',current_heading)


    # Convert positions to numpy arrays for vector calculations
    initial_pos = np.array(initial_pos)
    turning_pos = np.array(turning_pos)
    target_pos = np.array(target_pos)
    current_pos = np.array(current_pos)
    
    # Calculate vectors and distances
    vector_to_turning = turning_pos - initial_pos
    vector_to_target_from_turning = target_pos - turning_pos
    current_to_turning_vector = turning_pos - current_pos
    current_to_target_vector = target_pos - current_pos
    
    # Calculate angles
    heading_vector = np.array([np.sin(current_heading), np.cos(current_heading)])  # 需要交换？注意坐标系
    print('船舶航向矢量 of judgement:',heading_vector)
    print('船舶转向矢量 of judgement:',vector_to_turning)
    angle_to_turning = vector_angle(heading_vector, vector_to_turning)
    print('计算转角to转向点 of judgement:',angle_to_turning)
    angle_to_target = vector_angle(heading_vector, vector_to_target_from_turning)
    print('靠泊转角 of judgement:',angle_to_target)
    
    # Distance checks with tolerance
    distance_to_turning = np.linalg.norm(current_to_turning_vector)
    distance_to_target = np.linalg.norm(current_to_target_vector)
    print('到转向点的距离 of judgement:',distance_to_turning)
    print('到靠泊点的距离 of judge ment:',distance_to_target)
    # Check positions and headings

    print('判断1 of judgement',np.array_equal(current_pos, initial_pos))
    print('判断4 ',np.linalg.norm(turning_pos - initial_pos))
    print('判断5',np.linalg.norm(target_pos - turning_pos))
    if np.array_equal(current_pos, initial_pos):  # 船舶在初始点的情况
        if angle_to_turning <= 0.001:
            return 'go_in_line'
        else:
            return 'turning'
    elif distance_to_turning <= tolerance:
        if np.linalg.norm(current_to_target_vector) <= tolerance: # 靠泊完成的判断
            return 'berthing_complete'
        elif abs(angle_to_target - 90)<=0.01:    # 这里有个使用向量积来判断
            return 'berthing'
        else:                                     # 其余进行转向
            return 'turning'
    elif distance_to_target <= tolerance:
        return 'berthing_complete'
    elif np.linalg.norm(turning_pos - initial_pos) >= distance_to_turning > tolerance:
        return 'go_in_line'
    elif np.linalg.norm(target_pos - turning_pos) >= distance_to_turning > tolerance:
        return 'berthing'
    else:
        return 'unknown'

# test
'''
state = [300.09,300.09, 89.999]
target_position = [300,400]
turn_position = [300,300]
result = ship_motions(state,turn_position,  target_position, tolerance=1)
print(result)
'''





'''
如果当前在初始点
    若航向指向转向点则前进
    否则旋转
如果当前在转向点
    若航向平行于岸边则平行靠泊
    否则旋转
如果
'''