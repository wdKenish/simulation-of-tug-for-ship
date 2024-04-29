import numpy as np

def safe_divide(numerator, denominator):
    # 确保除法时没有分母为零
    return numerator / (denominator if denominator != 0 else 1e-10)


def turn_pos(target_position, p, totargetangle="东"):# 东是岸的方向
    # 确定船舶停泊后的艏向，并计算出转向点的位置
    turn_position = [0, 0]
    breadth = p['breadth']
    safety_distance = 4 * breadth + 20  # 定义安全距离

    if totargetangle in ["东", "西"]:  # 考虑东西方向
        turn_position[0] = target_position[0]
        turn_position[1] = target_position[1] + (safety_distance if target_position[0] < 0 else -safety_distance)
    elif totargetangle in ["北", "南"]:  # 考虑南北方向
        turn_position[1] = target_position[1]
        turn_position[0] = target_position[0] + (safety_distance if target_position[1] < 0 else -safety_distance)
    else:
        raise ValueError("无法决定船舶的靠泊艏向")

    return (turn_position[0],turn_position[1])


def dof2p(p1,p2):
    # 计算两点之间的距离
    return np.linalg.norm(np.array(p1)-np.array(p2))


def aof2l(k1,k2):
    # 计算两线之间的夹角
    angle = np.arctan(k1 - k2) / (1 + k1 * k2)
    return np.degrees(angle)


def k2degrees(k):
    # 计算直线对应的角度
    angle = np.arctan(k)
    return np.degrees(angle)

def kof2p(p1,p2):
    dy = p2[1] - p1[1]
    dx = p2[0] - p1[0]
    if dx == 0:
        # 直接返回角度
        return np.degrees(np.pi/2)
    else:
        return k2degrees(dy / dx)