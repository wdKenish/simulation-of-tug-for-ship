
from envforce import all_env_force
# 得到船舶在运动过程中的update_state列表
# 得到拖轮的总力的tugs_fores列表

# 通过船舶的状态来计算环境的总阻力
# 而真正的拖轮力就是拖轮力加上环境总阻力
def get_T(p, update_state, tug_forces, env_params):
    Tug = []
    for i in range(len(update_state)):
        Xe, Ye, Ne = all_env_force(update_state[i], p, env_params)
        t = tug_forces
        xt, yt, nt = t[i][0] ,t[i][1], t[i][2]

        XT, YT, NT = -Xe+xt, -Ye+yt, Ne+nt
        
        Tug.append([XT, YT, NT])
    return Tug


def distribute_tug():
    # 依据输出的四个阶段的字典长度，最好再有一个判断的字符，进入到对应的力的求解
   def turn():
      return
   def