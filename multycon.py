这份文件内容涉及描述船只三自由度（3-DOF）的运动模型，其中包括动力学和运动学方程。下面是对方程的简要分析和思考如何用代码实现这些方程的方法：

### 方程解析
1. **位置和速度关系**:
   \[
   \dot{\eta}(t) = R(\psi(t))v(t)
   \]
   其中，\(\eta(t) = [x(t), y(t), \psi(t)]^T\) 为世界坐标系下的位置和航向，\(v(t) = [u(t), v(t), r(t)]^T\) 是船体坐标系下的速度向量（纵向、横向和转向速度），\(R(\psi(t))\) 是从船体坐标系到世界坐标系的旋转矩阵。

2. **动力学方程**:
   \[
   M\dot{v}(t) + C(v(t))v(t) + Dv(t) = \tau(t) + \tau_e(t)
   \]
   - \(M\) 是质量矩阵，包括刚体质量和附加质量。
   - \(C(v(t))\) 是科里奥利-离心力矩阵。
   - \(D\) 是阻尼矩阵，通常只考虑线性部分。
   - \(\tau(t)\) 是可控输入，即通过拖船施加的力。
   - \(\tau_e(t)\) 是环境扰动力。

3. **可控输入的计算**:
   \[
   \tau_s(t) = \sum_{i=1}^{n} [L_{S_i}(x_{i},y_{i})B_{S_i}(\alpha_{i}(t))F_{i}(t)]
   \]
   每个拖船的力通过位置、方向和力的大小计算得出。

### 代码实现思路
- **定义旋转矩阵** \(R(\psi(t))\) 函数，接受航向角作为输入，返回3x3矩阵。
- **模拟船体动力学** 使用给定的\(M, C, D, \tau, \tau_e\) 来数值求解动力学方程。
- **计算可控输入** \(\tau_s(t)\)，可能需要建立函数来计算每个拖船对主船的作用力。
- **数值积分** 使用诸如Euler方法或Runge-Kutta方法来模拟船只的位置和速度随时间的变化。

代码实现时，我们可以使用`numpy`库来处理矩阵运算，使用`scipy.integrate`的ODE求解器来进行数值积分。接下来，我将编写一个初步的框架代码，并详细讨论其实现。请稍等。



import numpy as np

def rotation_matrix(psi):
    """
    Compute the rotation matrix from the body frame to the world frame.
    
    Parameters:
    psi : float
        Heading angle in radians.
    
    Returns:
    numpy.ndarray
        The 3x3 rotation matrix R.
    """
    R = np.array([[np.cos(psi), -np.sin(psi), 0],
                  [np.sin(psi), np.cos(psi), 0],
                  [0, 0, 1]])
    return R

# Example usage with a heading angle of 45 degrees (converted to radians)
# rotation_matrix(np.deg2rad(45))

# Uncomment the function call below to see the result when you are running this locally or in an environment where execution is permitted.
# rotation_matrix(np.deg2rad(45))


def coriolis_matrix(C, v):
    """
    Compute the Coriolis matrix for given velocity vector.
    
    Parameters:
    C : numpy.ndarray
        The 3x3 Coriolis matrix, dependent on the velocity vector.
    v : numpy.ndarray
        The velocity vector [u, v, r].
    
    Returns:
    numpy.ndarray
        The Coriolis force matrix multiplied by the velocity vector.
    """
    return C @ v

def ship_dynamics(t, eta, v, M, C, D, tau, tau_e):
    """
    Calculate the derivative of the velocity and position for the ship based on the dynamics equation.
    
    Parameters:
    t : float
        The current time.
    eta : numpy.ndarray
        The current position and heading vector [x, y, psi].
    v : numpy.ndarray
        The current velocity vector [u, v, r] in the body-fixed frame.
    M : numpy.ndarray
        The mass matrix.
    C : numpy.ndarray
        The Coriolis matrix, functionally dependent on velocity.
    D : numpy.ndarray
        The damping matrix.
    tau : numpy.ndarray
        The control input vector [tau_u, tau_v, tau_r].
    tau_e : numpy.ndarray
        The environmental disturbance vector [tau_u_e, tau_v_e, tau_r_e].
    
    Returns:
    tuple
        The derivative of position and velocity as a tuple (d_eta, d_v).
    """
    psi = eta[2]
    R_psi = rotation_matrix(psi)
    d_eta = R_psi @ v  # Transform velocity to global frame
    inv_M = np.linalg.inv(M)
    coriolis_force = coriolis_matrix(C, v)
    d_v = inv_M @ (tau + tau_e - coriolis_force - D @ v)
    return d_eta, d_v

# Example matrices and vectors (these would be defined based on the specific ship characteristics)
# M = np.eye(3)  # Simplified mass matrix
# C = np.zeros((3, 3))  # Simplified Coriolis matrix for demonstration
# D = np.eye(3) * 0.1  # Simple linear damping
# tau = np.array([1, 0, 0])  # Example control input
# tau_e = np.array([0.1, 0.1, 0.1])  # Example environmental disturbance

# Uncomment the function call below to see the result when you are running this locally or in an environment where execution is permitted.
# ship_dynamics(0, np.array([0, 0, 0]), np.array([1, 0, 0]), M, C, D, tau, tau_e)


def towline_force(L, B, F):
    """
    Calculate the force vector from a single towline.
    
    Parameters:
    L : numpy.ndarray
        The lever arm matrix for the towline in the body-fixed frame.
    B : numpy.ndarray
        The direction matrix for the towline force.
    F : float
        The magnitude of the force applied by the towline.
    
    Returns:
    numpy.ndarray
        The force vector produced by the towline.
    """
    return L @ B @ np.array([[F], [0], [0]])

def total_towline_force(towlines):
    """
    Calculate the total towline force vector as the sum of individual towline forces.
    
    Parameters:
    towlines : list of tuples
        Each tuple contains (L, B, F) for each towline.
    
    Returns:
    numpy.ndarray
        The total force vector from all towlines.
    """
    total_force = np.zeros((3, 1))
    for L, B, F in towlines:
        total_force += towline_force(L, B, F)
    return total_force

# Example usage with dummy data for towlines
# towlines = [
#     (np.eye(3), rotation_matrix(np.deg2rad(30)), 1000),
#     (np.eye(3), rotation_matrix(np.deg2rad(-30)), 800)
# ]

# Uncomment the function call below to see the result when you are running this locally or in an environment where execution is permitted.
# total_towline_force(towlines)

from scipy.integrate import solve_ivp

def ship_motion_equations(t, state, M, C, D, towlines, tau_e):
    """
    Formulate the equations of motion for the ship to be used with an ODE solver.
    
    Parameters:
    t : float
        Current time.
    state : numpy.ndarray
        Current state of the system, combining position [x, y, psi] and velocity [u, v, r] into a single array.
    M : numpy.ndarray
        The mass matrix.
    C : numpy.ndarray
        The Coriolis matrix, here assumed to be static for simplification.
    D : numpy.ndarray
        The damping matrix.
    towlines : list of tuples
        Each tuple contains (L, B, F) for each towline.
    tau_e : numpy.ndarray
        Environmental disturbance forces and moments.

    Returns:
    numpy.ndarray
        The derivatives of the state vector for the ODE solver.
    """
    # Split state into position and velocity
    eta = state[:3]
    v = state[3:]

    # Compute total control force from towlines
    tau_s = total_towline_force(towlines).flatten()

    # Calculate dynamics
    d_eta, d_v = ship_dynamics(t, eta, v, M, C, D, tau_s, tau_e)

    # Combine derivatives into a single state derivative vector
    return np.concatenate((d_eta, d_v))

def simulate_ship_movement(initial_state, t_span, M, C, D, towlines, tau_e, method='RK45'):
    """
    Simulate the ship's movement over time using numerical integration.
    
    Parameters:
    initial_state : numpy.ndarray
        The initial state of the ship combining initial position and velocity.
    t_span : tuple
        The time span for the simulation (start, end).
    M : numpy.ndarray
        The mass matrix.
    C : numpy.ndarray
        The Coriolis matrix.
    D : numpy.ndarray
        The damping matrix.
    towlines : list of tuples
        Each tuple contains (L, B, F) for each towline.
    tau_e : numpy.ndarray
        Environmental disturbance forces and moments.
    method : str
        The integration method to use, default is 'RK45'.

    Returns:
    OdeResult
        The result of the numerical integration containing the times and states.
    """
    return solve_ivp(fun=lambda t, y: ship_motion_equations(t, y, M, C, D, towlines, tau_e),
                     t_span=t_span,
                     y0=initial_state,
                     method=method)

# Example matrices and vectors have been defined in previous steps, including example calls.
# Uncomment those sections and calls in a local Python environment or in an environment where execution is permitted for simulation.



import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 这里假设所有的自定义函数都已经定义

# 质量矩阵，阻尼矩阵等参数
M = np.eye(3) * 1000
C = np.zeros((3, 3))
D = np.eye(3) * 0.1
towlines = [
    (np.eye(3), rotation_matrix(np.deg2rad(30)), 1000),
    (np.eye(3), rotation_matrix(np.deg2rad(-30)), 800)
]
tau_e = np.array([0.1, 0.1, 0.1])
initial_state = np.array([0, 0, 0, 1, 0, 0])
t_span = (0, 100)

# 执行模拟
result = simulate_ship_movement(initial_state, t_span, M, C, D, towlines, tau_e)

# 可视化结果
plt.figure(figsize=(8, 6))
plt.plot(result.y[0], result.y[1], label='Ship Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Ship Movement Over Time')
plt.legend()
plt.grid(True)
plt.show()




这张图片提供了关于拖船和船舶的物理动力学模型更详细的数学描述，以及环境干扰因素的考量。我将解释其关键方程和概念，并讨论如何在代码中实现它们。

### 关键概念和方程

1. **拖船力矩阵和方向矩阵 (L_Ti, B_Ti)**:
   - \( L_{Ti} \) 是拖船相对于船体的力矩阵，考虑到拖船相对于船体的位置。
   - \( B_{Ti} \) 是拖船力的方向矩阵，根据拖船的方向角 \( \beta_i(t) \) 动态计算。
   - 这些矩阵用于计算拖船对船舶的实际作用力，这是控制输入 \( \tau_{Ti} \) 的一部分。

2. **拖船控制输入 \( \tau_{Ti} \)**:
   - 计算公式: \( \tau_{Ti}(t) = L_{Ti}B_{Ti}(\beta_i(t))F_{i}'(t) \)，其中 \( F_i'(t) \) 是通过拖船施加的力。
   - 这反映了拖船的动力学输入对船舶运动的影响。

3. **环境扰动 \( \tau_{w} \)**:
   - 包括风力和水流对船只的影响。
   - 风力由风速 \( V_{w} \) 和风向 \( \gamma_w \) 决定，公式包括船体的侧向和纵向投影面积，以及空气密度等因素。

### 代码实现方案

1. **定义拖船力和方向矩阵**:
   - 我们可以编写函数来计算 \( L_{Ti} \) 和 \( B_{Ti} \)，并根据拖船的实时位置和方向动态调整。

2. **计算拖船对船的作用力**:
   - 根据上述矩阵和拖船的力计算总控制输入。

3. **模拟环境扰动**:
   - 定义函数以模拟风力和水流对船只的影响。这可能包括考虑风速、风向以及船只的方向和速度。

4. **整合拖船力和环境力**:
   - 在动力学方程中结合这些力来模拟船只的运动。

现在，我将开始在代码解释器中实现这些函数。首先，我将定义计算 \( L_{Ti} \) 和 \( B_{Ti} \) 的函数。


def force_arm_matrix(lx, ly):
    """
    Calculate the force arm matrix L_Ti for a tug with respect to the ship body frame.
    
    Parameters:
    lx : float
        Longitudinal distance from the towing point to the ship's center of gravity.
    ly : float
        Lateral distance from the towing point to the ship's center of gravity.
    
    Returns:
    numpy.ndarray
        The force arm matrix L_Ti.
    """
    L_Ti = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [lx, ly, 1]])
    return L_Ti

def direction_matrix(beta):
    """
    Calculate the direction matrix B_Ti for a tug's force direction.
    
    Parameters:
    beta : float
        The direction angle of the tow force, in radians.
    
    Returns:
    numpy.ndarray
        The direction matrix B_Ti.
    """
    B_Ti = np.array([[np.cos(beta), 0],
                     [np.sin(beta), 0],
                     [0, 1]])
    return B_Ti

# Example usage
# L_Ti_example = force_arm_matrix(10, 5)
# B_Ti_example = direction_matrix(np.deg2rad(30))
# Uncomment the lines below to test the functions locally
# print("L_Ti_example:\n", L_Ti_example)
# print("B_Ti_example:\n", B_Ti_example)

def tug_force(L_Ti, B_Ti, Fi):
    """
    Calculate the tug force vector τ_Ti given the matrices L_Ti, B_Ti and force Fi.
    
    Parameters:
    L_Ti : numpy.ndarray
        The force arm matrix for the tug.
    B_Ti : numpy.ndarray
        The direction matrix for the tug's force.
    Fi : float
        The magnitude of the force applied by the tug.
    
    Returns:
    numpy.ndarray
        The resultant tug force vector τ_Ti.
    """
    # Force vector in the tug's body frame, only in the 'x' direction
    F_vector = np.array([[Fi], [0], [0]])
    
    # Calculate the force in the ship-body frame
    τ_Ti = L_Ti @ B_Ti @ F_vector
    return τ_Ti.flatten()

# Example usage
# L_Ti_example = force_arm_matrix(10, 5)
# B_Ti_example = direction_matrix(np.deg2rad(30))
# tug_force_example = tug_force(L_Ti_example, B_Ti_example, 1000)
# Uncomment the lines below to test the function locally
# print("Tug force vector τ_Ti_example:\n", tug_force_example)


def wind_disturbance(Vw, gamma_w, psi, A_fw, A_lw, rho_a, Cx, Cy):
    """
    Calculate the wind disturbance forces on the ship.
    
    Parameters:
    Vw : float
        Wind speed (m/s).
    gamma_w : float
        Wind direction relative to the north (radians).
    psi : float
        Ship's heading relative to the north (radians).
    A_fw : float
        Frontal projected area of the vessel (m^2).
    A_lw : float
        Lateral projected area of the vessel (m^2).
    rho_a : float
        Air density (kg/m^3).
    Cx : float
        Drag coefficient in the X-direction.
    Cy : float
        Drag coefficient in the Y-direction.
    
    Returns:
    numpy.ndarray
        The wind force vector [Fx, Fy, Mz] acting on the ship.
    """
    # Relative wind direction w.r.t the ship
    gamma_rel = gamma_w - psi
    
    # Wind force components in the world frame
    Fx_w = 0.5 * rho_a * Vw**2 * Cx * A_fw * np.cos(gamma_rel)
    Fy_w = 0.5 * rho_a * Vw**2 * Cy * A_lw * np.sin(gamma_rel)
    
    # Convert wind forces to the ship-body frame
    Fx_b = Fx_w * np.cos(psi) + Fy_w * np.sin(psi)
    Fy_b = -Fx_w * np.sin(psi) + Fy_w * np.cos(psi)
    
    # Moment due to wind around the Z-axis (assuming lever arm length is negligible for simplicity)
    Mz_b = 0  # This is a simplification and could be refined based on further details.
    
    return np.array([Fx_b, Fy_b, Mz_b])

# Example usage:
# Vw = 10  # Wind speed in m/s
# gamma_w = np.deg2rad(45)  # Wind direction in radians
# psi = np.deg2rad(30)  # Ship heading in radians
# A_fw = 20  # Frontal area in square meters
# A_lw = 30  # Lateral area in square meters
# rho_a = 1.225  # Air density in kg/m^3
# Cx = 1.2  # Drag coefficient in the X-direction
# Cy = 1.0  # Drag coefficient in the Y-direction
# wind_disturbance(Vw, gamma_w, psi, A_fw, A_lw, rho_a, Cx, Cy)



import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 假设所有函数已经定义并且被导入

# 参数设置
M = np.eye(3) * 1000  # 质量矩阵
D = np.eye(3) * 0.1  # 阻尼矩阵
C = np.zeros((3, 3))  # 科里奥利矩阵

# 拖船和环境参数
L_Ti = force_arm_matrix(10, 5)
B_Ti = direction_matrix(np.deg2rad(45))
Fi = 1500  # 拖船力
Vw = 10  # 风速 (m/s)
gamma_w = np.deg2rad(45)  # 风向 (radians)
psi = np.deg2rad(30)  # 船舶朝向 (radians)
A_fw = 20  # 前投影面积
A_lw = 30  # 侧投影面积
rho_a = 1.225  # 空气密度
Cx = 1.2  # X方向阻力系数
Cy = 1.0  # Y方向阻力系数

# 初始状态
initial_state = np.array([0, 0, 0, 1, 0, 0])
t_span = (0, 3600)  # 模拟时间

# 定义ODE系统
def ship_system(t, y):
    eta = y[:3]
    v = y[3:]
    tau_ti = tug_force(L_Ti, B_Ti, Fi)
    tau_w = wind_disturbance(Vw, gamma_w, psi, A_fw, A_lw, rho_a, Cx, Cy)
    d_eta, d_v = ship_dynamics(t, eta, v, M, C, D, tau_ti, tau_w)
    return np.concatenate((d_eta, d_v))

# 模拟
result = solve_ivp(fun=ship_system, t_span=t_span, y0=initial_state, method='RK45')

# 可视化
plt.plot(result.y[0], result.y[1], label='Ship Trajectory')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.title('Ship Movement Over Time')
plt.legend()
plt.grid(True)
plt.show()



这些图片描述了一个多船只控制系统的控制框架，涉及优化器、自适应权重计算器和多个参考计算单元，以协调多拖船的控制任务。接下来是如何将这些控制理论转化为代码实现的分析：

### 关键概念和组件

1. **多船只监督控制器**:
   - 控制多个拖船以实现对主船的协调控制。
   - 包括优化器和自适应权重计算器。

2. **优化器**:
   - 计算拖力和角度，使得成本函数最小化。
   - 成本函数是根据船只当前位置与目标位置的偏差和当前速度与目标速度的偏差加权求和。

3. **自适应权重计算器**:
   - 根据船只接近目标位置的程度动态调整位置和速度在成本函数中的权重。
   - 初始阶段，更注重速度的平稳；接近目标时，更注重位置的精确。

4. **参考计算单元**:
   - 每个拖船根据优化器和自适应权重计算器的输出来计算其应施加的力和角度。

### 实现步骤

1. **定义成本函数**:
   - 根据船只的当前状态和目标状态，计算位置偏差和速度偏差的加权和。

2. **实现优化器**:
   - 使用数值优化方法（如梯度下降法）来调整拖船的力和角度，最小化成本函数。

3. **编写自适应权重计算器**:
   - 根据船只的进展动态调整权重，例如使用逻辑函数或其他数学模型来调整权重。

4. **集成各控制单元**:
   - 将优化器和自适应权重计算器的输出用作输入，计算每个拖船的控制指令。

5. **模拟与测试**:
   - 在模拟环境中测试控制系统的性能，调整参数以优化控制效果。

### 代码示例：定义成本函数

以下是一个简化的成本函数定义示例，它计算当前状态与目标状态之间的偏差：

```python
import numpy as np

def cost_function(current_position, target_position, current_velocity, target_velocity, weights):
    position_error = np.linalg.norm(current_position - target_position)
    velocity_error = np.linalg.norm(current_velocity - target_velocity)
    cost = weights['position'] * position_error + weights['velocity'] * velocity_error
    return cost
```

### 代码示例：自适应权重计算

以下是一个基本的自适应权重计算器，随着船只接近目标，逐渐调整权重：

```python
def adaptive_weights(distance_to_goal, initial_weight, final_weight):
    # 使用简单的线性插值作为示例
    weight = initial_weight + (final_weight - initial_weight) * (1 - np.exp(-distance_to_goal))
    return weight
```

这些示例为如何将控制理论转化为实用的代码提供了基础。你可以基于这些基础进一步开发和测试整个控制系统。如果需要进一步的帮助，请随时询问！



def cost_function(current_state, target_state, weights):
    """
    Calculate the weighted cost based on the position and velocity errors.
    
    Parameters:
    current_state : dict
        Dictionary containing 'position' and 'velocity' of the ship.
    target_state : dict
        Dictionary containing 'position' and 'velocity' of the ship.
    weights : dict
        Dictionary containing 'position' and 'velocity' weights.
    
    Returns:
    float
        The calculated cost.
    """
    position_error = np.linalg.norm(current_state['position'] - target_state['position'])
    velocity_error = np.linalg.norm(current_state['velocity'] - target_state['velocity'])
    cost = weights['position'] * position_error + weights['velocity'] * velocity_error
    return cost


def optimizer(current_state, target_state, initial_guess, max_iter=100, learning_rate=0.01):
    """
    Simple gradient descent optimizer to find the optimal force and angle.
    
    Parameters:
    current_state : dict
        Current position and velocity of the ship.
    target_state : dict
        Desired position and velocity of the ship.
    initial_guess : dict
        Initial guess for the force and angle.
    max_iter : int
        Maximum number of iterations for the gradient descent.
    learning_rate : float
        Learning rate for the gradient descent.
    
    Returns:
    dict
        Optimal force and angle.
    """
    force, angle = initial_guess['force'], initial_guess['angle']
    for _ in range(max_iter):
        # This is a placeholder for the actual gradient calculation
        gradient = np.random.randn(2)  # Random gradient for demonstration
        force -= learning_rate * gradient[0]
        angle -= learning_rate * gradient[1]
    return {'force': force, 'angle': angle}



def adaptive_weights(distance_to_goal, initial_weight, final_weight):
    """
    Calculate adaptive weights based on the distance to the goal.
    
    Parameters:
    distance_to_goal : float
        Distance to the target position.
    initial_weight : float
        Initial weight.
    final_weight : float
        Final weight after reaching the goal.
    
    Returns:
    float
        Adaptively calculated weight.
    """
    weight = initial_weight + (final_weight - initial_weight) * (1 - np.exp(-distance_to_goal))
    return weight


def multi_tug_control_system(current_state, target_state, initial_guess):
    """
    Simulate the multi-tug control system.
    
    Parameters:
    current_state : dict
        Current state of the ship including position and velocity.
    target_state : dict
        Target state of the ship including position and velocity.
    
    Returns:
    dict
        The control outputs for each tug.
    """
    # Compute distance to goal
    distance_to_goal = np.linalg.norm(current_state['position'] - target_state['position'])
    
    # Calculate adaptive weights
    position_weight = adaptive_weights(distance_to_goal, 1, 0.1)
    velocity_weight = adaptive_weights(distance_to_goal, 0.1, 1)
    weights = {'position': position_weight, 'velocity': velocity_weight}
    
    # Opt imize the control inputs
    optimal_control = optimizer(current_state, target_state, initial_guess)
    
    return optimal_control
```

这些代码块为实现多拖船控制系统提供了一个基本的框架。你可以在本地环境中测试这些函数，并根据需要调整参数和算法。如果需要更多帮助或有任何疑问，请随时联系我。


import numpy as np
import matplotlib.pyplot as plt

def cost_function(current_state, target_state, weights):
    position_error = np.linalg.norm(current_state['position'] - target_state['position'])
    velocity_error = np.linalg.norm(current_state['velocity'] - target_state['velocity'])
    return weights['position'] * position_error + weights['velocity'] * velocity_error

def optimizer(current_state, target_state, initial_guess, max_iter=100, learning_rate=0.01):
    force, angle = initial_guess['force'], initial_guess['angle']
    for _ in range(max_iter):
        gradient = np.random.randn(2)  # Random gradient for demonstration
        force -= learning_rate * gradient[0]
        angle -= learning_rate * gradient[1]
    return {'force': force, 'angle': angle}

def adaptive_weights(distance_to_goal, initial_weight, final_weight):
    return initial_weight + (final_weight - initial_weight) * (1 - np.exp(-distance_to_goal))

def multi_tug_control_system(current_state, target_state, initial_guess):
    distance_to_goal = np.linalg.norm(current_state['position'] - target_state['position'])
    position_weight = adaptive_weights(distance_to_goal, 1, 0.1)
    velocity_weight = adaptive_weights(distance_to_goal, 0.1, 1)
    weights = {'position': position_weight, 'velocity': velocity_weight}
    return optimizer(current_state, target_state, initial_guess, max_iter=100, learning_rate=0.01)

# Define the initial and target states
current_state = {
    'position': np.array([0, 0]),
    'velocity': np.array([1, 1])
}
target_state = {
    'position': np.array([100, 100]),
    'velocity': np.array([0, 0])
}
initial_guess = {'force': 10, 'angle': np.pi / 4}

# Simulate the control system
optimal_control = multi_tug_control_system(current_state, target_state, initial_guess)

# Print the results
print("Optimal Control Output:", optimal_control)

# Plotting the trajectory (simplified demonstration)
plt.figure()
plt.title('Simulated Tug Trajectory')
plt.plot([0, 100], [0, 100], 'bo-', label='Desired Path')
plt.quiver(0, 0, optimal_control['force'] * np.cos(optimal_control['angle']), 
           optimal_control['force'] * np.sin(optimal_control['angle']), 
           angles='xy', scale_units='xy', scale=1, color='r', label='Control Force Direction')
plt.xlim(-10, 110)
plt.ylim(-10, 110)
plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.legend()
plt.grid(True)
plt.show()


这两张图片详细介绍了用于船舶拖曳系统中的多层控制策略，包括较高层的路径规划和较低层的推进器力和角度控制。现在，我们将讨论如何实现这些控制策略，具体包括以下几个关键部分：

### 高层控制（路径规划）
高层控制负责计算船舶的参考轨迹和拖船的作用力及角度，以便使船只从当前位置移动到目标位置。使用一系列计算步骤来达成此目标，包括：
1. **计算E矩阵和F矩阵**：这些矩阵与拖船相对于船舶的位置和角度相关。
2. **计算船舶的期望位置和速度**：根据当前位置和速度，以及目标位置和速度，使用自适应权重来平衡路径规划中的速度和位置精度。

### 较低层控制（推进器控制）
较低层控制负责根据高层提供的参考轨迹计算推进器的具体力和角度，确保船只能够沿着计划路径移动。这包括：
1. **计算推进器力和角度**：使用成本函数最小化的方式，结合实际拖船的位置、速度和环境因素，计算推进器的最优力和角度。
2. **环境扰动和约束**：考虑风力和其他环境因素的扰动，以及推进器力的物理限制。

### 实现步骤和代码示例

#### 高层控制器的实现
假设我们已经有了一个计算E矩阵和F矩阵的函数，以及计算船舶目标位置的函数。我们可以这样实现：

```python
import numpy as np

def compute_E_F_matrices(psi, alpha):
    """计算E和F矩阵，代表拖船对船舶的影响"""
    E = np.array([
        [-np.sin(psi + alpha), np.cos(psi + alpha), 0],
        [-np.sin(psi + alpha), np.cos(psi + alpha), 0],
        [0, 0, 1]
    ])
    F = np.array([
        [-np.sin(psi + alpha), np.cos(psi + alpha), 0]
    ])
    return E, F

def high_layer_control(current_position, target_position, current_velocity, target_velocity):
    """计算拖船的作用力和角度"""
    psi = np.arctan2(target_position[1] - current_position[1], target_position[0] - current_position[0])
    alpha = 0.1  # 假设的拖船角度
    E, F = compute_E_F_matrices(psi, alpha)
    return E, F
```

#### 低层控制器的实现
低层控制器接收高层计算的E和F矩阵，然后计算具体的推进器力和角度：

```python
def lower_layer_control(E, F, target_trajectory, environmental_factors):
    """计算推进器的力和角度以跟踪目标轨迹"""
    # 此处我们需要一个优化算法来最小化成本函数，并考虑环境扰动
    # 这里简化表示
    thrust_force = np.linalg.inv(E).dot(target_trajectory - environmental_factors)
    return thrust_force
```

### 测试和验证
这些函数的实现提供了一个基础框架。在真实的应用中，还需要集成真实的环境数据，进行更详细的算法开发和参数调优，以及进行广泛的模拟


def calculate_target_heading(current_position, target_position):
    """计算目标航向角"""
    return np.arctan2(target_position[1] - current_position[1], target_position[0] - current_position[0])

def calculate_adaptive_weights(distance_to_goal, initial_weight, final_weight):
    """计算自适应权重"""
    return initial_weight + (final_weight - initial_weight) * (1 - np.exp(-distance_to_goal))

def high_layer_control(current_state, target_state, psi, gamma):
    """
    高层控制算法实现，生成目标轨迹和拖船控制参数。
    :param current_state: 当前状态，包括位置和速度
    :param target_state: 目标状态，包括位置和速度
    :param psi: 当前船舶航向
    :param gamma: 环境扰动角度
    :return: 拖船力和角度
    """
    # 计算目标航向和当前航向之间的差异
    psi_d = calculate_target_heading(current_state['position'], target_state['position'])
    beta = psi_d - psi  # 船舶与拖船间的角度差

    # 自适应权重计算
    distance_to_goal = np.linalg.norm(current_state['position'] - target_state['position'])
    weight_position = calculate_adaptive_weights(distance_to_goal, 1, 0.1)
    weight_velocity = calculate_adaptive_weights(distance_to_goal, 0.1, 1)
    weights = {'position': weight_position, 'velocity': weight_velocity}

    # 这里简化控制计算
    control_input = {'force': weight_position * 1000, 'angle': beta}  # 假设力的大小和角度

    return control_input


def lower_layer_control(target_control, current_velocity, max_force):
    """
    低层控制算法实现，计算推进器的实际力和角度。
    :param target_control: 高层控制器提供的目标控制输入
    :param current_velocity: 当前船舶速度
    :param max_force: 推进器的最大力
    :return: 推进器实际力和角度
    """
    # 这里假设一个简单的控制算法，实际可能需要PID控制器或其他复杂算法
    actual_force = min(target_control['force'], max_force)
    actual_angle = target_control['angle']  # 直接使用高层控制器的角度

    return {'force': actual_force, 'angle': actual_angle}


def calculate_ef_matrices(ship_psi, tug_relative_angle):
    """ 根据船舶和拖船的角度关系计算E和F矩阵 """
    E = np.array([
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0],
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0],
        [0, 0, 1]
    ])
    F = np.array([
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0]
    ])
    return E, F



def control_thruster(tug_force, tug_angle, max_force, max_angle):
    """ 计算实际推进器输出，确保不超过最大限制 """
    actual_force = min(tug_force, max_force)
    actual_angle = max(min(tug_angle, max_angle), -max_angle)
    return actual_force, actual_angle


import numpy as np

def calculate_ef_matrices(ship_psi, tug_relative_angle):
    E = np.array([
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0],
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0],
        [0, 0, 1]
    ])
    F = np.array([
        [-np.sin(ship_psi + tug_relative_angle), np.cos(ship_psi + tug_relative_angle), 0]
    ])
    return E, F

def control_thruster(tug_force, tug_angle, max_force, max_angle):
    actual_force = min(tug_force, max_force)
    actual_angle = max(min(tug_angle, max_angle), -max_angle)
    return actual_force, actual_angle

def simulate_ship_movement(initial_position, target_position, initial_velocity, ship_heading, max_force, max_angle):
    current_position = np.array(initial_position)
    target_velocity = np.array([0, 0])  # Assuming the ship should stop at the target
    current_velocity = np.array(initial_velocity)
    
    # Simulation parameters
    time_steps = 100
    time_interval = 1  # one unit time per step

    for step in range(time_steps):
        ship_psi = np.arctan2(current_velocity[1], current_velocity[0])  # Direction of current velocity
        relative_angle = ship_heading - ship_psi
        E, F = calculate_ef_matrices(ship_psi, relative_angle)
        
        # Simple control logic to reduce the position error
        position_error = np.linalg.norm(target_position - current_position)
        tug_force = position_error * 10  # Simplified proportional control
        tug_angle = relative_angle  # Align force with relative angle

        # Calculate thruster output
        actual_force, actual_angle = control_thruster(tug_force, tug_angle, max_force, max_angle)
        
        # Update ship position and velocity (simplified physics)
        current_velocity += (actual_force * np.array([np.cos(actual_angle), np.sin(actual_angle)])) * time_interval
        current_position += current_velocity * time_interval

        print(f"Time Step {step}: Position {current_position}, Velocity {current_velocity}")

        if position_error < 1.0:
            print("Target reached.")
            break

# Example usage
simulate_ship_movement([0, 0], [100, 100], [10, 5], np.pi / 4, 100, np.pi / 6)


import numpy as np

# 参数设置
params = {
    'l_tow1': 1.0,
    'l_tow2': 1.0,
    'l_s1': 0.67,
    'l_s2': 0.585,
    'l_t1': 0.5,
    'l_t2': 0.5,
    'F_max1': 3,
    'F_max2': 3,
    'T_max1': 10,
    'T_max2': 10,
    'a_max1': 5,
    'a_max2': 5,
    'F_rate1': 0.1,
    'F_rate2': 0.1
}

def simulate_tug_control(strategy='A'):
    # 初始化状态
    position = np.array([0, 0])
    velocity = np.array([0.1, 0.1])
    target_position = np.array([100, 100])
    target_velocity = np.array([0, 0])
    
    # 模拟过程中记录
    trajectory = [position.copy()]
    
    for _ in range(1000):
        if strategy == 'A':
            # 使用固定权重
            weights = np.array([150, 150])
        elif strategy == 'B':
            # 使用自适应权重
            distance_to_goal = np.linalg.norm(target_position - position)
            weights = np.array([150 + 0.15 * distance_to_goal, 150 + 0.01 * distance_to_goal])
        
        # 假设控制输入计算（简化）
        control_input = weights * (target_position - position) / np.linalg.norm(target_position - position)
        control_input = np.clip(control_input, -params['F_max1'], params['F_max1'])
        
        # 更新状态
        velocity += control_input
        position += velocity * 0.1
        trajectory.append(position.copy())
        
        if np.linalg.norm(target_position - position) < 1:
            print("Target reached.")
            break
    
    return trajectory

# 运行模拟
trajectory_A = simulate_tug_control('A')
trajectory_B = simulate_tug_control('B')

# 绘图显示结果
import matplotlib.pyplot as plt

trajectory_A = np.array(trajectory_A)
trajectory_B = np.array(trajectory_B)

plt.figure(figsize=(10, 5))
plt.plot(trajectory_A[:, 0], trajectory_A[:, 1], label='Trajectory A (Fixed Weights)')
plt.plot(trajectory_B[:, 0], trajectory_B[:, 1], label='Trajectory B (Adaptive Weights)')
plt.scatter([100], [100], color='red', label='Target')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Tug Trajectories under Different Control Strategies')
plt.legend()
plt.grid(True)
plt.show()



为了集成并完整实现拖轮辅助船舶靠泊的仿真程序设计方法，我们需要从高层设计开始，确保每一部分都紧密协作以模拟整个靠泊过程。这包括确定控制结构、参数化模型、实现算法，并最后进行测试和验证。下面是整合之前讨论内容的步骤和方法概览。

### 1. **系统架构设计**
首先，明确控制系统的层级结构，包括：
- **高层控制器**：负责生成目标轨迹，并计算拖船应施加的力和角度，以便将船只从当前位置导向目标位置。
- **低层控制器**：根据高层控制器提供的目标轨迹和控制参数，计算实际的推进器力和角度，确保船舶按计划路径移动。

### 2. **参数和模型的定义**
基于提供的技术参数（如拖船力、角度变化速率等），定义模型：
- **拖船和船舶的动力学模型**：涉及计算船舶和拖船间的相对位置、力的作用点以及受力影响。
- **环境模型**：包括风力、水流等环境因素对船舶行为的影响。

### 3. **控制策略和算法实现**
实现高层和低层的控制逻辑：
- **自适应控制权重**：根据船舶与目标位置的距离动态调整，确保控制的平滑过渡。
- **成本函数优化**：使用优化算法（如梯度下降法）最小化成本函数，确定最优控制输入。

### 4. **集成和接口设计**
确保所有组件能够无缝交互：
- **数据流管理**：确保高层和低层控制器之间的信息流是准确和及时的。
- **事件处理**：对于特定事件（如达到目标区域、遇到突发环境变化），进行适当的响应。

### 5. **测试和验证**
进行广泛的仿真测试来评估控制系统的性能：
- **场景测试**：在不同的环境条件和船舶配置下测试系统性能。
- **鲁棒性测试**：评估系统对参数变化和外部扰动的敏感度和适应能力。
- **性能分析**：通过对比不同控制策略（如固定权重与自适应权重）的结果，评估哪种方法更有效。

### 6. **部署和操作**
将测试通过的系统应用于实际的船舶靠泊仿真，监控其操作并根据需要进行调整。

### 实现示例代码整合
可以是将之前讨论的各部分代码合并，并确保它们在同一环境下协同工作。例如，高层控制器输出的目标轨迹和控制参数应直接传递给低层控制器来计算实际的推进器输出。

### 结论
成功实现这个仿真程序需要对控制理论、船舶动力学、以及软件工程有深入的理解。通过迭代开发和测试，可以逐步完善系统设计，确保其能在各种条件下稳定运行。这不仅是技术上的挑战，也是对项目管理和团队协作能力的考验。


